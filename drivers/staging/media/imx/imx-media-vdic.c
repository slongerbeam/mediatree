// SPDX-License-Identifier: GPL-2.0+
/*
 * V4L2 Deinterlacer Subdev for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2017 Mentor Graphics Inc.
 */
#include <linux/interrupt.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/imx.h>
#include "imx-media.h"

/*
 * This subdev implements two different video pipelines:
 *
 * CSI -> VDIC
 *
 * In this pipeline, the CSI sends a single interlaced field F(n-1)
 * directly to the VDIC (and optionally the following field F(n)
 * can be sent to memory via IDMAC channel 13). This pipeline only works
 * in VDIC's high motion mode, which only requires a single field for
 * processing. The other motion modes (low and medium) require three
 * fields, so this pipeline does not work in those modes. Also, it is
 * not clear how this pipeline can deal with the various field orders
 * (sequential BT/TB, interlaced BT/TB).
 *
 * MEM -> CH8,9,10 -> VDIC
 *
 * In this pipeline, previous field F(n-1), current field F(n), and next
 * field F(n+1) are transferred to the VDIC via IDMAC channels 8,9,10.
 * These memory buffers can come from a video output or mem2mem device.
 * All motion modes are supported by this pipeline.
 *
 * The "direct" CSI->VDIC pipeline requires no DMA, but it can only be
 * used in high motion mode.
 */

struct vdic_priv;

struct vdic_pipeline_ops {
	int (*setup)(struct vdic_priv *priv);
	int (*start)(struct vdic_priv *priv);
	void (*stop)(struct vdic_priv *priv);
	void (*disable)(struct vdic_priv *priv);
};

/*
 * Min/Max supported width and heights.
 */
#define MIN_W       176
#define MIN_H       144
#define MAX_W_VDIC  968
#define MAX_H_VDIC 2048
#define W_ALIGN    4 /* multiple of 16 pixels */
#define H_ALIGN    1 /* multiple of 2 lines */
#define S_ALIGN    1 /* multiple of 2 */

struct vdic_priv {
	struct device *ipu_dev;
	struct ipu_soc *ipu;

	struct v4l2_subdev   sd;
	struct media_pad pad[VDIC_NUM_PADS];

	/* lock to protect all members below */
	struct mutex lock;

	/* IPU units we require */
	struct ipu_vdi *vdi;
	struct ipu_ic *ic;

	int active_input_pad;
	int active_output_pad;

	struct ipuv3_channel *vdi_in_ch_p; /* F(n-1) transfer channel */
	struct ipuv3_channel *vdi_in_ch;   /* F(n) transfer channel */
	struct ipuv3_channel *vdi_in_ch_n; /* F(n+1) transfer channel */
	struct ipuv3_channel *vdi_out_ch;  /* progressive output channel */

	/* active input buffer to send to idmac sink pad */
	struct vb2_v4l2_buffer *active_srcbuf;
	/* active output buffer to send to idmac src pad */
	struct vb2_v4l2_buffer *active_dstbuf;

	/* pipeline operations */
	struct vdic_pipeline_ops *ops;

	/* current and previous input buffers indirect path */
	struct vb2_v4l2_buffer *curr_in_buf;
	struct vb2_v4l2_buffer *prev_in_buf;

	/*
	 * translated field type, input line stride, and field size
	 * for indirect path
	 */
	u32 fieldtype;
	u32 in_stride;
	u32 field_size;

	/* the source (a video device or subdev) */
	struct media_entity *src;
	/* the sink that will receive the progressive out buffers */
	struct media_entity *sink;

	struct v4l2_mbus_framefmt format_mbus[VDIC_NUM_PADS];
	const struct imx_media_pixfmt *cc[VDIC_NUM_PADS];
	struct v4l2_fract frame_interval[VDIC_NUM_PADS];

	/* the video device connected to idmac input pad */
	struct imx_media_video_dev *vdev_src;
	struct v4l2_format vdev_src_fmt;
	struct v4l2_rect vdev_compose;
	/* the video device connected to idmac output pad */
	struct imx_media_video_dev *vdev_sink;
	struct v4l2_format vdev_sink_fmt;
	/* the open context for indirect path */
	void *vdev_ctx;

	/* using direct CSI->VDIC->IC pipeline (direct input pad) */
	bool csi_direct;

	/* motion select control */
	struct v4l2_ctrl_handler ctrl_hdlr;
	enum ipu_motion_sel motion;

	spinlock_t irqlock; /* protect eof_irq handler */

	int eof_irq;
	int nfb4eof_irq;

	int stream_count;
	u32 frame_sequence; /* frame sequence counter */
	bool nfb4eof;       /* NFB4EOF encountered during streaming */
};

static void vdic_put_ipu_resources(struct vdic_priv *priv)
{
	if (priv->vdi_in_ch_p)
		ipu_idmac_put(priv->vdi_in_ch_p);
	priv->vdi_in_ch_p = NULL;

	if (priv->vdi_in_ch)
		ipu_idmac_put(priv->vdi_in_ch);
	priv->vdi_in_ch = NULL;

	if (priv->vdi_in_ch_n)
		ipu_idmac_put(priv->vdi_in_ch_n);
	priv->vdi_in_ch_n = NULL;

	if (priv->vdi_out_ch)
		ipu_idmac_put(priv->vdi_out_ch);
	priv->vdi_out_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->ic))
		ipu_ic_put(priv->ic);
	priv->ic = NULL;

	if (!IS_ERR_OR_NULL(priv->vdi))
		ipu_vdi_put(priv->vdi);
	priv->vdi = NULL;
}

static int vdic_get_ipu_resources(struct vdic_priv *priv)
{
	int ret, err_chan;
	struct ipuv3_channel *ch;
	struct ipu_vdi *vdi;
	struct ipu_ic *ic;

	vdi = ipu_vdi_get(priv->ipu);
	if (IS_ERR(vdi)) {
		v4l2_err(&priv->sd, "failed to get VDIC\n");
		ret = PTR_ERR(vdi);
		goto out;
	}
	priv->vdi = vdi;

	if (!priv->csi_direct) {
		ch = ipu_idmac_get(priv->ipu, IPUV3_CHANNEL_MEM_VDI_PREV);
		if (IS_ERR(ch)) {
			err_chan = IPUV3_CHANNEL_MEM_VDI_PREV;
			ret = PTR_ERR(ch);
			goto out_err_chan;
		}
		priv->vdi_in_ch_p = ch;

		ch = ipu_idmac_get(priv->ipu, IPUV3_CHANNEL_MEM_VDI_CUR);
		if (IS_ERR(ch)) {
			err_chan = IPUV3_CHANNEL_MEM_VDI_CUR;
			ret = PTR_ERR(ch);
			goto out_err_chan;
		}
		priv->vdi_in_ch = ch;

		ch = ipu_idmac_get(priv->ipu, IPUV3_CHANNEL_MEM_VDI_NEXT);
		if (IS_ERR(ch)) {
			err_chan = IPUV3_CHANNEL_MEM_VDI_NEXT;
			ret = PTR_ERR(ch);
			goto out_err_chan;
		}
		priv->vdi_in_ch_n = ch;

		ic = ipu_ic_get(priv->ipu, IC_TASK_VIEWFINDER);
		if (IS_ERR(ic)) {
			v4l2_err(&priv->sd, "failed to get IC\n");
			ret = PTR_ERR(ic);
			goto out;
		}
		priv->ic = ic;

		ch = ipu_idmac_get(priv->ipu, IPUV3_CHANNEL_IC_PRP_VF_MEM);
		if (IS_ERR(ch)) {
			err_chan = IPUV3_CHANNEL_IC_PRP_VF_MEM;
			ret = PTR_ERR(ch);
			goto out_err_chan;
		}
		priv->vdi_out_ch = ch;
	}

	return 0;

out_err_chan:
	v4l2_err(&priv->sd, "could not get IDMAC channel %u\n", err_chan);
out:
	vdic_put_ipu_resources(priv);
	return ret;
}

static void vdi_out_buf_done(struct vdic_priv *priv,
			     enum vb2_buffer_state status)
{
	struct imx_media_video_dev *vdev_sink = priv->vdev_sink;
	struct vb2_v4l2_buffer *srcbuf = priv->active_srcbuf;
	struct vb2_v4l2_buffer *dstbuf = priv->active_dstbuf;
	void *vdev_ctx = priv->vdev_ctx;

	dev_dbg(priv->ipu_dev, "%s: srcbuf %p, dstbuf %p\n", __func__,
		srcbuf, dstbuf);

	priv->frame_sequence++;
	priv->nfb4eof = false;
	priv->active_srcbuf = NULL;
	priv->active_dstbuf = NULL;
	priv->vdev_ctx = NULL;

	if (srcbuf && dstbuf && vdev_ctx) {
		srcbuf->sequence = priv->frame_sequence;
		dstbuf->sequence = srcbuf->sequence;
		vdev_sink->ops->buf_done(vdev_sink, vdev_ctx, dstbuf, status);
	}
}

static irqreturn_t vdi_out_idmac_eof_interrupt(int irq, void *dev_id)
{
	struct vdic_priv *priv = dev_id;
	enum vb2_buffer_state status;

	spin_lock(&priv->irqlock);

	priv->ops->stop(priv);

	status = priv->nfb4eof ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE;

	vdi_out_buf_done(priv, status);

	spin_unlock(&priv->irqlock);
	return IRQ_HANDLED;
}

static irqreturn_t vdi_out_idmac_nfb4eof_interrupt(int irq, void *dev_id)
{
	struct vdic_priv *priv = dev_id;

	spin_lock(&priv->irqlock);

	/*
	 * this is not an unrecoverable error, just mark
	 * the next captured frame with error flag.
	 */
	priv->nfb4eof = true;

	v4l2_err(&priv->sd, "NFB4EOF\n");

	spin_unlock(&priv->irqlock);

	return IRQ_HANDLED;
}

static int prepare_vdic_in_buffers(struct vdic_priv *priv)
{
	dma_addr_t prev_phys, curr_phys, next_phys;
	struct vb2_v4l2_buffer *curr, *prev;
	struct vb2_buffer *curr_vb, *prev_vb;
	u32 fs = priv->field_size;
	u32 is = priv->in_stride;

	curr = priv->active_srcbuf;
	if (!curr)
		return -EIO;

	/* current input buffer is now previous */
	priv->prev_in_buf = priv->curr_in_buf;
	priv->curr_in_buf = curr;
	prev = priv->prev_in_buf ? priv->prev_in_buf : curr;

	prev_vb = &prev->vb2_buf;
	curr_vb = &curr->vb2_buf;

	switch (priv->fieldtype) {
	case V4L2_FIELD_SEQ_TB:
	case V4L2_FIELD_SEQ_BT:
		prev_phys = vb2_dma_contig_plane_dma_addr(prev_vb, 0) + fs;
		curr_phys = vb2_dma_contig_plane_dma_addr(curr_vb, 0);
		next_phys = vb2_dma_contig_plane_dma_addr(curr_vb, 0) + fs;
		break;
	case V4L2_FIELD_INTERLACED_TB:
	case V4L2_FIELD_INTERLACED_BT:
	case V4L2_FIELD_INTERLACED:
		prev_phys = vb2_dma_contig_plane_dma_addr(prev_vb, 0) + is;
		curr_phys = vb2_dma_contig_plane_dma_addr(curr_vb, 0);
		next_phys = vb2_dma_contig_plane_dma_addr(curr_vb, 0) + is;
		break;
	default:
		/*
		 * can't get here, priv->fieldtype can only be one of
		 * the above. This is to quiet smatch errors.
		 */
		return -EINVAL;
	}

	ipu_cpmem_set_buffer(priv->vdi_in_ch_p, 0, prev_phys);
	ipu_cpmem_set_buffer(priv->vdi_in_ch,   0, curr_phys);
	ipu_cpmem_set_buffer(priv->vdi_in_ch_n, 0, next_phys);

	ipu_idmac_select_buffer(priv->vdi_in_ch_p, 0);
	ipu_idmac_select_buffer(priv->vdi_in_ch, 0);
	ipu_idmac_select_buffer(priv->vdi_in_ch_n, 0);

	return 0;
}

static int prepare_vdic_out_buffer(struct vdic_priv *priv)
{
	struct vb2_v4l2_buffer *curr;
	dma_addr_t phys;

	curr = priv->active_dstbuf;
	if (!curr)
		return -EIO;

	phys = vb2_dma_contig_plane_dma_addr(&curr->vb2_buf, 0);

	ipu_cpmem_set_buffer(priv->vdi_out_ch, 0, phys);

	ipu_idmac_select_buffer(priv->vdi_out_ch, 0);

	return 0;
}

static int setup_vdic_in_channel(struct vdic_priv *priv,
				 struct ipuv3_channel *channel)
{
	unsigned int burst_size;
	struct ipu_image image;
	int ret;

	ipu_cpmem_zero(channel);

	memset(&image, 0, sizeof(image));
	image.pix = priv->vdev_src_fmt.fmt.pix;
	image.rect = priv->vdev_compose;
	/* one field to VDIC channels */
	image.pix.height /= 2;
	image.rect.height /= 2;

	ret = ipu_cpmem_set_image(channel, &image);
	if (ret)
		return ret;

	burst_size = (image.pix.width & 0xf) ? 8 : 16;
	ipu_cpmem_set_burstsize(channel, burst_size);

	ipu_cpmem_set_axi_id(channel, 1);

	ipu_idmac_set_double_buffer(channel, false);

	return 0;
}

static int setup_vdic_out_channel(struct vdic_priv *priv,
				  struct ipuv3_channel *channel)
{
	unsigned int burst_size;
	struct ipu_image image;
	int ret;

	ipu_cpmem_zero(channel);

	memset(&image, 0, sizeof(image));
	image.pix = priv->vdev_sink_fmt.fmt.pix;
	image.rect.width = image.pix.width;
	image.rect.height = image.pix.height;

	switch (image.pix.pixelformat) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
		/* Skip writing U and V components to odd rows */
		ipu_cpmem_skip_odd_chroma_rows(channel);
		break;
	}

	ret = ipu_cpmem_set_image(channel, &image);
	if (ret) {
		v4l2_err(&priv->sd, "ipu_cpmem_set_image failed: %d\n", ret);
		return ret;
	}

	burst_size = (image.pix.width & 0xf) ? 8 : 16;
	ipu_cpmem_set_burstsize(channel, burst_size);

	ipu_cpmem_set_axi_id(channel, 1);

	ipu_idmac_set_double_buffer(channel, false);

	return 0;
}

static int vdic_out_setup(struct vdic_priv *priv)
{
	struct imx_media_video_dev *vdev_sink = priv->vdev_sink;
	int ret;

	priv->frame_sequence = 0;

	priv->vdev_sink_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vdev_sink->ops->get_fmt(vdev_sink, &priv->vdev_sink_fmt, NULL, NULL);

	ret = setup_vdic_out_channel(priv, priv->vdi_out_ch);
	if (ret)
		return ret;

	priv->nfb4eof_irq = ipu_idmac_channel_irq(priv->ipu,
						  priv->vdi_out_ch,
						  IPU_IRQ_NFB4EOF);
	ret = devm_request_irq(priv->ipu_dev, priv->nfb4eof_irq,
			       vdi_out_idmac_nfb4eof_interrupt, 0,
			       "imx-vdic-nfb4eof", priv);
	if (ret) {
		v4l2_err(&priv->sd,
			 "Error registering NFB4EOF irq: %d\n", ret);
		return ret;
	}

	priv->eof_irq = ipu_idmac_channel_irq(priv->ipu, priv->vdi_out_ch,
					      IPU_IRQ_EOF);

	ret = devm_request_irq(priv->ipu_dev, priv->eof_irq,
			       vdi_out_idmac_eof_interrupt, 0,
			       "imx-vdic-eof", priv);
	if (ret) {
		v4l2_err(&priv->sd, "Error registering eof irq: %d\n", ret);
		goto out_free_nfb4eof_irq;
	}

	return 0;

out_free_nfb4eof_irq:
	devm_free_irq(priv->ipu_dev, priv->nfb4eof_irq, priv);
	return ret;
}

static void vdic_out_unsetup(struct vdic_priv *priv)
{
	devm_free_irq(priv->ipu_dev, priv->eof_irq, priv);
	devm_free_irq(priv->ipu_dev, priv->nfb4eof_irq, priv);
}

static int vdic_setup_direct(struct vdic_priv *priv)
{
	/* set VDIC to receive from CSI for direct path */
	ipu_fsu_link(priv->ipu, IPUV3_CHANNEL_CSI_DIRECT,
		     IPUV3_CHANNEL_CSI_VDI_PREV);

	return 0;
}

static int vdic_start_direct(struct vdic_priv *priv)
{
	ipu_vdi_enable(priv->vdi);
	return 0;
}

static void vdic_stop_direct(struct vdic_priv *priv)
{
	ipu_vdi_disable(priv->vdi);
}

static void vdic_disable_direct(struct vdic_priv *priv)
{
	ipu_fsu_unlink(priv->ipu, IPUV3_CHANNEL_CSI_DIRECT,
		       IPUV3_CHANNEL_CSI_VDI_PREV);
}

static int vdic_setup_indirect(struct vdic_priv *priv)
{
	struct imx_media_video_dev *vdev_src = priv->vdev_src;
	struct v4l2_mbus_framefmt *infmt;
	struct v4l2_pix_format *outfmt;
	const struct imx_media_pixfmt *incc;
	struct ipu_ic_csc csc;
	int in_size, ret;

	infmt = &priv->format_mbus[VDIC_SINK_PAD_IDMAC];
	outfmt = &priv->vdev_sink_fmt.fmt.pix;
	incc = priv->cc[VDIC_SINK_PAD_IDMAC];

	priv->vdev_src_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	vdev_src->ops->get_fmt(vdev_src, &priv->vdev_src_fmt,
			       &priv->vdev_compose, NULL);

	in_size = (infmt->width * incc->bpp * infmt->height) >> 3;

	/* 1/2 full image size */
	priv->field_size = in_size / 2;
	priv->in_stride = incc->planar ?
		infmt->width : (infmt->width * incc->bpp) >> 3;

	priv->prev_in_buf = NULL;
	priv->curr_in_buf = NULL;

	priv->fieldtype = infmt->field;

	/* init the vdi-in channels */
	ret = setup_vdic_in_channel(priv, priv->vdi_in_ch_p);
	if (ret)
		return ret;
	ret = setup_vdic_in_channel(priv, priv->vdi_in_ch);
	if (ret)
		return ret;
	ret = setup_vdic_in_channel(priv, priv->vdi_in_ch_n);
	if (ret)
		return ret;

	ret = vdic_out_setup(priv);
	if (ret)
		return ret;

	/* enable the IC */
	ret = ipu_ic_calc_csc(&csc,
			      infmt->ycbcr_enc, infmt->quantization,
			      IPUV3_COLORSPACE_YUV,
			      outfmt->ycbcr_enc, outfmt->quantization,
			      IPUV3_COLORSPACE_YUV);
	if (ret) {
		v4l2_err(&priv->sd, "ipu_ic_calc_csc failed, %d\n",
			 ret);
		return ret;
	}

	ipu_ic_task_init(priv->ic, &csc,
			 infmt->width, infmt->height,
			 infmt->width, infmt->height);
	ipu_ic_task_idma_init(priv->ic, priv->vdi_out_ch,
			      infmt->width, infmt->height, 16, 0);

	return ret;
}

static int vdic_start_indirect(struct vdic_priv *priv)
{
	int ret;

	priv->nfb4eof = false;

	ipu_vdi_enable(priv->vdi);
	ipu_ic_enable(priv->ic);
	ipu_ic_task_enable(priv->ic);

	ret = prepare_vdic_out_buffer(priv);
	if (ret)
		return ret;

	ret = prepare_vdic_in_buffers(priv);
	if (ret)
		return ret;

	/* enable the channels */
	ipu_idmac_enable_channel(priv->vdi_out_ch);
	ipu_idmac_enable_channel(priv->vdi_in_ch_p);
	ipu_idmac_enable_channel(priv->vdi_in_ch);
	ipu_idmac_enable_channel(priv->vdi_in_ch_n);

	return 0;
}

static void vdic_stop_indirect(struct vdic_priv *priv)
{
	ipu_ic_task_disable(priv->ic);

	/* disable channels */
	ipu_idmac_disable_channel(priv->vdi_in_ch_p);
	ipu_idmac_disable_channel(priv->vdi_in_ch);
	ipu_idmac_disable_channel(priv->vdi_in_ch_n);
	ipu_idmac_disable_channel(priv->vdi_out_ch);

	ipu_ic_disable(priv->ic);
	ipu_vdi_disable(priv->vdi);
}

static void vdic_disable_indirect(struct vdic_priv *priv)
{
	vdic_out_unsetup(priv);
}

static struct vdic_pipeline_ops direct_ops = {
	.setup = vdic_setup_direct,
	.start = vdic_start_direct,
	.stop = vdic_stop_direct,
	.disable = vdic_disable_direct,
};

static struct vdic_pipeline_ops indirect_ops = {
	.setup = vdic_setup_indirect,
	.start = vdic_start_indirect,
	.stop = vdic_stop_indirect,
	.disable = vdic_disable_indirect,
};

static u32 vdic_get_top_field_position(u32 field)
{
	switch (field) {
	case V4L2_FIELD_INTERLACED_TB:
	case V4L2_FIELD_SEQ_TB:
		return IPU_VDI_FIELD_CURR; /* BTB order to VDIC */
	case V4L2_FIELD_INTERLACED_BT:
	case V4L2_FIELD_SEQ_BT:
		return IPU_VDI_FIELD_PREV; /* TBT order to VDIC */
	default:
		break;
	}

	return IPU_VDI_FIELD_CURR; /* assume BTB order to VDIC */
}

static int vdic_start(struct vdic_priv *priv)
{
	struct v4l2_mbus_framefmt *infmt;
	int ret;

	infmt = &priv->format_mbus[priv->active_input_pad];

	priv->ops = priv->csi_direct ? &direct_ops : &indirect_ops;

	ret = vdic_get_ipu_resources(priv);
	if (ret)
		return ret;

	/*
	 * init the VDIC.
	 *
	 * note we don't give infmt->code to ipu_vdi_setup(). The VDIC
	 * only supports 4:2:2 or 4:2:0, and this subdev will only
	 * negotiate 4:2:2 at its sink pads.
	 */
	ipu_vdi_setup(priv->vdi, MEDIA_BUS_FMT_UYVY8_2X8,
		      infmt->width, infmt->height);
	ipu_vdi_set_field_order(priv->vdi,
				vdic_get_top_field_position(infmt->field));
	ipu_vdi_set_motion(priv->vdi, priv->motion);

	ret = priv->ops->setup(priv);
	if (ret)
		goto out_put_ipu;

	if (priv->csi_direct) {
		ret = priv->ops->start(priv);
		if (ret)
			goto out_disable;
	}

	return 0;

out_disable:
	priv->ops->disable(priv);
out_put_ipu:
	vdic_put_ipu_resources(priv);
	return ret;
}

static void vdic_stop(struct vdic_priv *priv)
{
	priv->ops->stop(priv);
	if (!priv->csi_direct)
		vdi_out_buf_done(priv, VB2_BUF_STATE_ERROR);
	priv->ops->disable(priv);

	vdic_put_ipu_resources(priv);
}

/*
 * V4L2 subdev operations.
 */

static int vdic_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vdic_priv *priv = container_of(ctrl->handler,
					      struct vdic_priv, ctrl_hdlr);
	enum ipu_motion_sel motion;
	int ret = 0;

	mutex_lock(&priv->lock);

	switch (ctrl->id) {
	case V4L2_CID_DEINTERLACING_MODE:
		motion = ctrl->val;
		if (motion != priv->motion) {
			/* can't change motion control mid-streaming */
			if (priv->stream_count > 0) {
				ret = -EBUSY;
				goto out;
			}
			priv->motion = motion;
		}
		break;
	default:
		v4l2_err(&priv->sd, "Invalid control\n");
		ret = -EINVAL;
	}

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static const struct v4l2_ctrl_ops vdic_ctrl_ops = {
	.s_ctrl = vdic_s_ctrl,
};

static const char * const vdic_ctrl_motion_menu[] = {
	"No Motion Compensation",
	"Low Motion",
	"Medium Motion",
	"High Motion",
};

static int vdic_init_controls(struct vdic_priv *priv)
{
	struct v4l2_ctrl_handler *hdlr = &priv->ctrl_hdlr;
	int ret;

	v4l2_ctrl_handler_init(hdlr, 1);

	v4l2_ctrl_new_std_menu_items(hdlr, &vdic_ctrl_ops,
				     V4L2_CID_DEINTERLACING_MODE,
				     HIGH_MOTION, 0, HIGH_MOTION,
				     vdic_ctrl_motion_menu);

	priv->sd.ctrl_handler = hdlr;

	if (hdlr->error) {
		ret = hdlr->error;
		goto out_free;
	}

	v4l2_ctrl_handler_setup(hdlr);
	return 0;

out_free:
	v4l2_ctrl_handler_free(hdlr);
	return ret;
}

static int vdic_device_run(struct v4l2_subdev *sd, void *ctx)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	struct imx_media_video_dev *vdev_sink = priv->vdev_sink;
	struct imx_media_video_dev *vdev_src = priv->vdev_src;

	if (WARN_ON(priv->active_output_pad != VDIC_SRC_PAD_IDMAC))
		return -EPIPE;
	if (priv->vdev_ctx)
		return -EBUSY;

	priv->vdev_ctx = ctx;

	/* get next queued buffers */
	priv->active_srcbuf =
		vdev_src->ops->get_next_buf(vdev_src, priv->vdev_ctx,
					    V4L2_BUF_TYPE_VIDEO_OUTPUT);
	priv->active_dstbuf =
		vdev_sink->ops->get_next_buf(vdev_sink, priv->vdev_ctx,
					     V4L2_BUF_TYPE_VIDEO_CAPTURE);

	dev_dbg(priv->ipu_dev, "%s: srcbuf %p, dstbuf %p\n", __func__,
		priv->active_srcbuf, priv->active_dstbuf);

	return priv->ops->start(priv);
}

static int vdic_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *src_sd = NULL;
	int ret = 0;

	mutex_lock(&priv->lock);

	if (!priv->src || !priv->sink) {
		ret = -EPIPE;
		goto out;
	}

	if (priv->csi_direct)
		src_sd = media_entity_to_v4l2_subdev(priv->src);

	/*
	 * enable/disable streaming only if stream_count is
	 * going from 0 to 1 / 1 to 0.
	 */
	if (priv->stream_count != !enable)
		goto update_count;

	dev_dbg(priv->ipu_dev, "%s: stream %s\n", sd->name,
		enable ? "ON" : "OFF");

	if (enable)
		ret = vdic_start(priv);
	else
		vdic_stop(priv);
	if (ret)
		goto out;

	if (src_sd) {
		/* start/stop upstream */
		ret = v4l2_subdev_call(src_sd, video, s_stream, enable);
		ret = (ret && ret != -ENOIOCTLCMD) ? ret : 0;
		if (ret) {
			if (enable)
				vdic_stop(priv);
			goto out;
		}
	}

update_count:
	priv->stream_count += enable ? 1 : -1;
	if (priv->stream_count < 0)
		priv->stream_count = 0;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static struct v4l2_mbus_framefmt *
__vdic_get_fmt(struct vdic_priv *priv, struct v4l2_subdev_pad_config *cfg,
	       unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&priv->sd, cfg, pad);
	else
		return &priv->format_mbus[pad];
}

static int vdic_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad >= VDIC_NUM_PADS)
		return -EINVAL;

	return imx_media_enum_ipu_format(&code->code, code->index, CS_SEL_YUV);
}

static int vdic_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *sdformat)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	if (sdformat->pad >= VDIC_NUM_PADS)
		return -EINVAL;

	mutex_lock(&priv->lock);

	fmt = __vdic_get_fmt(priv, cfg, sdformat->pad, sdformat->which);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}

	sdformat->format = *fmt;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static void vdic_try_fmt(struct vdic_priv *priv,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *sdformat,
			 const struct imx_media_pixfmt **cc)
{
	struct v4l2_mbus_framefmt *infmt;

	*cc = imx_media_find_ipu_format(sdformat->format.code, CS_SEL_YUV);
	if (!*cc) {
		u32 code;

		imx_media_enum_ipu_format(&code, 0, CS_SEL_YUV);
		*cc = imx_media_find_ipu_format(code, CS_SEL_YUV);
		sdformat->format.code = (*cc)->codes[0];
	}

	infmt = __vdic_get_fmt(priv, cfg, priv->active_input_pad,
			       sdformat->which);

	switch (sdformat->pad) {
	case VDIC_SRC_PAD_DIRECT:
	case VDIC_SRC_PAD_IDMAC:
		sdformat->format = *infmt;
		/* output is always progressive! */
		sdformat->format.field = V4L2_FIELD_NONE;
		break;
	case VDIC_SINK_PAD_DIRECT:
	case VDIC_SINK_PAD_IDMAC:
		v4l_bound_align_image(&sdformat->format.width,
				      MIN_W, MAX_W_VDIC, W_ALIGN,
				      &sdformat->format.height,
				      MIN_H, MAX_H_VDIC, H_ALIGN, S_ALIGN);

		/* input must be interlaced! Choose SEQ_TB if not */
		if (!V4L2_FIELD_HAS_BOTH(sdformat->format.field))
			sdformat->format.field = V4L2_FIELD_SEQ_TB;
		break;
	}

	imx_media_try_colorimetry(&sdformat->format, true);
}

static int vdic_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *sdformat)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	const struct imx_media_pixfmt *cc;
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	if (sdformat->pad >= VDIC_NUM_PADS)
		return -EINVAL;

	mutex_lock(&priv->lock);

	if (priv->stream_count > 0) {
		ret = -EBUSY;
		goto out;
	}

	vdic_try_fmt(priv, cfg, sdformat, &cc);

	fmt = __vdic_get_fmt(priv, cfg, sdformat->pad, sdformat->which);
	*fmt = sdformat->format;

	if (sdformat->pad == VDIC_SINK_PAD_DIRECT ||
	    sdformat->pad == VDIC_SINK_PAD_IDMAC) {
		int pad;

		/* propagate format to source pads */
		for (pad = VDIC_SINK_PAD_IDMAC + 1;
		     pad < VDIC_NUM_PADS; pad++) {
			const struct imx_media_pixfmt *outcc;
			struct v4l2_mbus_framefmt *outfmt;
			struct v4l2_subdev_format format;

			format.pad = pad;
			format.which = sdformat->which;
			format.format = sdformat->format;
			vdic_try_fmt(priv, cfg, &format, &outcc);

			outfmt = __vdic_get_fmt(priv, cfg, pad,
						sdformat->which);
			*outfmt = format.format;
			if (sdformat->which == V4L2_SUBDEV_FORMAT_ACTIVE)
				priv->cc[pad] = outcc;
		}
	}

	if (sdformat->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		priv->cc[sdformat->pad] = cc;

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int vdic_link_setup_sink(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote_sd;

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (priv->src)
			return -EBUSY;
	} else {
		priv->src = NULL;
		return 0;
	}

	switch (local->index) {
	case VDIC_SINK_PAD_IDMAC:
		priv->csi_direct = false;
		break;
	case VDIC_SINK_PAD_DIRECT:
		if (!is_media_entity_v4l2_subdev(remote->entity))
			return -EINVAL;

		remote_sd = media_entity_to_v4l2_subdev(remote->entity);

		/* direct pad must connect to a CSI */
		if (!(remote_sd->grp_id & IMX_MEDIA_GRP_ID_IPU_CSI) ||
		    remote->index != CSI_SRC_PAD_DIRECT)
			return -EINVAL;

		/* direct pad cannot route to idmac output pad */
		if (priv->active_output_pad == VDIC_SRC_PAD_IDMAC)
			return -EINVAL;

		priv->csi_direct = true;
		break;
	}

	priv->src = remote->entity;
	/* record which input pad is now active */
	priv->active_input_pad = local->index;

	return 0;
}

static int vdic_link_setup_source(struct media_entity *entity,
				  const struct media_pad *local,
				  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (priv->sink)
			return -EBUSY;
	} else {
		priv->sink = NULL;
		return 0;
	}

	switch (local->index) {
	case VDIC_SRC_PAD_DIRECT:
		if (!is_media_entity_v4l2_subdev(remote->entity))
			return -EINVAL;
		break;
	case VDIC_SRC_PAD_IDMAC:
		/* idmac pad cannot route from direct input pad */
		if (priv->active_input_pad == VDIC_SINK_PAD_DIRECT)
			return -EINVAL;

		break;
	}

	priv->sink = remote->entity;
	/* record which output pad is now active */
	priv->active_output_pad = local->index;

	return 0;
}

static int vdic_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	int ret;

	dev_dbg(priv->ipu_dev, "link setup %s -> %s", remote->entity->name,
		local->entity->name);

	mutex_lock(&priv->lock);

	if (local->flags & MEDIA_PAD_FL_SINK)
		ret = vdic_link_setup_sink(entity, local, remote, flags);
	else
		ret = vdic_link_setup_source(entity, local, remote, flags);

	mutex_unlock(&priv->lock);
	return ret;
}

static int vdic_link_validate(struct v4l2_subdev *sd,
			      struct media_link *link,
			      struct v4l2_subdev_format *source_fmt,
			      struct v4l2_subdev_format *sink_fmt)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	int ret;

	ret = v4l2_subdev_link_validate_default(sd, link,
						source_fmt, sink_fmt);
	if (ret)
		return ret;

	mutex_lock(&priv->lock);

	if (priv->csi_direct && priv->motion != HIGH_MOTION) {
		v4l2_err(&priv->sd,
			 "direct CSI pipeline requires high motion\n");
		ret = -EINVAL;
	}

	mutex_unlock(&priv->lock);
	return ret;
}

static int vdic_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fi)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);

	if (fi->pad >= VDIC_NUM_PADS)
		return -EINVAL;

	mutex_lock(&priv->lock);

	fi->interval = priv->frame_interval[fi->pad];

	mutex_unlock(&priv->lock);

	return 0;
}

static int vdic_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fi)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_fract *input_fi;
	int ret = 0;

	mutex_lock(&priv->lock);

	input_fi = &priv->frame_interval[priv->active_input_pad];

	switch (fi->pad) {
	case VDIC_SINK_PAD_DIRECT:
	case VDIC_SINK_PAD_IDMAC:
		/* No limits on valid input frame intervals */
		if (fi->interval.numerator == 0 ||
		    fi->interval.denominator == 0)
			fi->interval = priv->frame_interval[fi->pad];
		/* Reset output intervals */
		priv->frame_interval[VDIC_SRC_PAD_DIRECT] = fi->interval;
		priv->frame_interval[VDIC_SRC_PAD_IDMAC] = fi->interval;
		if (priv->csi_direct) {
			priv->frame_interval[VDIC_SRC_PAD_DIRECT].denominator *= 2;
			priv->frame_interval[VDIC_SRC_PAD_IDMAC].denominator *= 2;
		}
		break;
	case VDIC_SRC_PAD_DIRECT:
	case VDIC_SRC_PAD_IDMAC:
		/*
		 * frame rate at output pad is double input
		 * rate when using direct CSI->VDIC pipeline.
		 *
		 * TODO: implement VDIC frame skipping
		 */
		fi->interval = *input_fi;
		if (priv->csi_direct)
			fi->interval.denominator *= 2;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	priv->frame_interval[fi->pad] = fi->interval;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int vdic_registered(struct v4l2_subdev *sd)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);
	int i, ret;
	u32 code;

	for (i = 0; i < VDIC_NUM_PADS; i++) {
		code = 0;

		/* init default frame interval */
		priv->frame_interval[i].numerator = 1;
		priv->frame_interval[i].denominator = 30;

		switch (i) {
		case VDIC_SINK_PAD_DIRECT:
			imx_media_enum_ipu_format(&code, 0, CS_SEL_YUV);
			/* fallthrough */
		case VDIC_SINK_PAD_IDMAC:
			priv->pad[i].flags = MEDIA_PAD_FL_SINK;
			break;
		case VDIC_SRC_PAD_DIRECT:
			priv->frame_interval[i].denominator *= 2;
			imx_media_enum_ipu_format(&code, 0, CS_SEL_YUV);
			/* fallthrough */
		case VDIC_SRC_PAD_IDMAC:
			priv->pad[i].flags = MEDIA_PAD_FL_SOURCE;
			break;
		}

		/* set a default mbus format  */
		ret = imx_media_init_mbus_fmt(&priv->format_mbus[i],
					      640, 480, code, V4L2_FIELD_NONE,
					      &priv->cc[i]);
		if (ret)
			return ret;
	}

	priv->active_input_pad = VDIC_SINK_PAD_IDMAC;
	priv->active_output_pad = VDIC_SRC_PAD_DIRECT;

	ret = vdic_init_controls(priv);
	if (ret)
		return ret;

	ret = media_entity_pads_init(&sd->entity, VDIC_NUM_PADS, priv->pad);
	if (ret)
		goto free_ctrls;

	priv->vdev_src = imx_media_mem2mem_vdic_init(priv->ipu_dev, sd,
						     VDIC_SINK_PAD_IDMAC,
						     VDIC_SRC_PAD_IDMAC);
	if (IS_ERR(priv->vdev_src)) {
		ret = PTR_ERR(priv->vdev_src);
		goto free_ctrls;
	}

	/*
	 * for now, we only support mem2mem devices at the idmac pads,
	 * where the video device acts as both an output and capture
	 * device.
	 */
	priv->vdev_sink = priv->vdev_src;

	ret = imx_media_mem2mem_vdic_register(priv->vdev_src);
	if (ret)
		goto free_ctrls;

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrl_hdlr);
	return ret;
}

static void vdic_unregistered(struct v4l2_subdev *sd)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);

	imx_media_mem2mem_vdic_unregister(priv->vdev_src);
	imx_media_mem2mem_vdic_remove(priv->vdev_src);
	v4l2_ctrl_handler_free(&priv->ctrl_hdlr);
}

static const struct v4l2_subdev_pad_ops vdic_pad_ops = {
	.init_cfg = imx_media_init_cfg,
	.enum_mbus_code = vdic_enum_mbus_code,
	.get_fmt = vdic_get_fmt,
	.set_fmt = vdic_set_fmt,
	.link_validate = vdic_link_validate,
};

static const struct v4l2_subdev_video_ops vdic_video_ops = {
	.g_frame_interval = vdic_g_frame_interval,
	.s_frame_interval = vdic_s_frame_interval,
	.s_stream = vdic_s_stream,
};

static const struct v4l2_subdev_proc_ops vdic_proc_ops = {
	.device_run = vdic_device_run,
};

static const struct media_entity_operations vdic_entity_ops = {
	.link_setup = vdic_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_ops vdic_subdev_ops = {
	.video = &vdic_video_ops,
	.proc = &vdic_proc_ops,
	.pad = &vdic_pad_ops,
};

static const struct v4l2_subdev_internal_ops vdic_internal_ops = {
	.registered = vdic_registered,
	.unregistered = vdic_unregistered,
};

struct v4l2_subdev *imx_media_vdic_register(struct v4l2_device *v4l2_dev,
					    struct device *ipu_dev,
					    struct ipu_soc *ipu,
					    u32 grp_id)
{
	struct vdic_priv *priv;
	int i, ret;

	priv = devm_kzalloc(ipu_dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->ipu_dev = ipu_dev;
	priv->ipu = ipu;

	v4l2_subdev_init(&priv->sd, &vdic_subdev_ops);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.internal_ops = &vdic_internal_ops;
	priv->sd.entity.ops = &vdic_entity_ops;
	priv->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	priv->sd.owner = ipu_dev->driver->owner;
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->sd.grp_id = grp_id;
	imx_media_grp_id_to_sd_name(priv->sd.name, sizeof(priv->sd.name),
				    priv->sd.grp_id, ipu_get_num(ipu));

	spin_lock_init(&priv->irqlock);
	mutex_init(&priv->lock);

	for (i = 0; i < VDIC_NUM_PADS; i++)
		priv->pad[i].flags = (i == VDIC_SRC_PAD_DIRECT) ?
			MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(&priv->sd.entity, VDIC_NUM_PADS,
				     priv->pad);
	if (ret)
		goto free;

	ret = v4l2_device_register_subdev(v4l2_dev, &priv->sd);
	if (ret)
		goto free;

	return &priv->sd;
free:
	mutex_destroy(&priv->lock);
	return ERR_PTR(ret);
}

int imx_media_vdic_unregister(struct v4l2_subdev *sd)
{
	struct vdic_priv *priv = v4l2_get_subdevdata(sd);

	v4l2_info(sd, "Removing\n");

	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&priv->lock);
	media_entity_cleanup(&sd->entity);

	return 0;
}
