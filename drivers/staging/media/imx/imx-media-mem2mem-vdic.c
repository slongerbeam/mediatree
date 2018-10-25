// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * i.MX VDIC mem2mem de-interlace driver
 *
 * Copyright (c) 2018-2019 Mentor Graphics Inc.
 */
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/version.h>

#include <media/media-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/imx.h>
#include "imx-media.h"

#define fh_to_ctx(__fh)	container_of(__fh, struct mem2mem_ctx, fh)

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

struct mem2mem_ctx;

struct mem2mem_priv {
	struct imx_media_video_dev vdev;
	struct imx_media_dev  *md;

	struct v4l2_subdev    *vdic_sink;
	u32                   vdic_sink_pad;
	struct v4l2_subdev    *vdic_src;
	u32                   vdic_src_pad;

	struct v4l2_m2m_dev   *m2m_dev;
	struct device         *dev;

	struct mutex          mutex;       /* mem2mem device mutex */

	struct v4l2_pix_format fmt[2];
	const struct imx_media_pixfmt *cc[2];
};

#define to_mem2mem_priv(v) container_of(v, struct mem2mem_priv, vdev)

struct mem2mem_ctx {
	struct mem2mem_priv	*priv;
	struct v4l2_fh		fh;
	struct work_struct      job_done_work;
};

static struct v4l2_pix_format *get_format(struct mem2mem_priv *priv,
					  enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &priv->fmt[V4L2_M2M_SRC];
	else
		return &priv->fmt[V4L2_M2M_DST];
}

/*
 * imx-media video device ops follow
 */

static int mem2mem_get_fmt(struct imx_media_video_dev *vdev,
			   struct v4l2_format *f,
			   struct v4l2_rect *compose,
			   const struct imx_media_pixfmt **cc)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);
	struct v4l2_pix_format *fmt;

	fmt = get_format(priv, f->type);
	f->fmt.pix = *fmt;
	if (compose) {
		compose->left = 0;
		compose->top = 0;
		compose->width = fmt->width;
		compose->height = fmt->height;
	}
	if (cc)
		*cc = priv->cc[f->type];

	return 0;
}

static struct vb2_v4l2_buffer *
mem2mem_remove_next_buf(struct imx_media_video_dev *vdev, void *_ctx,
			enum v4l2_buf_type type)
{
	struct mem2mem_ctx *ctx = _ctx;
	struct vb2_v4l2_buffer *buf;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	return buf;
}

static struct vb2_v4l2_buffer *
mem2mem_get_next_buf(struct imx_media_video_dev *vdev, void *_ctx,
		     enum v4l2_buf_type type)
{
	struct mem2mem_ctx *ctx = _ctx;
	struct vb2_v4l2_buffer *buf;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	return buf;
}

static void mem2mem_buf_done(struct imx_media_video_dev *vdev, void *_ctx,
			     enum v4l2_buf_type type,
			     struct vb2_v4l2_buffer *done,
			     enum vb2_buffer_state status)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);
	struct mem2mem_ctx *ctx = _ctx;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dev_dbg(priv->dev, "%s: dstbuf %p\n", __func__, done);
		done->vb2_buf.timestamp = ktime_get_ns();
		done->field = V4L2_FIELD_NONE;
		v4l2_m2m_buf_done(done, status);
		/* job is done */
		schedule_work(&ctx->job_done_work);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		dev_dbg(priv->dev, "%s: srcbuf %p\n", __func__, done);
		v4l2_m2m_buf_done(done, status);
		break;
	default:
		break;
	}
}

static int mem2mem_bufs_ready(struct imx_media_video_dev *vdev, void *_ctx,
			      enum v4l2_buf_type type)
{
	struct mem2mem_ctx *ctx = _ctx;
	unsigned int num_bufs;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		num_bufs = v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		num_bufs = v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx);
		break;
	default:
		return -EINVAL;
	}

	return num_bufs;
}

static void mem2mem_device_error(struct imx_media_video_dev *vdev, void *_ctx)
{
	struct mem2mem_ctx *ctx = _ctx;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	if (!vb2_is_streaming(vq))
		return;

	vb2_queue_error(vq);
}

static const struct imx_media_video_dev_ops mem2mem_ops = {
	.get_fmt = mem2mem_get_fmt,
	.remove_next_buf = mem2mem_remove_next_buf,
	.get_next_buf = mem2mem_get_next_buf,
	.bufs_ready = mem2mem_bufs_ready,
	.buf_done = mem2mem_buf_done,
	.device_error = mem2mem_device_error,
};

static void mem2mem_job_done_work(struct work_struct *work)
{
	struct mem2mem_ctx *ctx =
		container_of(work, struct mem2mem_ctx, job_done_work);
	struct mem2mem_priv *priv = ctx->priv;

	v4l2_subdev_call(priv->vdic_src, core, ioctl, IMX_CMD_JOB_DONE, ctx);
	v4l2_m2m_job_finish(priv->m2m_dev, ctx->fh.m2m_ctx);
}

/*
 * mem2mem callbacks
 */

static int job_ready(void *_ctx)
{
	struct mem2mem_ctx *ctx = _ctx;
	struct mem2mem_priv *priv = ctx->priv;

	return v4l2_subdev_call(priv->vdic_src, core, ioctl,
				IMX_CMD_JOB_READY, ctx);
}

static void device_run(void *_ctx)
{
	struct mem2mem_ctx *ctx = _ctx;
	struct mem2mem_priv *priv = ctx->priv;
	int ret;

	ret = v4l2_subdev_call(priv->vdic_src, core, ioctl,
			       IMX_CMD_JOB_RUN, ctx);
	if (ret)
		v4l2_err(&priv->md->v4l2_dev, "%s: failed with %d\n",
			 __func__, ret);
}

/*
 * Video ioctls
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, "imx-media-mem2mem-vdic",
		sizeof(cap->driver) - 1);
	strncpy(cap->card, "imx-media-mem2mem-vdic", sizeof(cap->card) - 1);
	strncpy(cap->bus_info, "platform:imx-media-mem2mem-vdic",
		sizeof(cap->bus_info) - 1);
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int mem2mem_enum_fmt(struct file *file, void *fh,
			    struct v4l2_fmtdesc *f)
{
	enum imx_pixfmt_sel sel;
	u32 fourcc;
	int ret;

	sel = (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
		PIXFMT_SEL_YUV_RGB : PIXFMT_SEL_YUV;

	ret = imx_media_enum_pixel_formats(&fourcc, f->index, sel);
	if (ret)
		return ret;

	f->pixelformat = fourcc;

	return 0;
}

static int mem2mem_g_fmt(struct file *file, void *_priv, struct v4l2_format *f)
{
	struct mem2mem_ctx *ctx = fh_to_ctx(_priv);
	struct mem2mem_priv *priv = ctx->priv;

	return mem2mem_get_fmt(&priv->vdev, f, NULL, NULL);
}

static int mem2mem_try_fmt(struct file *file, void *_priv,
			   struct v4l2_format *f)
{
	struct mem2mem_ctx *ctx = fh_to_ctx(_priv);
	struct mem2mem_priv *priv = ctx->priv;
	u32 fourcc = f->fmt.pix.pixelformat;
	const struct imx_media_pixfmt *cc;
	struct v4l2_subdev_format fmt;
	enum imx_pixfmt_sel sel;
	struct v4l2_subdev *sd;
	int ret;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		fmt.pad = priv->vdic_src_pad;
		sd = priv->vdic_src;
		sel = PIXFMT_SEL_YUV_RGB;
	} else {
		fmt.pad = priv->vdic_sink_pad;
		sd = priv->vdic_sink;
		sel = PIXFMT_SEL_YUV;
	}

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	cc = imx_media_find_pixel_format(fourcc, sel);
	if (!cc) {
		imx_media_enum_pixel_formats(&fourcc, 0, PIXFMT_SEL_YUV);
		cc = imx_media_find_pixel_format(fourcc, PIXFMT_SEL_YUV);
	}

	imx_media_mbus_fmt_to_pix_fmt(&f->fmt.pix, &fmt.format, cc);

	return 0;
}

static int mem2mem_s_fmt(struct file *file, void *_priv, struct v4l2_format *f)
{
	struct mem2mem_ctx *ctx = fh_to_ctx(_priv);
	struct mem2mem_priv *priv = ctx->priv;
	struct v4l2_pix_format *fmt;
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&priv->md->v4l2_dev, "%s queue busy\n",  __func__);
		return -EBUSY;
	}

	ret = mem2mem_try_fmt(file, _priv, f);
	if (ret < 0)
		return ret;

	fmt = get_format(priv, f->type);
	*fmt = f->fmt.pix;
	priv->cc[f->type] =
		imx_media_find_pixel_format(f->fmt.pix.pixelformat,
					    PIXFMT_SEL_YUV_RGB);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		/* Propagate colorimetry to the capture queue */
		fmt = get_format(priv, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		fmt->colorspace = f->fmt.pix.colorspace;
		fmt->ycbcr_enc = f->fmt.pix.ycbcr_enc;
		fmt->xfer_func = f->fmt.pix.xfer_func;
		fmt->quantization = f->fmt.pix.quantization;
	}

	/*
	 * TODO: Setting colorimetry on the capture queue is currently not
	 * supported by the V4L2 API
	 */

	return 0;
}

static const struct v4l2_ioctl_ops mem2mem_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = mem2mem_enum_fmt,
	.vidioc_g_fmt_vid_cap	= mem2mem_g_fmt,
	.vidioc_try_fmt_vid_cap	= mem2mem_try_fmt,
	.vidioc_s_fmt_vid_cap	= mem2mem_s_fmt,

	.vidioc_enum_fmt_vid_out = mem2mem_enum_fmt,
	.vidioc_g_fmt_vid_out	= mem2mem_g_fmt,
	.vidioc_try_fmt_vid_out	= mem2mem_try_fmt,
	.vidioc_s_fmt_vid_out	= mem2mem_s_fmt,

	.vidioc_reqbufs		= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf	= v4l2_m2m_ioctl_querybuf,

	.vidioc_qbuf		= v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf		= v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf		= v4l2_m2m_ioctl_dqbuf,
	.vidioc_create_bufs	= v4l2_m2m_ioctl_create_bufs,

	.vidioc_streamon	= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff	= v4l2_m2m_ioctl_streamoff,
};

/*
 * Queue operations
 */

static int mem2mem_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct mem2mem_ctx *ctx = vb2_get_drv_priv(vq);
	struct mem2mem_priv *priv = ctx->priv;
	struct v4l2_pix_format *fmt;
	unsigned int size, count;

	fmt = get_format(priv, vq->type);

	size = fmt->sizeimage;

	switch (vq->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		count = max(2U, *nbuffers);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		count = max(4U, *nbuffers);
		break;
	default:
		return -EINVAL;
	}

	*nbuffers = count;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	dev_dbg(priv->dev, "%s queue: %d buffers of size %d each\n",
		vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
		"capture" : "output", count, size);

	return 0;
}

static int mem2mem_buf_prepare(struct vb2_buffer *vb)
{
	struct mem2mem_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mem2mem_priv *priv = ctx->priv;
	struct v4l2_pix_format *fmt;

	fmt = get_format(priv, vb->vb2_queue->type);

	if (vb2_plane_size(vb, 0) < fmt->sizeimage) {
		v4l2_err(&priv->md->v4l2_dev,
			 "%s: data will not fit into plane (%lu < %lu)\n",
			 __func__, vb2_plane_size(vb, 0),
			 (long)fmt->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, fmt->sizeimage);

	return 0;
}

static void mem2mem_buf_queue(struct vb2_buffer *vb)
{
	struct mem2mem_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int mem2mem_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mem2mem_ctx *ctx = vb2_get_drv_priv(q);
	struct mem2mem_priv *priv = ctx->priv;
	struct vb2_v4l2_buffer *buf;
	struct vb2_queue *other_q;
	int ret;

	dev_dbg(priv->dev, "%s: %s\n", __func__,
		(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
		"capture" : "output");

	other_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				  (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
				  V4L2_BUF_TYPE_VIDEO_OUTPUT :
				  V4L2_BUF_TYPE_VIDEO_CAPTURE);

	/* last call starts the pipeline */
	if (!vb2_is_streaming(other_q))
		return 0;

	ret = imx_media_pipeline_set_stream(priv->md,
					    &priv->vdic_src->entity,
					    true);
	if (ret) {
		v4l2_err(&priv->md->v4l2_dev,
			 "pipeline start failed with %d\n", ret);
		goto return_bufs;
	}

	return 0;

return_bufs:
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	}

	return ret;
}

static void mem2mem_stop_streaming(struct vb2_queue *q)
{
	struct mem2mem_ctx *ctx = vb2_get_drv_priv(q);
	struct mem2mem_priv *priv = ctx->priv;
	struct vb2_v4l2_buffer *buf;
	struct vb2_queue *other_q;
	int ret;

	dev_dbg(priv->dev, "%s: %s\n", __func__,
		(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
		"capture" : "output");

	other_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				  (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
				  V4L2_BUF_TYPE_VIDEO_OUTPUT :
				  V4L2_BUF_TYPE_VIDEO_CAPTURE);

	/* first call stops the pipeline */
	if (!vb2_is_streaming(other_q))
		goto return_bufs;

	ret = imx_media_pipeline_set_stream(priv->md,
					    &priv->vdic_src->entity,
					    false);
	if (ret)
		v4l2_warn(&priv->md->v4l2_dev,
			  "pipeline stop failed with %d\n", ret);

return_bufs:
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx))) {
			dev_dbg(priv->dev, "%s: srcbuf %p\n", __func__, buf);
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
		}
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx))) {
			dev_dbg(priv->dev, "%s: dstbuf %p\n", __func__, buf);
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
		}
	}
}

static const struct vb2_ops mem2mem_qops = {
	.queue_setup	= mem2mem_queue_setup,
	.buf_prepare	= mem2mem_buf_prepare,
	.buf_queue	= mem2mem_buf_queue,
	.wait_prepare	= vb2_ops_wait_prepare,
	.wait_finish	= vb2_ops_wait_finish,
	.start_streaming = mem2mem_start_streaming,
	.stop_streaming = mem2mem_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct mem2mem_ctx *ctx = priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->min_buffers_needed = 1;
	src_vq->ops = &mem2mem_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->priv->mutex;
	src_vq->dev = ctx->priv->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->min_buffers_needed = 1;
	dst_vq->ops = &mem2mem_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->priv->mutex;
	dst_vq->dev = ctx->priv->dev;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int mem2mem_open(struct file *file)
{
	struct mem2mem_priv *priv = video_drvdata(file);
	struct mem2mem_ctx *ctx = NULL;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	ctx->priv = priv;

	INIT_WORK(&ctx->job_done_work, mem2mem_job_done_work);

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(priv->m2m_dev, ctx,
					    &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctx;
	}

	return 0;

err_ctx:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int mem2mem_release(struct file *file)
{
	struct mem2mem_ctx *ctx = fh_to_ctx(file->private_data);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations mem2mem_fops = {
	.owner		= THIS_MODULE,
	.open		= mem2mem_open,
	.release	= mem2mem_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static struct v4l2_m2m_ops m2m_ops = {
	.job_ready	= job_ready,
	.device_run	= device_run,
};

static void imx_media_mem2mem_vdic_release(struct video_device *vdev)
{
	struct mem2mem_priv *priv = video_get_drvdata(vdev);

	v4l2_m2m_release(priv->m2m_dev);
	video_device_release(vdev);
	mutex_destroy(&priv->mutex);
	kfree(priv);
}

static const struct video_device mem2mem_template = {
	.fops		= &mem2mem_fops,
	.ioctl_ops	= &mem2mem_ioctl_ops,
	.minor		= -1,
	.release	= imx_media_mem2mem_vdic_release,
	.vfl_dir	= VFL_DIR_M2M,
	.tvnorms	= V4L2_STD_NTSC | V4L2_STD_PAL | V4L2_STD_SECAM,
	.device_caps	= V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
};

/*
 * Returns the VDIC mem2mem video device that connects to given pad
 * at given IPU. The pad must be either the processing sink pad
 * or processing source pad.
 */
struct imx_media_video_dev *
imx_media_mem2mem_vdic_get(const struct media_pad *pad, int ipu_id)
{
	struct media_pad *proc_sink, *proc_src;
	struct imx_media_video_dev *vdev;
	struct imx_media_dev *imxmd;
	struct mem2mem_priv *priv;
	struct v4l2_subdev *sd;

	if (!is_media_entity_v4l2_subdev(pad->entity))
		return NULL;
	sd = media_entity_to_v4l2_subdev(pad->entity);

	imxmd = container_of(sd->v4l2_dev->mdev, struct imx_media_dev, md);
	vdev = imxmd->m2m_vdic_vdev[ipu_id];
	if (!vdev)
		return NULL;
	priv = to_mem2mem_priv(vdev);

	proc_sink = &priv->vdic_sink->entity.pads[priv->vdic_sink_pad];
	proc_src = &priv->vdic_src->entity.pads[priv->vdic_src_pad];

	if (pad != proc_sink && pad != proc_src)
		return NULL;

	return vdev;
}

int imx_media_mem2mem_vdic_register(struct imx_media_video_dev *vdev)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);
	struct media_pad *proc_sink, *proc_src;
	struct video_device *vfd = vdev->vfd;
	int ret;

	vfd->v4l2_dev = &priv->md->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(vfd->v4l2_dev, "Failed to register video device\n");
		return ret;
	}

	proc_sink = &priv->vdic_sink->entity.pads[priv->vdic_sink_pad];
	proc_src = &priv->vdic_src->entity.pads[priv->vdic_src_pad];

	ret = __v4l2_m2m_register_media_controller(priv->m2m_dev, vfd,
						   proc_sink, proc_src,
						   0, 0);
	if (ret) {
		v4l2_err(vfd->v4l2_dev,
			 "Failed to register with media controller\n");
		return ret;
	}

	v4l2_info(vfd->v4l2_dev, "Registered %s as /dev/%s\n", vfd->name,
		  video_device_node_name(vfd));

	return 0;
}

void imx_media_mem2mem_vdic_unregister(struct imx_media_video_dev *vdev)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);
	struct video_device *vfd = priv->vdev.vfd;

	if (video_is_registered(vfd)) {
		v4l2_m2m_unregister_media_controller(priv->m2m_dev);
		video_unregister_device(vfd);
		media_entity_cleanup(&vfd->entity);
	}
}

struct imx_media_video_dev *
imx_media_mem2mem_vdic_init(struct imx_media_dev *md,
			    struct v4l2_subdev *vdic_sink,
			    u32 vdic_sink_pad,
			    struct v4l2_subdev *vdic_src,
			    u32 vdic_src_pad)
{
	struct mem2mem_priv *priv;
	struct video_device *vfd;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->md = md;
	priv->dev = md->md.dev;

	mutex_init(&priv->mutex);

	priv->vdic_sink = vdic_sink;
	priv->vdic_src = vdic_src;
	priv->vdic_sink_pad = vdic_sink_pad;
	priv->vdic_src_pad = vdic_src_pad;

	vfd = video_device_alloc();
	if (!vfd) {
		ret = -ENOMEM;
		goto err_vfd;
	}

	*vfd = mem2mem_template;
	snprintf(vfd->name, sizeof(vfd->name), "%s mem2mem",
		 priv->vdic_sink->name);
	vfd->lock = &priv->mutex;
	priv->vdev.vfd = vfd;
	priv->vdev.ops = &mem2mem_ops;

	INIT_LIST_HEAD(&priv->vdev.list);

	video_set_drvdata(vfd, priv);

	priv->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(priv->m2m_dev)) {
		ret = PTR_ERR(priv->m2m_dev);
		dev_err(priv->dev, "Failed to init mem2mem device: %d\n", ret);
		goto err_m2m;
	}

	return &priv->vdev;

err_m2m:
	video_set_drvdata(vfd, NULL);
err_vfd:
	mutex_destroy(&priv->mutex);
	kfree(priv);
	return ERR_PTR(ret);
}

MODULE_DESCRIPTION("i.MX VDIC mem2mem de-interlace driver");
MODULE_AUTHOR("Steve Longerbeam <steve_longerbeam@mentor.com>");
MODULE_LICENSE("GPL");
