// SPDX-License-Identifier: GPL-2.0+
/*
 * V4L2 Capture CSI Subdev for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2014-2017 Mentor Graphics Inc.
 * Copyright (C) 2017 Pengutronix, Philipp Zabel <kernel@pengutronix.de>
 */
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <video/imx-ipu-v3.h>
#include <media/imx.h>
#include "imx-media.h"

/*
 * Min/Max supported width and heights.
 *
 * We allow planar output, so we have to align width by 16 pixels
 * to meet IDMAC alignment requirements.
 *
 * TODO: move this into pad format negotiation, if capture device
 * has not requested planar formats, we should allow 8 pixel
 * alignment.
 */
#define MIN_W       176
#define MIN_H       144
#define MAX_W      4096
#define MAX_H      4096
#define W_ALIGN    1 /* multiple of 2 pixels */
#define H_ALIGN    1 /* multiple of 2 lines */
#define S_ALIGN    1 /* multiple of 2 */

/*
 * struct csi_skip_desc - CSI frame skipping descriptor
 * @keep - number of frames kept per max_ratio frames
 * @max_ratio - width of skip_smfc, written to MAX_RATIO bitfield
 * @skip_smfc - skip pattern written to the SKIP_SMFC bitfield
 */
struct csi_skip_desc {
	u8 keep;
	u8 max_ratio;
	u8 skip_smfc;
};

struct csi_priv {
	struct device *dev;
	struct ipu_soc *ipu;
	struct v4l2_subdev sd;
	struct media_pad pad[CSI_NUM_PADS];
	struct v4l2_async_notifier notifier;

	/* the video device at IDMAC output pad */
	struct imx_media_video_dev *vdev;
	int csi_id;

	/* lock to protect all members below */
	struct mutex lock;

	int active_output_pad;

	struct ipu_csi *csi;

	struct v4l2_mbus_framefmt format_mbus[CSI_NUM_PADS];
	const struct imx_media_pixfmt *cc[CSI_NUM_PADS];
	struct v4l2_fract frame_interval[CSI_NUM_PADS];
	struct v4l2_rect crop;
	struct v4l2_rect compose;
	const struct csi_skip_desc *skip;

	/* the sink for the captured frames */
	struct media_entity *sink;
	enum ipu_csi_dest dest;
	/* the source subdev */
	struct v4l2_subdev *src_sd;

	/* the mipi virtual channel number at link validate */
	int vc_num;

	/* the upstream endpoint CSI is receiving from */
	struct v4l2_fwnode_endpoint upstream_ep;

	int stream_count; /* streaming counter */
};

static inline struct csi_priv *sd_to_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct csi_priv, sd);
}

static inline struct csi_priv *notifier_to_dev(struct v4l2_async_notifier *n)
{
	return container_of(n, struct csi_priv, notifier);
}

static inline bool is_parallel_bus(struct v4l2_fwnode_endpoint *ep)
{
	return ep->bus_type != V4L2_MBUS_CSI2_DPHY;
}

static inline bool is_parallel_16bit_bus(struct v4l2_fwnode_endpoint *ep)
{
	return is_parallel_bus(ep) && ep->bus.parallel.bus_width >= 16;
}

/*
 * Check for conditions that require the IPU to handle the
 * data internally as generic data, aka passthrough mode:
 * - raw bayer media bus formats, or
 * - the CSI is receiving from a 16-bit parallel bus, or
 * - the CSI is receiving from an 8-bit parallel bus and the incoming
 *   media bus format is other than UYVY8_2X8/YUYV8_2X8.
 */
static inline bool requires_passthrough(struct v4l2_fwnode_endpoint *ep,
					struct v4l2_mbus_framefmt *infmt,
					const struct imx_media_pixfmt *incc)
{
	return incc->bayer || is_parallel_16bit_bus(ep) ||
		(is_parallel_bus(ep) &&
		 infmt->code != MEDIA_BUS_FMT_UYVY8_2X8 &&
		 infmt->code != MEDIA_BUS_FMT_YUYV8_2X8);
}

/*
 * Parses the fwnode endpoint from the source pad of the entity
 * connected to this CSI. This will either be the entity directly
 * upstream from the CSI-2 receiver, directly upstream from the
 * video mux, or directly upstream from the CSI itself. The endpoint
 * is needed to determine the bus type and bus config coming into
 * the CSI.
 */
static int csi_get_upstream_endpoint(struct csi_priv *priv,
				     struct v4l2_fwnode_endpoint *ep)
{
	struct fwnode_handle *endpoint;
	struct v4l2_subdev *sd;
	struct media_pad *pad;

	if (!IS_ENABLED(CONFIG_OF))
		return -ENXIO;

	if (!priv->src_sd)
		return -EPIPE;

	sd = priv->src_sd;

	switch (sd->grp_id) {
	case IMX_MEDIA_GRP_ID_CSI_MUX:
		/*
		 * CSI is connected directly to CSI mux, skip up to
		 * CSI-2 receiver if it is in the path, otherwise stay
		 * with the CSI mux.
		 */
		sd = imx_media_pipeline_subdev(&sd->entity,
					       IMX_MEDIA_GRP_ID_CSI2,
					       true);
		if (IS_ERR(sd))
			sd = priv->src_sd;
		break;
	case IMX_MEDIA_GRP_ID_CSI2:
		break;
	default:
		/*
		 * the source is neither the CSI mux nor the CSI-2 receiver,
		 * get the source pad directly upstream from CSI itself.
		 */
		sd = &priv->sd;
		break;
	}

	/* get source pad of entity directly upstream from sd */
	pad = imx_media_pipeline_pad(&sd->entity, 0, 0, true);
	if (!pad)
		return -ENODEV;

	endpoint = imx_media_get_pad_fwnode(pad);
	if (IS_ERR(endpoint))
		return PTR_ERR(endpoint);

	v4l2_fwnode_endpoint_parse(endpoint, ep);

	fwnode_handle_put(endpoint);

	return 0;
}

/* Update the CSI whole sensor and active windows */
static int csi_setup(struct csi_priv *priv)
{
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	const struct imx_media_pixfmt *incc;
	struct v4l2_mbus_config mbus_cfg;
	struct v4l2_mbus_framefmt if_fmt;
	struct v4l2_rect crop;

	infmt = &priv->format_mbus[CSI_SINK_PAD];
	incc = priv->cc[CSI_SINK_PAD];
	outfmt = &priv->format_mbus[priv->active_output_pad];

	/* compose mbus_config from the upstream endpoint */
	mbus_cfg.type = priv->upstream_ep.bus_type;
	mbus_cfg.flags = is_parallel_bus(&priv->upstream_ep) ?
		priv->upstream_ep.bus.parallel.flags :
		priv->upstream_ep.bus.mipi_csi2.flags;

	if_fmt = *infmt;
	crop = priv->crop;

	/*
	 * if cycles is set, we need to handle this over multiple cycles as
	 * generic/bayer data
	 */
	if (is_parallel_bus(&priv->upstream_ep) && incc->cycles) {
		if_fmt.width *= incc->cycles;
		crop.width *= incc->cycles;
	}

	ipu_csi_set_window(priv->csi, &crop);

	ipu_csi_set_downsize(priv->csi,
			     priv->crop.width == 2 * priv->compose.width,
			     priv->crop.height == 2 * priv->compose.height);

	ipu_csi_init_interface(priv->csi, &mbus_cfg, &if_fmt, outfmt);

	ipu_csi_set_dest(priv->csi, priv->dest);

	if (priv->dest == IPU_CSI_DEST_IDMAC)
		ipu_csi_set_skip_smfc(priv->csi, priv->skip->skip_smfc,
				      priv->skip->max_ratio - 1, 0);

	ipu_csi_dump(priv->csi);

	return 0;
}

static int csi_start(struct csi_priv *priv)
{
	int ret;

	/* start upstream */
	ret = v4l2_subdev_call(priv->src_sd, video, s_stream, 1);
	ret = (ret && ret != -ENOIOCTLCMD) ? ret : 0;
	if (ret)
		return ret;

	ret = csi_setup(priv);
	if (ret)
		goto stop_upstream;

	ret = ipu_csi_enable(priv->csi);
	if (ret) {
		v4l2_err(&priv->sd, "CSI enable error: %d\n", ret);
		goto stop_upstream;
	}

	return 0;

stop_upstream:
	v4l2_subdev_call(priv->src_sd, video, s_stream, 0);
	return ret;
}

static void csi_stop(struct csi_priv *priv)
{
	/*
	 * Disable the CSI asap, after syncing with the last EOF.
	 * Doing so after the IDMA channel is disabled has shown to
	 * create hard system-wide hangs.
	 */
	ipu_csi_disable(priv->csi);

	/* stop upstream */
	v4l2_subdev_call(priv->src_sd, video, s_stream, 0);
}

static const struct csi_skip_desc csi_skip[12] = {
	{ 1, 1, 0x00 }, /* Keep all frames */
	{ 5, 6, 0x10 }, /* Skip every sixth frame */
	{ 4, 5, 0x08 }, /* Skip every fifth frame */
	{ 3, 4, 0x04 }, /* Skip every fourth frame */
	{ 2, 3, 0x02 }, /* Skip every third frame */
	{ 3, 5, 0x0a }, /* Skip frames 1 and 3 of every 5 */
	{ 1, 2, 0x01 }, /* Skip every second frame */
	{ 2, 5, 0x0b }, /* Keep frames 1 and 4 of every 5 */
	{ 1, 3, 0x03 }, /* Keep one in three frames */
	{ 1, 4, 0x07 }, /* Keep one in four frames */
	{ 1, 5, 0x0f }, /* Keep one in five frames */
	{ 1, 6, 0x1f }, /* Keep one in six frames */
};

static void csi_apply_skip_interval(const struct csi_skip_desc *skip,
				    struct v4l2_fract *interval)
{
	unsigned int div;

	interval->numerator *= skip->max_ratio;
	interval->denominator *= skip->keep;

	/* Reduce fraction to lowest terms */
	div = gcd(interval->numerator, interval->denominator);
	if (div > 1) {
		interval->numerator /= div;
		interval->denominator /= div;
	}
}

/*
 * Find the skip pattern to produce the output frame interval closest to the
 * requested one, for the given input frame interval. Updates the output frame
 * interval to the exact value.
 */
static const struct csi_skip_desc *csi_find_best_skip(struct v4l2_fract *in,
						      struct v4l2_fract *out)
{
	const struct csi_skip_desc *skip = &csi_skip[0], *best_skip = skip;
	u32 min_err = UINT_MAX;
	u64 want_us;
	int i;

	/* Default to 1:1 ratio */
	if (out->numerator == 0 || out->denominator == 0 ||
	    in->numerator == 0 || in->denominator == 0) {
		*out = *in;
		return best_skip;
	}

	want_us = div_u64((u64)USEC_PER_SEC * out->numerator, out->denominator);

	/* Find the reduction closest to the requested time per frame */
	for (i = 0; i < ARRAY_SIZE(csi_skip); i++, skip++) {
		u64 tmp, err;

		tmp = div_u64((u64)USEC_PER_SEC * in->numerator *
			      skip->max_ratio, in->denominator * skip->keep);

		err = abs((s64)tmp - want_us);
		if (err < min_err) {
			min_err = err;
			best_skip = skip;
		}
	}

	*out = *in;
	csi_apply_skip_interval(best_skip, out);

	return best_skip;
}

/*
 * V4L2 subdev operations.
 */

static int csi_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fi)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);

	if (fi->pad >= CSI_NUM_PADS)
		return -EINVAL;

	mutex_lock(&priv->lock);

	fi->interval = priv->frame_interval[fi->pad];

	mutex_unlock(&priv->lock);

	return 0;
}

static int csi_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fi)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_fract *input_fi;
	int ret = 0;

	mutex_lock(&priv->lock);

	input_fi = &priv->frame_interval[CSI_SINK_PAD];

	switch (fi->pad) {
	case CSI_SINK_PAD:
		/* No limits on valid input frame intervals */
		if (fi->interval.numerator == 0 ||
		    fi->interval.denominator == 0)
			fi->interval = *input_fi;
		/* Reset output intervals and frame skipping ratio to 1:1 */
		priv->frame_interval[CSI_SRC_PAD_IDMAC] = fi->interval;
		priv->frame_interval[CSI_SRC_PAD_DIRECT] = fi->interval;
		priv->skip = &csi_skip[0];
		break;
	case CSI_SRC_PAD_IDMAC:
		/*
		 * frame interval at IDMAC output pad depends on input
		 * interval, modified by frame skipping.
		 */
		priv->skip = csi_find_best_skip(input_fi, &fi->interval);
		break;
	case CSI_SRC_PAD_DIRECT:
		/*
		 * frame interval at DIRECT output pad is same as input
		 * interval.
		 */
		fi->interval = *input_fi;
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

static int csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&priv->lock);

	if (!priv->src_sd || !priv->sink) {
		ret = -EPIPE;
		goto out;
	}

	/*
	 * enable/disable streaming only if stream_count is
	 * going from 0 to 1 / 1 to 0.
	 */
	if (priv->stream_count != !enable)
		goto update_count;

	if (enable) {
		dev_dbg(priv->dev, "stream ON\n");
		ret = csi_start(priv);
		if (ret)
			goto out;
	} else {
		dev_dbg(priv->dev, "stream OFF\n");
		csi_stop(priv);
	}

update_count:
	priv->stream_count += enable ? 1 : -1;
	if (priv->stream_count < 0)
		priv->stream_count = 0;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int csi_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote_sd;
	int ret = 0;

	dev_dbg(priv->dev, "link setup %s -> %s\n", remote->entity->name,
		local->entity->name);

	mutex_lock(&priv->lock);

	if (local->flags & MEDIA_PAD_FL_SINK) {
		if (!is_media_entity_v4l2_subdev(remote->entity)) {
			ret = -EINVAL;
			goto out;
		}

		remote_sd = media_entity_to_v4l2_subdev(remote->entity);

		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (priv->src_sd) {
				ret = -EBUSY;
				goto out;
			}
			priv->src_sd = remote_sd;
		} else {
			priv->src_sd = NULL;
		}

		goto out;
	}

	/* this is a source pad */

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (priv->sink) {
			ret = -EBUSY;
			goto out;
		}
	} else {
		priv->sink = NULL;
		/* do not apply IC burst alignment in csi_try_crop */
		priv->active_output_pad = CSI_SRC_PAD_IDMAC;
		goto out;
	}

	/* record which output pad is now active */
	priv->active_output_pad = local->index;

	/* set CSI destination */
	if (local->index == CSI_SRC_PAD_IDMAC) {
		if (!is_media_entity_v4l2_video_device(remote->entity)) {
			ret = -EINVAL;
			goto out;
		}

		priv->dest = IPU_CSI_DEST_IDMAC;
	} else {
		if (!is_media_entity_v4l2_subdev(remote->entity)) {
			ret = -EINVAL;
			goto out;
		}

		remote_sd = media_entity_to_v4l2_subdev(remote->entity);
		switch (remote_sd->grp_id) {
		case IMX_MEDIA_GRP_ID_IPU_VDIC:
			priv->dest = IPU_CSI_DEST_VDIC;
			break;
		case IMX_MEDIA_GRP_ID_IPU_IC_PRP:
			priv->dest = IPU_CSI_DEST_IC;
			break;
		default:
			ret = -EINVAL;
			goto out;
		}
	}

	priv->sink = remote->entity;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int csi_link_validate(struct v4l2_subdev *sd,
			     struct media_link *link,
			     struct v4l2_subdev_format *source_fmt,
			     struct v4l2_subdev_format *sink_fmt)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_fwnode_endpoint upstream_ep = { .bus_type = 0 };
	bool is_csi2;
	int ret;

	ret = v4l2_subdev_link_validate_default(sd, link,
						source_fmt, sink_fmt);
	if (ret)
		return ret;

	ret = csi_get_upstream_endpoint(priv, &upstream_ep);
	if (ret) {
		v4l2_err(&priv->sd, "failed to find upstream endpoint\n");
		return ret;
	}

	mutex_lock(&priv->lock);

	priv->upstream_ep = upstream_ep;
	is_csi2 = !is_parallel_bus(&upstream_ep);
	if (is_csi2) {
		int vc_num = 0;
		/*
		 * NOTE! It seems the virtual channels from the mipi csi-2
		 * receiver are used only for routing by the video mux's,
		 * or for hard-wired routing to the CSI's. Once the stream
		 * enters the CSI's however, they are treated internally
		 * in the IPU as virtual channel 0.
		 */
#if 0
		mutex_unlock(&priv->lock);
		vc_num = imx_media_find_mipi_csi2_channel(&priv->sd.entity);
		if (vc_num < 0)
			return vc_num;
		mutex_lock(&priv->lock);
#endif
		ipu_csi_set_mipi_datatype(priv->csi, vc_num,
					  &priv->format_mbus[CSI_SINK_PAD]);
	}

	/* select either parallel or MIPI-CSI2 as input to CSI */
	ipu_set_csi_src_mux(priv->ipu, priv->csi_id, is_csi2);

	mutex_unlock(&priv->lock);
	return ret;
}

static struct v4l2_mbus_framefmt *
__csi_get_fmt(struct csi_priv *priv, struct v4l2_subdev_pad_config *cfg,
	      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&priv->sd, cfg, pad);
	else
		return &priv->format_mbus[pad];
}

static struct v4l2_rect *
__csi_get_crop(struct csi_priv *priv, struct v4l2_subdev_pad_config *cfg,
	       enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_crop(&priv->sd, cfg, CSI_SINK_PAD);
	else
		return &priv->crop;
}

static struct v4l2_rect *
__csi_get_compose(struct csi_priv *priv, struct v4l2_subdev_pad_config *cfg,
		  enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_compose(&priv->sd, cfg,
						   CSI_SINK_PAD);
	else
		return &priv->compose;
}

static void csi_try_crop(struct csi_priv *priv,
			 struct v4l2_rect *crop,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_mbus_framefmt *infmt,
			 struct v4l2_fwnode_endpoint *upstream_ep)
{
	u32 in_height;

	crop->width = min_t(__u32, infmt->width, crop->width);
	if (crop->left + crop->width > infmt->width)
		crop->left = infmt->width - crop->width;
	/* adjust crop left/width to h/w alignment restrictions */
	crop->left &= ~0x3;
	if (priv->active_output_pad == CSI_SRC_PAD_DIRECT)
		crop->width &= ~0x7; /* multiple of 8 pixels (IC burst) */
	else
		crop->width &= ~0x1; /* multiple of 2 pixels */

	in_height = infmt->height;
	if (infmt->field == V4L2_FIELD_ALTERNATE)
		in_height *= 2;

	/*
	 * FIXME: not sure why yet, but on interlaced bt.656,
	 * changing the vertical cropping causes loss of vertical
	 * sync, so fix it to NTSC/PAL active lines. NTSC contains
	 * 2 extra lines of active video that need to be cropped.
	 */
	if (upstream_ep->bus_type == V4L2_MBUS_BT656 &&
	    (V4L2_FIELD_HAS_BOTH(infmt->field) ||
	     infmt->field == V4L2_FIELD_ALTERNATE)) {
		crop->height = in_height;
		crop->top = (in_height == 480) ? 2 : 0;
	} else {
		crop->height = min_t(__u32, in_height, crop->height);
		if (crop->top + crop->height > in_height)
			crop->top = in_height - crop->height;
	}
}

static int csi_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_fwnode_endpoint upstream_ep = { .bus_type = 0 };
	const struct imx_media_pixfmt *incc;
	struct v4l2_mbus_framefmt *infmt;
	int ret = 0;

	mutex_lock(&priv->lock);

	infmt = __csi_get_fmt(priv, cfg, CSI_SINK_PAD, code->which);
	incc = imx_media_find_mbus_format(infmt->code, PIXFMT_SEL_ANY);

	switch (code->pad) {
	case CSI_SINK_PAD:
		ret = imx_media_enum_mbus_formats(&code->code, code->index,
						  PIXFMT_SEL_ANY);
		break;
	case CSI_SRC_PAD_DIRECT:
	case CSI_SRC_PAD_IDMAC:
		ret = csi_get_upstream_endpoint(priv, &upstream_ep);
		if (ret) {
			v4l2_err(&priv->sd, "failed to find upstream endpoint\n");
			goto out;
		}

		if (requires_passthrough(&upstream_ep, infmt, incc)) {
			if (code->index != 0) {
				ret = -EINVAL;
				goto out;
			}
			code->code = infmt->code;
		} else {
			enum imx_pixfmt_sel fmt_sel =
				(incc->cs == IPUV3_COLORSPACE_YUV) ?
				PIXFMT_SEL_YUV : PIXFMT_SEL_RGB;

			ret = imx_media_enum_ipu_formats(&code->code,
							 code->index,
							 fmt_sel);
		}
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int csi_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_rect *crop;
	int ret = 0;

	if (fse->pad >= CSI_NUM_PADS ||
	    fse->index > (fse->pad == CSI_SINK_PAD ? 0 : 3))
		return -EINVAL;

	mutex_lock(&priv->lock);

	if (fse->pad == CSI_SINK_PAD) {
		fse->min_width = MIN_W;
		fse->max_width = MAX_W;
		fse->min_height = MIN_H;
		fse->max_height = MAX_H;
	} else {
		crop = __csi_get_crop(priv, cfg, fse->which);

		fse->min_width = fse->index & 1 ?
			crop->width / 2 : crop->width;
		fse->max_width = fse->min_width;
		fse->min_height = fse->index & 2 ?
			crop->height / 2 : crop->height;
		fse->max_height = fse->min_height;
	}

	mutex_unlock(&priv->lock);
	return ret;
}

static int csi_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_fract *input_fi;
	struct v4l2_rect *crop;
	int ret = 0;

	if (fie->pad >= CSI_NUM_PADS ||
	    fie->index >= (fie->pad != CSI_SRC_PAD_IDMAC ?
			   1 : ARRAY_SIZE(csi_skip)))
		return -EINVAL;

	mutex_lock(&priv->lock);

	input_fi = &priv->frame_interval[CSI_SINK_PAD];
	crop = __csi_get_crop(priv, cfg, fie->which);

	if ((fie->width != crop->width && fie->width != crop->width / 2) ||
	    (fie->height != crop->height && fie->height != crop->height / 2)) {
		ret = -EINVAL;
		goto out;
	}

	fie->interval = *input_fi;

	if (fie->pad == CSI_SRC_PAD_IDMAC)
		csi_apply_skip_interval(&csi_skip[fie->index],
					&fie->interval);

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int csi_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *sdformat)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	if (sdformat->pad >= CSI_NUM_PADS)
		return -EINVAL;

	mutex_lock(&priv->lock);

	fmt = __csi_get_fmt(priv, cfg, sdformat->pad, sdformat->which);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}

	sdformat->format = *fmt;
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static void csi_try_field(struct csi_priv *priv,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *sdformat)
{
	struct v4l2_mbus_framefmt *infmt =
		__csi_get_fmt(priv, cfg, CSI_SINK_PAD, sdformat->which);

	/*
	 * no restrictions on sink pad field type except must
	 * be initialized.
	 */
	if (sdformat->pad == CSI_SINK_PAD) {
		if (sdformat->format.field == V4L2_FIELD_ANY)
			sdformat->format.field = V4L2_FIELD_NONE;
		return;
	}

	switch (infmt->field) {
	case V4L2_FIELD_SEQ_TB:
	case V4L2_FIELD_SEQ_BT:
		/*
		 * If the user requests sequential at the source pad,
		 * allow it (along with possibly inverting field order).
		 * Otherwise passthrough the field type.
		 */
		if (!V4L2_FIELD_IS_SEQUENTIAL(sdformat->format.field))
			sdformat->format.field = infmt->field;
		break;
	case V4L2_FIELD_ALTERNATE:
		/*
		 * This driver does not support alternate field mode, and
		 * the CSI captures a whole frame, so the CSI never presents
		 * alternate mode at its source pads. If user has not
		 * already requested sequential, translate ALTERNATE at
		 * sink pad to SEQ_TB or SEQ_BT at the source pad depending
		 * on input height (assume NTSC BT order if 480 total active
		 * frame lines, otherwise PAL TB order).
		 */
		if (!V4L2_FIELD_IS_SEQUENTIAL(sdformat->format.field))
			sdformat->format.field = (infmt->height == 480 / 2) ?
				V4L2_FIELD_SEQ_BT : V4L2_FIELD_SEQ_TB;
		break;
	default:
		/* Passthrough for all other input field types */
		sdformat->format.field = infmt->field;
		break;
	}
}

static void csi_try_fmt(struct csi_priv *priv,
			struct v4l2_fwnode_endpoint *upstream_ep,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *sdformat,
			struct v4l2_rect *crop,
			struct v4l2_rect *compose,
			const struct imx_media_pixfmt **cc)
{
	const struct imx_media_pixfmt *incc;
	struct v4l2_mbus_framefmt *infmt;
	u32 code;

	infmt = __csi_get_fmt(priv, cfg, CSI_SINK_PAD, sdformat->which);

	switch (sdformat->pad) {
	case CSI_SRC_PAD_DIRECT:
	case CSI_SRC_PAD_IDMAC:
		incc = imx_media_find_mbus_format(infmt->code, PIXFMT_SEL_ANY);

		sdformat->format.width = compose->width;
		sdformat->format.height = compose->height;

		if (requires_passthrough(upstream_ep, infmt, incc)) {
			sdformat->format.code = infmt->code;
			*cc = incc;
		} else {
			enum imx_pixfmt_sel fmt_sel =
				(incc->cs == IPUV3_COLORSPACE_YUV) ?
				PIXFMT_SEL_YUV : PIXFMT_SEL_RGB;

			*cc = imx_media_find_ipu_format(sdformat->format.code,
							fmt_sel);
			if (!*cc) {
				imx_media_enum_ipu_formats(&code, 0, fmt_sel);
				*cc = imx_media_find_ipu_format(code, fmt_sel);
				sdformat->format.code = (*cc)->codes[0];
			}
		}

		csi_try_field(priv, cfg, sdformat);

		/* propagate colorimetry from sink */
		sdformat->format.colorspace = infmt->colorspace;
		sdformat->format.xfer_func = infmt->xfer_func;
		sdformat->format.quantization = infmt->quantization;
		sdformat->format.ycbcr_enc = infmt->ycbcr_enc;

		break;
	case CSI_SINK_PAD:
		v4l_bound_align_image(&sdformat->format.width, MIN_W, MAX_W,
				      W_ALIGN, &sdformat->format.height,
				      MIN_H, MAX_H, H_ALIGN, S_ALIGN);

		*cc = imx_media_find_mbus_format(sdformat->format.code,
						 PIXFMT_SEL_ANY);
		if (!*cc) {
			imx_media_enum_mbus_formats(&code, 0,
						    PIXFMT_SEL_YUV_RGB);
			*cc = imx_media_find_mbus_format(code,
							 PIXFMT_SEL_YUV_RGB);
			sdformat->format.code = (*cc)->codes[0];
		}

		csi_try_field(priv, cfg, sdformat);

		/* Reset crop and compose rectangles */
		crop->left = 0;
		crop->top = 0;
		crop->width = sdformat->format.width;
		crop->height = sdformat->format.height;
		if (sdformat->format.field == V4L2_FIELD_ALTERNATE)
			crop->height *= 2;
		csi_try_crop(priv, crop, cfg, &sdformat->format, upstream_ep);
		compose->left = 0;
		compose->top = 0;
		compose->width = crop->width;
		compose->height = crop->height;

		break;
	}

	imx_media_try_colorimetry(&sdformat->format,
			priv->active_output_pad == CSI_SRC_PAD_DIRECT);
}

static int csi_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *sdformat)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_fwnode_endpoint upstream_ep = { .bus_type = 0 };
	const struct imx_media_pixfmt *cc;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop, *compose;
	int ret;

	if (sdformat->pad >= CSI_NUM_PADS)
		return -EINVAL;

	ret = csi_get_upstream_endpoint(priv, &upstream_ep);
	if (ret) {
		v4l2_err(&priv->sd, "failed to find upstream endpoint\n");
		return ret;
	}

	mutex_lock(&priv->lock);

	if (priv->stream_count > 0) {
		ret = -EBUSY;
		goto out;
	}

	crop = __csi_get_crop(priv, cfg, sdformat->which);
	compose = __csi_get_compose(priv, cfg, sdformat->which);

	csi_try_fmt(priv, &upstream_ep, cfg, sdformat, crop, compose, &cc);

	fmt = __csi_get_fmt(priv, cfg, sdformat->pad, sdformat->which);
	*fmt = sdformat->format;

	if (sdformat->pad == CSI_SINK_PAD) {
		int pad;

		/* propagate format to source pads */
		for (pad = CSI_SINK_PAD + 1; pad < CSI_NUM_PADS; pad++) {
			const struct imx_media_pixfmt *outcc;
			struct v4l2_mbus_framefmt *outfmt;
			struct v4l2_subdev_format format;

			format.pad = pad;
			format.which = sdformat->which;
			format.format = sdformat->format;
			csi_try_fmt(priv, &upstream_ep, cfg, &format,
				    NULL, compose, &outcc);

			outfmt = __csi_get_fmt(priv, cfg, pad, sdformat->which);
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

static int csi_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *infmt;
	struct v4l2_rect *crop, *compose;
	int ret = 0;

	if (sel->pad != CSI_SINK_PAD)
		return -EINVAL;

	mutex_lock(&priv->lock);

	infmt = __csi_get_fmt(priv, cfg, CSI_SINK_PAD, sel->which);
	crop = __csi_get_crop(priv, cfg, sel->which);
	compose = __csi_get_compose(priv, cfg, sel->which);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = infmt->width;
		sel->r.height = infmt->height;
		if (infmt->field == V4L2_FIELD_ALTERNATE)
			sel->r.height *= 2;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = *crop;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = crop->width;
		sel->r.height = crop->height;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		sel->r = *compose;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&priv->lock);
	return ret;
}

static int csi_set_scale(u32 *compose, u32 crop, u32 flags)
{
	if ((flags & (V4L2_SEL_FLAG_LE | V4L2_SEL_FLAG_GE)) ==
		     (V4L2_SEL_FLAG_LE | V4L2_SEL_FLAG_GE) &&
	    *compose != crop && *compose != crop / 2)
		return -ERANGE;

	if (*compose <= crop / 2 ||
	    (*compose < crop * 3 / 4 && !(flags & V4L2_SEL_FLAG_GE)) ||
	    (*compose < crop && (flags & V4L2_SEL_FLAG_LE)))
		*compose = crop / 2;
	else
		*compose = crop;

	return 0;
}

static int csi_set_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_fwnode_endpoint upstream_ep = { .bus_type = 0 };
	struct v4l2_mbus_framefmt *infmt;
	struct v4l2_rect *crop, *compose;
	int pad, ret;

	if (sel->pad != CSI_SINK_PAD)
		return -EINVAL;

	ret = csi_get_upstream_endpoint(priv, &upstream_ep);
	if (ret) {
		v4l2_err(&priv->sd, "failed to find upstream endpoint\n");
		return ret;
	}

	mutex_lock(&priv->lock);

	if (priv->stream_count > 0) {
		ret = -EBUSY;
		goto out;
	}

	infmt = __csi_get_fmt(priv, cfg, CSI_SINK_PAD, sel->which);
	crop = __csi_get_crop(priv, cfg, sel->which);
	compose = __csi_get_compose(priv, cfg, sel->which);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		/*
		 * Modifying the crop rectangle always changes the format on
		 * the source pads. If the KEEP_CONFIG flag is set, just return
		 * the current crop rectangle.
		 */
		if (sel->flags & V4L2_SEL_FLAG_KEEP_CONFIG) {
			sel->r = priv->crop;
			if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
				*crop = sel->r;
			goto out;
		}

		csi_try_crop(priv, &sel->r, cfg, infmt, &upstream_ep);

		*crop = sel->r;

		/* Reset scaling to 1:1 */
		compose->width = crop->width;
		compose->height = crop->height;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		/*
		 * Modifying the compose rectangle always changes the format on
		 * the source pads. If the KEEP_CONFIG flag is set, just return
		 * the current compose rectangle.
		 */
		if (sel->flags & V4L2_SEL_FLAG_KEEP_CONFIG) {
			sel->r = priv->compose;
			if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
				*compose = sel->r;
			goto out;
		}

		sel->r.left = 0;
		sel->r.top = 0;
		ret = csi_set_scale(&sel->r.width, crop->width, sel->flags);
		if (ret)
			goto out;
		ret = csi_set_scale(&sel->r.height, crop->height, sel->flags);
		if (ret)
			goto out;

		*compose = sel->r;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	/* Reset source pads to sink compose rectangle */
	for (pad = CSI_SINK_PAD + 1; pad < CSI_NUM_PADS; pad++) {
		struct v4l2_mbus_framefmt *outfmt;

		outfmt = __csi_get_fmt(priv, cfg, pad, sel->which);
		outfmt->width = compose->width;
		outfmt->height = compose->height;
	}

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int csi_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
			       struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_IMX_FRAME_INTERVAL_ERROR)
		return -EINVAL;
	if (sub->id != 0)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 0, NULL);
}

static int csi_unsubscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				 struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static int csi_registered(struct v4l2_subdev *sd)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct ipu_csi *csi;
	int i, ret;
	u32 code;

	/* get handle to IPU CSI */
	csi = ipu_csi_get(priv->ipu, priv->csi_id);
	if (IS_ERR(csi)) {
		v4l2_err(&priv->sd, "failed to get CSI%d\n", priv->csi_id);
		return PTR_ERR(csi);
	}
	priv->csi = csi;

	for (i = 0; i < CSI_NUM_PADS; i++) {
		code = 0;
		if (i != CSI_SINK_PAD)
			imx_media_enum_ipu_formats(&code, 0, PIXFMT_SEL_YUV);

		/* set a default mbus format  */
		ret = imx_media_init_mbus_fmt(&priv->format_mbus[i],
					      640, 480, code, V4L2_FIELD_NONE,
					      &priv->cc[i]);
		if (ret)
			goto put_csi;

		/* init default frame interval */
		priv->frame_interval[i].numerator = 1;
		priv->frame_interval[i].denominator = 30;
	}

	/* disable frame skipping */
	priv->skip = &csi_skip[0];

	/* init default crop and compose rectangle sizes */
	priv->crop.width = 640;
	priv->crop.height = 480;
	priv->compose.width = 640;
	priv->compose.height = 480;

	priv->vdev = imx_media_capture_device_init(priv->sd.dev,
						   &priv->sd,
						   CSI_SRC_PAD_IDMAC);
	if (IS_ERR(priv->vdev)) {
		ret = PTR_ERR(priv->vdev);
		goto put_csi;
	}

	ret = imx_media_capture_device_register(priv->vdev);
	if (ret)
		goto remove_vdev;

	return 0;

remove_vdev:
	imx_media_capture_device_remove(priv->vdev);
put_csi:
	ipu_csi_put(priv->csi);
	return ret;
}

static void csi_unregistered(struct v4l2_subdev *sd)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);

	imx_media_capture_device_unregister(priv->vdev);
	imx_media_capture_device_remove(priv->vdev);

	if (priv->csi)
		ipu_csi_put(priv->csi);
}

/*
 * The CSI has only one fwnode endpoint, at the sink pad. Verify the
 * endpoint belongs to us, and return CSI_SINK_PAD.
 */
static int csi_get_fwnode_pad(struct media_entity *entity,
			      struct fwnode_endpoint *endpoint)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct fwnode_handle *csi_port = dev_fwnode(priv->dev);
	struct fwnode_handle *csi_ep;
	int ret;

	csi_ep = fwnode_get_next_child_node(csi_port, NULL);

	ret = endpoint->local_fwnode == csi_ep ? CSI_SINK_PAD : -ENXIO;

	fwnode_handle_put(csi_ep);

	return ret;
}

static const struct media_entity_operations csi_entity_ops = {
	.link_setup = csi_link_setup,
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = csi_get_fwnode_pad,
};

static const struct v4l2_subdev_core_ops csi_core_ops = {
	.subscribe_event = csi_subscribe_event,
	.unsubscribe_event = csi_unsubscribe_event,
};

static const struct v4l2_subdev_video_ops csi_video_ops = {
	.g_frame_interval = csi_g_frame_interval,
	.s_frame_interval = csi_s_frame_interval,
	.s_stream = csi_s_stream,
};

static const struct v4l2_subdev_pad_ops csi_pad_ops = {
	.init_cfg = imx_media_init_cfg,
	.enum_mbus_code = csi_enum_mbus_code,
	.enum_frame_size = csi_enum_frame_size,
	.enum_frame_interval = csi_enum_frame_interval,
	.get_fmt = csi_get_fmt,
	.set_fmt = csi_set_fmt,
	.get_selection = csi_get_selection,
	.set_selection = csi_set_selection,
	.link_validate = csi_link_validate,
};

static const struct v4l2_subdev_ops csi_subdev_ops = {
	.core = &csi_core_ops,
	.video = &csi_video_ops,
	.pad = &csi_pad_ops,
};

static const struct v4l2_subdev_internal_ops csi_internal_ops = {
	.registered = csi_registered,
	.unregistered = csi_unregistered,
};

static int imx_csi_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_subdev *asd)
{
	struct csi_priv *priv = notifier_to_dev(notifier);
	struct media_pad *sink = &priv->sd.entity.pads[CSI_SINK_PAD];

	/*
	 * If the subdev is a video mux, it must be one of the CSI
	 * muxes. Mark it as such via its group id.
	 */
	if (sd->entity.function == MEDIA_ENT_F_VID_MUX)
		sd->grp_id = IMX_MEDIA_GRP_ID_CSI_MUX;

	return v4l2_create_fwnode_links_to_pad(sd, sink);
}

static const struct v4l2_async_notifier_operations csi_notify_ops = {
	.bound = imx_csi_notify_bound,
};

static int imx_csi_async_register(struct csi_priv *priv)
{
	struct v4l2_async_subdev *asd = NULL;
	struct fwnode_handle *ep;
	unsigned int port;
	int ret;

	v4l2_async_notifier_init(&priv->notifier);

	/* get this CSI's port id */
	ret = fwnode_property_read_u32(dev_fwnode(priv->dev), "reg", &port);
	if (ret < 0)
		return ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(priv->dev->parent),
					     port, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (ep) {
		asd = kzalloc(sizeof(*asd), GFP_KERNEL);
		if (!asd) {
			fwnode_handle_put(ep);
			return -ENOMEM;
		}

		ret = v4l2_async_notifier_add_fwnode_remote_subdev(
			&priv->notifier, ep, asd);

		fwnode_handle_put(ep);

		if (ret) {
			kfree(asd);
			/* OK if asd already exists */
			if (ret != -EEXIST)
				return ret;
		}
	}

	priv->notifier.ops = &csi_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd,
						  &priv->notifier);
	if (ret)
		return ret;

	return v4l2_async_register_subdev(&priv->sd);
}

static int imx_csi_probe(struct platform_device *pdev)
{
	struct ipu_client_platformdata *pdata;
	struct pinctrl *pinctrl;
	struct csi_priv *priv;
	int i, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, &priv->sd);
	priv->dev = &pdev->dev;

	ret = dma_set_coherent_mask(priv->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* get parent IPU */
	priv->ipu = dev_get_drvdata(priv->dev->parent);

	/* get our CSI id */
	pdata = priv->dev->platform_data;
	priv->csi_id = pdata->csi;

	priv->active_output_pad = CSI_SRC_PAD_IDMAC;

	timer_setup(&priv->eof_timeout_timer, csi_idmac_eof_timeout, 0);
	spin_lock_init(&priv->irqlock);

	v4l2_subdev_init(&priv->sd, &csi_subdev_ops);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.internal_ops = &csi_internal_ops;
	priv->sd.entity.ops = &csi_entity_ops;
	priv->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	priv->sd.dev = &pdev->dev;
	priv->sd.fwnode = of_fwnode_handle(pdata->of_node);
	priv->sd.owner = THIS_MODULE;
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	priv->sd.grp_id = priv->csi_id ?
		IMX_MEDIA_GRP_ID_IPU_CSI1 : IMX_MEDIA_GRP_ID_IPU_CSI0;
	imx_media_grp_id_to_sd_name(priv->sd.name, sizeof(priv->sd.name),
				    priv->sd.grp_id, ipu_get_num(priv->ipu));

	for (i = 0; i < CSI_NUM_PADS; i++)
		priv->pad[i].flags = (i == CSI_SINK_PAD) ?
			MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, CSI_NUM_PADS,
				     priv->pad);
	if (ret)
		return ret;

	mutex_init(&priv->lock);

	/*
	 * The IPUv3 driver did not assign an of_node to this
	 * device. As a result, pinctrl does not automatically
	 * configure our pin groups, so we need to do that manually
	 * here, after setting this device's of_node.
	 */
	priv->dev->of_node = pdata->of_node;
	pinctrl = devm_pinctrl_get_select_default(priv->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_dbg(priv->dev,
			"devm_pinctrl_get_select_default() failed: %d\n", ret);
		if (ret != -ENODEV)
			goto free;
	}

	ret = imx_csi_async_register(priv);
	if (ret)
		goto cleanup;

	return 0;

cleanup:
	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
free:
	mutex_destroy(&priv->lock);
	return ret;
}

static int imx_csi_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct csi_priv *priv = sd_to_dev(sd);

	mutex_destroy(&priv->lock);
	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct platform_device_id imx_csi_ids[] = {
	{ .name = "imx-ipuv3-csi" },
	{ },
};
MODULE_DEVICE_TABLE(platform, imx_csi_ids);

static struct platform_driver imx_csi_driver = {
	.probe = imx_csi_probe,
	.remove = imx_csi_remove,
	.id_table = imx_csi_ids,
	.driver = {
		.name = "imx-ipuv3-csi",
	},
};
module_platform_driver(imx_csi_driver);

MODULE_DESCRIPTION("i.MX CSI subdev driver");
MODULE_AUTHOR("Steve Longerbeam <steve_longerbeam@mentor.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ipuv3-csi");
