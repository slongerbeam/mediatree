// SPDX-License-Identifier: GPL-2.0+
/*
 * Video Capture/Output Device for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2012-2018 Mentor Graphics Inc.
 *
 */
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <video/imx-ipu-v3.h>
#include <media/imx.h>
#include "imx-media.h"

struct imx_video_priv {
	struct imx_media_video_dev vdev;
	enum v4l2_buf_type    type;

	struct v4l2_subdev    *sd;
	int                   sd_pad;
	struct device         *dev;

	struct imx_media_dev  *md;

	struct media_pad      vdev_pad;

	struct mutex          mutex;       /* device mutex */

	/* the videobuf2 queue */
	struct vb2_queue       q;
	/* list of ready imx_media_buffer's from q */
	struct list_head       ready_q;
	/* protect ready_q */
	spinlock_t             q_lock;

	/* controls inherited from subdevs */
	struct v4l2_ctrl_handler ctrl_hdlr;
};

#define to_imx_video_priv(v) container_of(v, struct imx_video_priv, vdev)

/* In bytes, per queue */
#define VID_MEM_LIMIT	SZ_64M

static const struct vb2_ops imx_vdev_qops;

/*
 * Video ioctls follow
 */

static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct imx_video_priv *priv = video_drvdata(file);

	strscpy(cap->driver, "imx-media-video", sizeof(cap->driver));
	strscpy(cap->card, "imx-media-video", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", priv->sd->name);

	if (priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE;
	else
		cap->device_caps = V4L2_CAP_VIDEO_OUTPUT;

	cap->capabilities = cap->device_caps |
		V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int imx_vdev_enum_framesizes(struct file *file, void *fh,
				    struct v4l2_frmsizeenum *fsize)
{
	struct imx_video_priv *priv = video_drvdata(file);
	const struct imx_media_pixfmt *cc;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.pad = priv->sd_pad,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	cc = imx_media_find_format(fsize->pixel_format, CS_SEL_ANY, true);
	if (!cc)
		return -EINVAL;

	fse.code = cc->codes[0];

	ret = v4l2_subdev_call(priv->sd, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	if (fse.min_width == fse.max_width &&
	    fse.min_height == fse.max_height) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.min_width;
		fsize->discrete.height = fse.min_height;
	} else {
		fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
		fsize->stepwise.min_width = fse.min_width;
		fsize->stepwise.max_width = fse.max_width;
		fsize->stepwise.min_height = fse.min_height;
		fsize->stepwise.max_height = fse.max_height;
		fsize->stepwise.step_width = 1;
		fsize->stepwise.step_height = 1;
	}

	return 0;
}

static int imx_vdev_enum_frameintervals(struct file *file, void *fh,
					struct v4l2_frmivalenum *fival)
{
	struct imx_video_priv *priv = video_drvdata(file);
	const struct imx_media_pixfmt *cc;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = fival->index,
		.pad = priv->sd_pad,
		.width = fival->width,
		.height = fival->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	cc = imx_media_find_format(fival->pixel_format, CS_SEL_ANY, true);
	if (!cc)
		return -EINVAL;

	fie.code = cc->codes[0];

	ret = v4l2_subdev_call(priv->sd, pad, enum_frame_interval, NULL, &fie);
	if (ret)
		return ret;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = fie.interval;

	return 0;
}

static int imx_vdev_enum_fmt(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	struct imx_video_priv *priv = video_drvdata(file);
	const struct imx_media_pixfmt *sd_cc;
	struct v4l2_subdev_format sd_fmt;
	u32 fourcc;
	int ret;

	sd_fmt.pad = priv->sd_pad;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(priv->sd, pad, get_fmt, NULL, &sd_fmt);
	if (ret) {
		v4l2_err(priv->sd, "failed to get sd format\n");
		return ret;
	}

	sd_cc = imx_media_find_ipu_format(sd_fmt.format.code, CS_SEL_ANY);
	if (sd_cc) {
		u32 cs_sel = (sd_cc->cs == IPUV3_COLORSPACE_YUV) ?
			CS_SEL_YUV : CS_SEL_RGB;

		ret = imx_media_enum_format(&fourcc, f->index, cs_sel);
		if (ret)
			return ret;
	} else {
		sd_cc = imx_media_find_mbus_format(sd_fmt.format.code,
						   CS_SEL_ANY, true);
		if (WARN_ON(!sd_cc))
			return -EINVAL;

		if (f->index != 0)
			return -EINVAL;
		fourcc = sd_cc->fourcc;
	}

	f->pixelformat = fourcc;

	return 0;
}

static int imx_vdev_g_fmt(struct file *file, void *fh,
			  struct v4l2_format *f)
{
	struct imx_video_priv *priv = video_drvdata(file);

	*f = priv->vdev.fmt;

	return 0;
}

static int __imx_vdev_try_fmt(struct imx_video_priv *priv,
			      struct v4l2_subdev_format *sd_fmt,
			      struct v4l2_format *f,
			      const struct imx_media_pixfmt **retcc,
			      struct v4l2_rect *compose)
{
	const struct imx_media_pixfmt *cc, *sd_cc;

	sd_cc = imx_media_find_ipu_format(sd_fmt->format.code, CS_SEL_ANY);
	if (sd_cc) {
		u32 fourcc, cs_sel;

		cs_sel = (sd_cc->cs == IPUV3_COLORSPACE_YUV) ?
			CS_SEL_YUV : CS_SEL_RGB;
		fourcc = f->fmt.pix.pixelformat;

		cc = imx_media_find_format(fourcc, cs_sel, false);
		if (!cc) {
			imx_media_enum_format(&fourcc, 0, cs_sel);
			cc = imx_media_find_format(fourcc, cs_sel, false);
		}
	} else {
		sd_cc = imx_media_find_mbus_format(sd_fmt->format.code,
						   CS_SEL_ANY, true);
		if (WARN_ON(!sd_cc))
			return -EINVAL;

		cc = sd_cc;
	}

	/*
	 * allow IDMAC interweave for capture but enforce field order
	 * from source
	 */
	if (priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    V4L2_FIELD_IS_INTERLACED(f->fmt.pix.field)) {
		switch (sd_fmt->format.field) {
		case V4L2_FIELD_SEQ_TB:
			sd_fmt->format.field = V4L2_FIELD_INTERLACED_TB;
			break;
		case V4L2_FIELD_SEQ_BT:
			sd_fmt->format.field = V4L2_FIELD_INTERLACED_BT;
			break;
		default:
			break;
		}
	}

	imx_media_mbus_fmt_to_pix_fmt(&f->fmt.pix, &sd_fmt->format, cc);

	if (retcc)
		*retcc = cc;

	if (compose) {
		compose->left = 0;
		compose->top = 0;
		compose->width = sd_fmt->format.width;
		compose->height = sd_fmt->format.height;
	}

	return 0;
}

static int imx_vdev_try_fmt(struct file *file, void *fh,
			    struct v4l2_format *f)
{
	struct imx_video_priv *priv = video_drvdata(file);
	struct v4l2_subdev_format sd_fmt;
	int ret;

	sd_fmt.pad = priv->sd_pad;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(priv->sd, pad, get_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;

	return __imx_vdev_try_fmt(priv, &sd_fmt, f, NULL, NULL);
}

static int imx_vdev_s_fmt(struct file *file, void *fh,
			  struct v4l2_format *f)
{
	struct imx_video_priv *priv = video_drvdata(file);
	struct v4l2_subdev_format sd_fmt;
	struct v4l2_rect *compose;
	int ret;

	if (vb2_is_busy(&priv->q)) {
		v4l2_err(priv->sd, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	sd_fmt.pad = priv->sd_pad;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(priv->sd, pad, get_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;

	compose = (priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
		&priv->vdev.compose : NULL;

	ret = __imx_vdev_try_fmt(priv, &sd_fmt, f, &priv->vdev.cc, compose);
	if (ret)
		return ret;

	priv->vdev.fmt.fmt.pix = f->fmt.pix;

	return 0;
}

static int imx_vdev_querystd(struct file *file, void *fh, v4l2_std_id *std)
{
	struct imx_video_priv *priv = video_drvdata(file);

	return v4l2_subdev_call(priv->sd, video, querystd, std);
}

static int imx_vdev_g_std(struct file *file, void *fh, v4l2_std_id *std)
{
	struct imx_video_priv *priv = video_drvdata(file);

	return v4l2_subdev_call(priv->sd, video, g_std, std);
}

static int imx_vdev_s_std(struct file *file, void *fh, v4l2_std_id std)
{
	struct imx_video_priv *priv = video_drvdata(file);

	if (vb2_is_busy(&priv->q))
		return -EBUSY;

	return v4l2_subdev_call(priv->sd, video, s_std, std);
}

static int imx_vdev_g_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	struct imx_video_priv *priv = video_drvdata(file);

	if (priv->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		/* The compose rectangle is fixed to the source format. */
		s->r = priv->vdev.compose;
		break;
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		/*
		 * The hardware writes with a configurable but fixed DMA burst
		 * size. If the source format width is not burst size aligned,
		 * the written frame contains padding to the right.
		 */
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = priv->vdev.fmt.fmt.pix.width;
		s->r.height = priv->vdev.fmt.fmt.pix.height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imx_vdev_g_parm(struct file *file, void *fh,
			   struct v4l2_streamparm *a)
{
	struct imx_video_priv *priv = video_drvdata(file);
	struct v4l2_subdev_frame_interval fi;
	int ret;

	if (priv->type != a->type)
		return -EINVAL;

	memset(&fi, 0, sizeof(fi));
	fi.pad = priv->sd_pad;
	ret = v4l2_subdev_call(priv->sd, video, g_frame_interval, &fi);
	if (ret < 0)
		return ret;

	if (a->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		a->parm.capture.timeperframe = fi.interval;
	} else {
		a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		a->parm.output.timeperframe = fi.interval;
	}

	return 0;
}

static int imx_vdev_s_parm(struct file *file, void *fh,
			   struct v4l2_streamparm *a)
{
	struct imx_video_priv *priv = video_drvdata(file);
	struct v4l2_subdev_frame_interval fi;
	int ret;

	if (priv->type != a->type)
		return -EINVAL;

	memset(&fi, 0, sizeof(fi));
	fi.pad = priv->sd_pad;
	fi.interval = (a->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
		a->parm.capture.timeperframe : a->parm.output.timeperframe;
	ret = v4l2_subdev_call(priv->sd, video, s_frame_interval, &fi);
	if (ret < 0)
		return ret;

	if (a->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		a->parm.capture.timeperframe = fi.interval;
	} else {
		a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		a->parm.output.timeperframe = fi.interval;
	}

	return 0;
}

static int imx_vdev_subscribe_event(struct v4l2_fh *fh,
				    const struct v4l2_event_subscription *sub)
{
	struct imx_media_video_dev *vdev =
		to_imx_media_video_dev(fh->vdev);
	struct imx_video_priv *priv = to_imx_video_priv(vdev);

	if (priv->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (sub->type) {
	case V4L2_EVENT_IMX_FRAME_INTERVAL_ERROR:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops imx_vdev_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_framesizes = imx_vdev_enum_framesizes,
	.vidioc_enum_frameintervals = imx_vdev_enum_frameintervals,

	.vidioc_enum_fmt_vid_cap = imx_vdev_enum_fmt,
	.vidioc_g_fmt_vid_cap    = imx_vdev_g_fmt,
	.vidioc_try_fmt_vid_cap  = imx_vdev_try_fmt,
	.vidioc_s_fmt_vid_cap    = imx_vdev_s_fmt,

	.vidioc_enum_fmt_vid_out = imx_vdev_enum_fmt,
	.vidioc_g_fmt_vid_out    = imx_vdev_g_fmt,
	.vidioc_try_fmt_vid_out  = imx_vdev_try_fmt,
	.vidioc_s_fmt_vid_out    = imx_vdev_s_fmt,

	.vidioc_querystd        = imx_vdev_querystd,
	.vidioc_g_std           = imx_vdev_g_std,
	.vidioc_s_std           = imx_vdev_s_std,

	.vidioc_g_selection     = imx_vdev_g_selection,

	.vidioc_g_parm          = imx_vdev_g_parm,
	.vidioc_s_parm          = imx_vdev_s_parm,

	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_create_bufs     = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf     = vb2_ioctl_prepare_buf,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,
	.vidioc_expbuf		= vb2_ioctl_expbuf,
	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,

	.vidioc_subscribe_event = imx_vdev_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/*
 * Queue operations
 */

static int imx_vdev_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct imx_video_priv *priv = vb2_get_drv_priv(vq);
	struct v4l2_pix_format *pix = &priv->vdev.fmt.fmt.pix;
	unsigned int count = *nbuffers;

	if (vq->type != priv->type)
		return -EINVAL;

	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < pix->sizeimage)
			return -EINVAL;
		count += vq->num_buffers;
	}

	count = min_t(__u32, VID_MEM_LIMIT / pix->sizeimage, count);

	if (*nplanes)
		*nbuffers = (count < vq->num_buffers) ? 0 :
			count - vq->num_buffers;
	else
		*nbuffers = count;

	*nplanes = 1;
	sizes[0] = pix->sizeimage;

	return 0;
}

static int imx_vdev_buf_init(struct vb2_buffer *vb)
{
	struct imx_media_buffer *buf = to_imx_media_vb(vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int imx_vdev_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct imx_video_priv *priv = vb2_get_drv_priv(vq);
	struct v4l2_pix_format *pix = &priv->vdev.fmt.fmt.pix;

	if (vb2_plane_size(vb, 0) < pix->sizeimage) {
		v4l2_err(priv->sd,
			 "data will not fit into plane (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), (long)pix->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, pix->sizeimage);

	return 0;
}

static void imx_vdev_buf_queue(struct vb2_buffer *vb)
{
	struct imx_video_priv *priv = vb2_get_drv_priv(vb->vb2_queue);
	struct imx_media_buffer *buf = to_imx_media_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&priv->q_lock, flags);

	list_add_tail(&buf->list, &priv->ready_q);

	spin_unlock_irqrestore(&priv->q_lock, flags);
}

static int imx_vdev_validate_fmt(struct imx_video_priv *priv)
{
	const struct imx_media_pixfmt *cc;
	struct v4l2_subdev_format fmt;
	struct v4l2_rect compose;
	struct v4l2_format f;
	int ret;

	fmt.pad = priv->sd_pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(priv->sd, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	v4l2_fill_pix_format(&f.fmt.pix, &fmt.format);

	ret = __imx_vdev_try_fmt(priv, &fmt, &f, &cc,
				 priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
				 &compose : NULL);
	if (ret)
		return ret;

	return (priv->vdev.fmt.fmt.pix.width != f.fmt.pix.width ||
		priv->vdev.fmt.fmt.pix.height != f.fmt.pix.height ||
		priv->vdev.cc->cs != cc->cs ||
		(priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		 (priv->vdev.compose.width != compose.width ||
		  priv->vdev.compose.height != compose.height))) ?
		-EINVAL : 0;
}

static int imx_vdev_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct imx_video_priv *priv = vb2_get_drv_priv(vq);
	struct imx_media_buffer *buf, *tmp;
	unsigned long flags;
	int ret;

	ret = imx_vdev_validate_fmt(priv);
	if (ret) {
		v4l2_err(priv->sd, "format not valid\n");
		goto return_bufs;
	}

	/*
	 * in a memory-to-memory pipeline, capture side starts the
	 * whole pipeline. Output side doesn't need to do anything.
	 */
	if (priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ret = imx_media_pipeline_set_stream(priv->md,
						    &priv->sd->entity,
						    true);
		if (ret) {
			v4l2_err(priv->sd, "pipeline start failed with %d\n",
				 ret);
			goto return_bufs;
		}
	}

	return 0;

return_bufs:
	spin_lock_irqsave(&priv->q_lock, flags);
	list_for_each_entry_safe(buf, tmp, &priv->ready_q, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_QUEUED);
	}
	spin_unlock_irqrestore(&priv->q_lock, flags);
	return ret;
}

static void imx_vdev_stop_streaming(struct vb2_queue *vq)
{
	struct imx_video_priv *priv = vb2_get_drv_priv(vq);
	struct imx_media_buffer *frame;
	struct imx_media_buffer *tmp;
	unsigned long flags;
	int ret;

	/*
	 * in a memory-to-memory pipeline, capture side stops the whole
	 * pipeline, output side stops only its connected subdevice.
	 */
	if (priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ret = imx_media_pipeline_set_stream(priv->md,
						    &priv->sd->entity,
						    false);
		if (ret)
			v4l2_warn(priv->sd, "pipeline stop failed with %d\n",
				  ret);
	} else {
		ret = v4l2_subdev_call(priv->sd, video, s_stream, 0);
		if (ret)
			v4l2_warn(priv->sd, "%s stop failed with %d\n",
				  priv->sd->name, ret);
	}

	/* release all active buffers */
	spin_lock_irqsave(&priv->q_lock, flags);
	list_for_each_entry_safe(frame, tmp, &priv->ready_q, list) {
		list_del(&frame->list);
		vb2_buffer_done(&frame->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&priv->q_lock, flags);
}

static const struct vb2_ops imx_vdev_qops = {
	.queue_setup	 = imx_vdev_queue_setup,
	.buf_init        = imx_vdev_buf_init,
	.buf_prepare	 = imx_vdev_buf_prepare,
	.buf_queue	 = imx_vdev_buf_queue,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
	.start_streaming = imx_vdev_start_streaming,
	.stop_streaming  = imx_vdev_stop_streaming,
};

/*
 * File operations
 */
static int imx_vdev_open(struct file *file)
{
	struct imx_video_priv *priv = video_drvdata(file);
	struct video_device *vfd = &priv->vdev.vfd;
	int ret;

	if (mutex_lock_interruptible(&priv->mutex))
		return -ERESTARTSYS;

	ret = v4l2_fh_open(file);
	if (ret)
		v4l2_err(priv->sd, "v4l2_fh_open failed\n");

	ret = v4l2_pipeline_pm_use(&vfd->entity, 1);
	if (ret)
		v4l2_fh_release(file);

	mutex_unlock(&priv->mutex);
	return ret;
}

static int imx_vdev_release(struct file *file)
{
	struct imx_video_priv *priv = video_drvdata(file);
	struct video_device *vfd = &priv->vdev.vfd;
	struct vb2_queue *vq = &priv->q;

	mutex_lock(&priv->mutex);

	if (file->private_data == vq->owner) {
		vb2_queue_release(vq);
		vq->owner = NULL;
	}

	v4l2_pipeline_pm_use(&vfd->entity, 0);

	v4l2_fh_release(file);
	mutex_unlock(&priv->mutex);
	return 0;
}

static const struct v4l2_file_operations imx_vdev_fops = {
	.owner		= THIS_MODULE,
	.open		= imx_vdev_open,
	.release	= imx_vdev_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

struct imx_media_buffer *
imx_media_video_device_next_buf(struct imx_media_video_dev *vdev)
{
	struct imx_video_priv *priv = to_imx_video_priv(vdev);
	struct imx_media_buffer *buf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&priv->q_lock, flags);

	/* get next queued buffer */
	if (!list_empty(&priv->ready_q)) {
		buf = list_entry(priv->ready_q.next, struct imx_media_buffer,
				 list);
		list_del(&buf->list);
	}

	spin_unlock_irqrestore(&priv->q_lock, flags);

	return buf;
}
EXPORT_SYMBOL_GPL(imx_media_video_device_next_buf);

void imx_media_video_device_error(struct imx_media_video_dev *vdev)
{
	struct imx_video_priv *priv = to_imx_video_priv(vdev);
	struct vb2_queue *vq = &priv->q;
	unsigned long flags;

	if (!vb2_is_streaming(vq))
		return;

	spin_lock_irqsave(&priv->q_lock, flags);
	vb2_queue_error(vq);
	spin_unlock_irqrestore(&priv->q_lock, flags);
}
EXPORT_SYMBOL_GPL(imx_media_video_device_error);

int imx_media_video_device_register(struct imx_media_video_dev *vdev)
{
	struct imx_video_priv *priv = to_imx_video_priv(vdev);
	struct v4l2_subdev *sd = priv->sd;
	struct v4l2_device *v4l2_dev = sd->v4l2_dev;
	struct video_device *vfd = &vdev->vfd;
	struct vb2_queue *vq = &priv->q;
	struct v4l2_subdev_format sd_fmt;
	struct media_entity *source, *sink;
	u16 source_pad, sink_pad;
	int ret;

	/* get media device */
	priv->md = container_of(v4l2_dev->mdev, struct imx_media_dev, md);

	vfd->v4l2_dev = v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(sd, "Failed to register video device\n");
		return ret;
	}

	vq->type = priv->type;
	vq->io_modes = VB2_MMAP | VB2_DMABUF;
	vq->drv_priv = priv;
	vq->buf_struct_size = sizeof(struct imx_media_buffer);
	vq->ops = &imx_vdev_qops;
	vq->mem_ops = &vb2_dma_contig_memops;
	vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vq->lock = &priv->mutex;
	vq->min_buffers_needed = 2;
	vq->dev = priv->dev;

	ret = vb2_queue_init(vq);
	if (ret) {
		v4l2_err(sd, "vb2_queue_init failed\n");
		goto unreg;
	}

	INIT_LIST_HEAD(&priv->ready_q);

	if (priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		priv->vdev_pad.flags = MEDIA_PAD_FL_SINK;
		source = &sd->entity;
		source_pad = priv->sd_pad;
		sink = &vfd->entity;
		sink_pad = 0;
	} else {
		priv->vdev_pad.flags = MEDIA_PAD_FL_SOURCE;
		source = &vfd->entity;
		source_pad = 0;
		sink = &sd->entity;
		sink_pad = priv->sd_pad;
	}

	ret = media_entity_pads_init(&vfd->entity, 1, &priv->vdev_pad);
	if (ret) {
		v4l2_err(sd, "failed to init dev pad\n");
		goto unreg;
	}

	/* create the link between sd pad and video device pad */
	ret = media_create_pad_link(source, source_pad, sink, sink_pad, 0);
	if (ret) {
		v4l2_err(sd, "failed to create link to device node\n");
		goto unreg;
	}

	/* setup default format */
	sd_fmt.pad = priv->sd_pad;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	v4l2_subdev_call(sd, pad, get_fmt, NULL, &sd_fmt);
	if (ret) {
		v4l2_err(sd, "failed to get sd format\n");
		goto unreg;
	}

	vdev->fmt.type = priv->type;
	imx_media_mbus_fmt_to_pix_fmt(&vdev->fmt.fmt.pix,
				      &sd_fmt.format, NULL);

	if (vdev->fmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		vdev->compose.width = sd_fmt.format.width;
		vdev->compose.height = sd_fmt.format.height;
	}

	vdev->cc = imx_media_find_format(vdev->fmt.fmt.pix.pixelformat,
					 CS_SEL_ANY, false);

	v4l2_info(sd, "Registered %s as /dev/%s\n", vfd->name,
		  video_device_node_name(vfd));

	vfd->ctrl_handler = &priv->ctrl_hdlr;

	/* add vdev to the video device list */
	imx_media_add_video_device(priv->md, vdev);

	return 0;
unreg:
	video_unregister_device(vfd);
	return ret;
}
EXPORT_SYMBOL_GPL(imx_media_video_device_register);

void imx_media_video_device_unregister(struct imx_media_video_dev *vdev)
{
	struct imx_video_priv *priv = to_imx_video_priv(vdev);
	struct video_device *vfd = &priv->vdev.vfd;

	mutex_lock(&priv->mutex);

	if (video_is_registered(vfd)) {
		video_unregister_device(vfd);
		media_entity_cleanup(&vfd->entity);
	}

	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL_GPL(imx_media_video_device_unregister);

struct imx_media_video_dev *
imx_media_video_device_init(struct device *dev, struct v4l2_subdev *sd,
			    enum v4l2_buf_type type, int pad)
{
	struct imx_video_priv *priv;
	struct video_device *vfd;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->type = type;
	priv->sd = sd;
	priv->sd_pad = pad;
	priv->dev = dev;

	mutex_init(&priv->mutex);
	spin_lock_init(&priv->q_lock);

	vfd = &priv->vdev.vfd;
	snprintf(vfd->name, sizeof(vfd->name), "%s %s",
		 sd->name, priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
		 "capture" : "output");
	vfd->fops = &imx_vdev_fops;
	vfd->ioctl_ops = &imx_vdev_ioctl_ops;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	if (priv->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		vfd->vfl_dir = VFL_DIR_RX;
		vfd->device_caps = V4L2_CAP_VIDEO_CAPTURE;
	} else {
		vfd->vfl_dir = VFL_DIR_TX;
		vfd->device_caps = V4L2_CAP_VIDEO_OUTPUT;
	}
	vfd->device_caps |= V4L2_CAP_STREAMING;
	vfd->tvnorms = V4L2_STD_NTSC | V4L2_STD_PAL | V4L2_STD_SECAM;
	vfd->lock = &priv->mutex;
	vfd->queue = &priv->q;

	INIT_LIST_HEAD(&priv->vdev.list);

	video_set_drvdata(vfd, priv);

	v4l2_ctrl_handler_init(&priv->ctrl_hdlr, 0);

	return &priv->vdev;
}
EXPORT_SYMBOL_GPL(imx_media_video_device_init);

void imx_media_video_device_remove(struct imx_media_video_dev *vdev)
{
	struct imx_video_priv *priv = to_imx_video_priv(vdev);

	v4l2_ctrl_handler_free(&priv->ctrl_hdlr);
}
EXPORT_SYMBOL_GPL(imx_media_video_device_remove);

MODULE_DESCRIPTION("i.MX5/6 v4l2 video interface driver");
MODULE_AUTHOR("Steve Longerbeam <steve_longerbeam@mentor.com>");
MODULE_LICENSE("GPL");
