// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * i.MX VDIC mem2mem de-interlace driver
 *
 * Copyright (c) 2018 Mentor Graphics Inc.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <media/media-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "imx-media.h"

#define fh_to_ctx(__fh)	container_of(__fh, struct mem2mem_ctx, fh)

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST  = 1,
};

struct mem2mem_ctx;

struct mem2mem_priv {
	struct imx_media_video_dev vdev;
	struct imx_media_dev  *md;

	struct v4l2_subdev    *vdic;
	int                   vdic_sink_pad;
	int                   vdic_src_pad;

	struct v4l2_m2m_dev   *m2m_dev;
	struct device         *dev;

	struct mutex          mutex;       /* mem2mem device mutex */

	struct v4l2_pix_format fmt[2];
	const struct imx_media_pixfmt *cc[2];

	atomic_t              num_inst;
};

#define to_mem2mem_priv(v) container_of(v, struct mem2mem_priv, vdev)

struct mem2mem_ctx {
	struct mem2mem_priv	*priv;
	struct v4l2_fh		fh;
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
			   struct v4l2_rect *unused,
			   const struct imx_media_pixfmt **cc)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);
	struct v4l2_pix_format *fmt;

	fmt = get_format(priv, f->type);
	f->fmt.pix = *fmt;
	if (cc)
		*cc = priv->cc[f->type];

	return 0;
}

static struct vb2_v4l2_buffer *
mem2mem_get_next_buf(struct imx_media_video_dev *vdev, void *run_ctx,
		     enum v4l2_buf_type type)
{
	struct mem2mem_ctx *ctx = run_ctx;
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

static void mem2mem_buf_done(struct imx_media_video_dev *vdev, void *run_ctx,
			     struct vb2_v4l2_buffer *done,
			     enum vb2_buffer_state status)
{
	struct mem2mem_ctx *ctx = run_ctx;
	struct mem2mem_priv *priv = ctx->priv;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;

	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	dev_dbg(priv->dev, "%s: srcbuf %p, dstbuf %p\n", __func__,
		src_buf, dst_buf);

	v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, true);

	v4l2_m2m_buf_done(src_buf, status);
	v4l2_m2m_buf_done(dst_buf, status);

	v4l2_m2m_job_finish(priv->m2m_dev, ctx->fh.m2m_ctx);
}

static void mem2mem_device_error(struct imx_media_video_dev *vdev,
				 void *run_ctx)
{
	struct mem2mem_ctx *ctx = run_ctx;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	if (!vb2_is_streaming(vq))
		return;

	vb2_queue_error(vq);
}

static const struct imx_media_video_dev_ops mem2mem_ops = {
	.get_fmt = mem2mem_get_fmt,
	.get_next_buf = mem2mem_get_next_buf,
	.buf_done = mem2mem_buf_done,
	.device_error = mem2mem_device_error,
};

/*
 * mem2mem callbacks
 */

static void device_run(void *_ctx)
{
	struct mem2mem_ctx *ctx = _ctx;
	struct mem2mem_priv *priv = ctx->priv;

	v4l2_subdev_call(priv->vdic, proc, device_run, ctx);
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
	u32 fourcc;
	int ret;

	ret = imx_media_enum_format(&fourcc, f->index, CS_SEL_YUV);
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
	const struct imx_media_pixfmt *cc;
	struct v4l2_subdev_format fmt;
	u32 fourcc;
	int ret;

	fmt.pad = (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
		priv->vdic_src_pad : priv->vdic_sink_pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(priv->vdic, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	fourcc = f->fmt.pix.pixelformat;
	cc = imx_media_find_format(fourcc, CS_SEL_YUV, false);
	if (!cc) {
		imx_media_enum_format(&fourcc, 0, CS_SEL_YUV);
		cc = imx_media_find_format(fourcc, CS_SEL_YUV, false);
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
		v4l2_err(priv->vdic, "%s queue busy\n",  __func__);
		return -EBUSY;
	}

	ret = mem2mem_try_fmt(file, _priv, f);
	if (ret < 0)
		return ret;

	fmt = get_format(priv, f->type);
	*fmt = f->fmt.pix;
	priv->cc[f->type] = imx_media_find_format(f->fmt.pix.pixelformat,
						  CS_SEL_YUV, false);

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
	unsigned int count = *nbuffers;
	struct v4l2_pix_format *fmt;

	fmt = get_format(priv, vq->type);

	*nplanes = 1;
	*nbuffers = count;
	sizes[0] = fmt->sizeimage;

	dev_dbg(ctx->priv->dev, "get %d buffer(s) of size %d each.\n",
		count, fmt->sizeimage);

	return 0;
}

static int mem2mem_buf_prepare(struct vb2_buffer *vb)
{
	struct mem2mem_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mem2mem_priv *priv = ctx->priv;
	unsigned int plane_size, payload;
	struct v4l2_pix_format *fmt;

	dev_dbg(ctx->priv->dev, "type: %d\n", vb->vb2_queue->type);

	fmt = get_format(priv, vb->vb2_queue->type);
	plane_size = fmt->sizeimage;

	if (vb2_plane_size(vb, 0) < plane_size) {
		dev_dbg(ctx->priv->dev,
			"%s data will not fit into plane (%lu < %lu)\n",
			__func__, vb2_plane_size(vb, 0), (long)plane_size);
		return -EINVAL;
	}

	payload = fmt->bytesperline * fmt->height;
	if (fmt->pixelformat == V4L2_PIX_FMT_YUV420 ||
	    fmt->pixelformat == V4L2_PIX_FMT_YVU420 ||
	    fmt->pixelformat == V4L2_PIX_FMT_NV12)
		payload = payload * 3 / 2;
	else if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P ||
		 fmt->pixelformat == V4L2_PIX_FMT_NV16)
		payload *= 2;

	vb2_set_plane_payload(vb, 0, payload);

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

	other_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				  (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
				  V4L2_BUF_TYPE_VIDEO_OUTPUT :
				  V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (!vb2_is_streaming(other_q))
		return 0;

	ret = imx_media_pipeline_set_stream(priv->md, &priv->vdic->entity,
					    true);
	if (ret) {
		v4l2_err(priv->vdic, "pipeline start failed with %d\n", ret);
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
	int ret;

	ret = imx_media_pipeline_set_stream(priv->md, &priv->vdic->entity,
					    false);
	if (ret)
		v4l2_warn(priv->vdic, "pipeline stop failed with %d\n", ret);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
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

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(priv->m2m_dev, ctx,
					    &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctx;
	}

	atomic_inc(&priv->num_inst);

	dev_dbg(priv->dev, "Created instance %p, m2m_ctx: %p\n", ctx,
		ctx->fh.m2m_ctx);

	return 0;

err_ctx:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int mem2mem_release(struct file *file)
{
	struct mem2mem_priv *priv = video_drvdata(file);
	struct mem2mem_ctx *ctx = fh_to_ctx(file->private_data);

	dev_dbg(priv->dev, "Releasing instance %p\n", ctx);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	atomic_dec(&priv->num_inst);

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
	.device_run	= device_run,
};

static const struct video_device mem2mem_template = {
	.fops		= &mem2mem_fops,
	.ioctl_ops	= &mem2mem_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.vfl_dir	= VFL_DIR_M2M,
	.tvnorms	= V4L2_STD_NTSC | V4L2_STD_PAL | V4L2_STD_SECAM,
	.device_caps	= V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
};

int imx_media_mem2mem_vdic_register(struct imx_media_video_dev *vdev)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);
	struct media_pad *vdic_sink, *vdic_src;
	struct video_device *vfd = vdev->vfd;
	int ret;

	priv->md = container_of(priv->vdic->v4l2_dev->mdev,
				struct imx_media_dev, md);

	vfd->v4l2_dev = &priv->md->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(priv->vdic, "Failed to register video device\n");
		return ret;
	}

	vdic_sink = &priv->vdic->entity.pads[priv->vdic_sink_pad];
	vdic_src = &priv->vdic->entity.pads[priv->vdic_src_pad];

	ret = __v4l2_m2m_register_media_controller(priv->m2m_dev, vfd,
						   vdic_sink, vdic_src,
						   0, 0);
	if (ret) {
		v4l2_err(priv->vdic,
			 "Failed to register with media controller\n");
		return ret;
	}

	v4l2_info(priv->vdic, "Registered %s as /dev/%s\n", vfd->name,
		  video_device_node_name(vfd));

	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_mem2mem_vdic_register);

void imx_media_mem2mem_vdic_unregister(struct imx_media_video_dev *vdev)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);
	struct video_device *vfd = priv->vdev.vfd;

	mutex_lock(&priv->mutex);

	if (video_is_registered(vfd)) {
		v4l2_m2m_unregister_media_controller(priv->m2m_dev);
		video_unregister_device(vfd);
		media_entity_cleanup(&vfd->entity);
	}

	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL_GPL(imx_media_mem2mem_vdic_unregister);

struct imx_media_video_dev *
imx_media_mem2mem_vdic_init(struct device *dev, struct v4l2_subdev *vdic,
			    int vdic_sink_pad, int vdic_src_pad)
{
	struct mem2mem_priv *priv;
	struct video_device *vfd;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->dev = dev;

	mutex_init(&priv->mutex);
	atomic_set(&priv->num_inst, 0);

	vfd = video_device_alloc();
	if (!vfd)
		return ERR_PTR(-ENOMEM);

	priv->vdic = vdic;
	priv->vdic_sink_pad = vdic_sink_pad;
	priv->vdic_src_pad = vdic_src_pad;
	*vfd = mem2mem_template;
	snprintf(vfd->name, sizeof(vfd->name), "%s mem2mem", vdic->name);
	vfd->lock = &priv->mutex;
	priv->vdev.vfd = vfd;
	priv->vdev.ops = &mem2mem_ops;

	INIT_LIST_HEAD(&priv->vdev.list);

	video_set_drvdata(vfd, priv);

	priv->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(priv->m2m_dev)) {
		ret = PTR_ERR(priv->m2m_dev);
		dev_err(priv->dev, "Failed to init mem2mem device: %d\n",
			ret);
		return ERR_PTR(ret);
	}

	return &priv->vdev;
}
EXPORT_SYMBOL_GPL(imx_media_mem2mem_vdic_init);

void imx_media_mem2mem_vdic_remove(struct imx_media_video_dev *vdev)
{
	struct mem2mem_priv *priv = to_mem2mem_priv(vdev);

	v4l2_m2m_release(priv->m2m_dev);
}
EXPORT_SYMBOL_GPL(imx_media_mem2mem_vdic_remove);

MODULE_DESCRIPTION("i.MX VDIC mem2mem de-interlace driver");
MODULE_AUTHOR("Steve Longerbeam <steve_longerbeam@mentor.com>");
MODULE_LICENSE("GPL");
