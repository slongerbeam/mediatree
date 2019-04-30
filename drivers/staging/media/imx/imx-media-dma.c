
struct imx_media_dma {
	struct ipu_soc *ipu;
	struct ipu_smfc *smfc;
	int smfc_id;
	struct ipuv3_channel *idmac_ch;

	struct imx_media_fim *fim;

	/* active vb2 buffers to send to video dev sink */
	struct imx_media_buffer *active_vb2_buf[2];
	struct imx_media_dma_buf underrun_buf;

	int ipu_buf_num;  /* ipu double buffer index: 0-1 */

	spinlock_t irqlock; /* protect eof_irq handler */
	struct timer_list eof_timeout_timer;
	int eof_irq;
	int nfb4eof_irq;

	u32 frame_sequence; /* frame sequence counter */
	bool last_eof;   /* waiting for last EOF at stream off */
	bool nfb4eof;    /* NFB4EOF encountered during streaming */
	bool interweave_swap; /* swap top/bottom lines when interweaving */
	struct completion last_eof_comp;
};

static void idmac_put_resources(struct imx_media_dma *priv)
{
	if (priv->idmac_ch)
		ipu_idmac_put(priv->idmac_ch);
	priv->idmac_ch = NULL;

	if (priv->smfc)
		ipu_smfc_put(priv->smfc);
	priv->smfc = NULL;
}

static int idmac_get_resources(struct imx_media_dma *priv)
{
	struct ipuv3_channel *idmac_ch;
	struct ipu_smfc *smfc;
	int ch_num, ret;

	priv->smfc_id = (priv->csi_id == 0) ? 0 : 2;

	ch_num = IPUV3_CHANNEL_CSI0 + priv->smfc_id;

	smfc = ipu_smfc_get(priv->ipu, ch_num);
	if (IS_ERR(smfc)) {
		v4l2_err(&priv->sd, "failed to get SMFC\n");
		ret = PTR_ERR(smfc);
		goto out;
	}
	priv->smfc = smfc;

	idmac_ch = ipu_idmac_get(priv->ipu, ch_num);
	if (IS_ERR(idmac_ch)) {
		v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
			 ch_num);
		ret = PTR_ERR(idmac_ch);
		goto out;
	}
	priv->idmac_ch = idmac_ch;

	return 0;
out:
	idmac_put_resources(priv);
	return ret;
}

static void csi_vb2_buf_done(struct csi_priv *priv)
{
	struct imx_media_video_dev *vdev = priv->vdev;
	struct imx_media_buffer *done, *next;
	struct vb2_buffer *vb;
	dma_addr_t phys;

	done = priv->active_vb2_buf[priv->ipu_buf_num];
	if (done) {
		done->vbuf.field = vdev->fmt.fmt.pix.field;
		done->vbuf.sequence = priv->frame_sequence;
		vb = &done->vbuf.vb2_buf;
		vb->timestamp = ktime_get_ns();
		vb2_buffer_done(vb, priv->nfb4eof ?
				VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	}

	priv->frame_sequence++;
	priv->nfb4eof = false;

	/* get next queued buffer */
	next = imx_media_capture_device_next_buf(vdev);
	if (next) {
		phys = vb2_dma_contig_plane_dma_addr(&next->vbuf.vb2_buf, 0);
		priv->active_vb2_buf[priv->ipu_buf_num] = next;
	} else {
		phys = priv->underrun_buf.phys;
		priv->active_vb2_buf[priv->ipu_buf_num] = NULL;
	}

	if (ipu_idmac_buffer_is_ready(priv->idmac_ch, priv->ipu_buf_num))
		ipu_idmac_clear_buffer(priv->idmac_ch, priv->ipu_buf_num);

	if (priv->interweave_swap)
		phys += vdev->fmt.fmt.pix.bytesperline;

	ipu_cpmem_set_buffer(priv->idmac_ch, priv->ipu_buf_num, phys);
}

static irqreturn_t csi_idmac_eof_interrupt(int irq, void *dev_id)
{
	struct csi_priv *priv = dev_id;

	spin_lock(&priv->irqlock);

	if (priv->last_eof) {
		complete(&priv->last_eof_comp);
		priv->last_eof = false;
		goto unlock;
	}

	if (priv->fim)
		/* call frame interval monitor */
		imx_media_fim_eof_monitor(priv->fim, ktime_get());

	csi_vb2_buf_done(priv);

	/* select new IPU buf */
	ipu_idmac_select_buffer(priv->idmac_ch, priv->ipu_buf_num);
	/* toggle IPU double-buffer index */
	priv->ipu_buf_num ^= 1;

	/* bump the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));

unlock:
	spin_unlock(&priv->irqlock);
	return IRQ_HANDLED;
}

static irqreturn_t csi_idmac_nfb4eof_interrupt(int irq, void *dev_id)
{
	struct csi_priv *priv = dev_id;

	spin_lock(&priv->irqlock);

	/*
	 * this is not an unrecoverable error, just mark
	 * the next captured frame with vb2 error flag.
	 */
	priv->nfb4eof = true;

	v4l2_err(&priv->sd, "NFB4EOF\n");

	spin_unlock(&priv->irqlock);

	return IRQ_HANDLED;
}

/*
 * EOF timeout timer function. This is an unrecoverable condition
 * without a stream restart.
 */
static void csi_idmac_eof_timeout(struct timer_list *t)
{
	struct csi_priv *priv = from_timer(priv, t, eof_timeout_timer);
	struct imx_media_video_dev *vdev = priv->vdev;

	v4l2_err(&priv->sd, "EOF timeout\n");

	/* signal a fatal error to capture device */
	imx_media_capture_device_error(vdev);
}

static void csi_idmac_setup_vb2_buf(struct csi_priv *priv, dma_addr_t *phys)
{
	struct imx_media_video_dev *vdev = priv->vdev;
	struct imx_media_buffer *buf;
	int i;

	for (i = 0; i < 2; i++) {
		buf = imx_media_capture_device_next_buf(vdev);
		if (buf) {
			priv->active_vb2_buf[i] = buf;
			phys[i] = vb2_dma_contig_plane_dma_addr(
				&buf->vbuf.vb2_buf, 0);
		} else {
			priv->active_vb2_buf[i] = NULL;
			phys[i] = priv->underrun_buf.phys;
		}
	}
}

static void csi_idmac_unsetup_vb2_buf(struct csi_priv *priv,
				      enum vb2_buffer_state return_status)
{
	struct imx_media_buffer *buf;
	int i;

	/* return any remaining active frames with return_status */
	for (i = 0; i < 2; i++) {
		buf = priv->active_vb2_buf[i];
		if (buf) {
			struct vb2_buffer *vb = &buf->vbuf.vb2_buf;

			vb->timestamp = ktime_get_ns();
			vb2_buffer_done(vb, return_status);
		}
	}
}

/* init the SMFC IDMAC channel */
static int csi_idmac_setup_channel(struct csi_priv *priv)
{
	struct imx_media_video_dev *vdev = priv->vdev;
	const struct imx_media_pixfmt *incc;
	struct v4l2_mbus_framefmt *infmt;
	struct v4l2_mbus_framefmt *outfmt;
	bool passthrough, interweave;
	struct ipu_image image;
	u32 passthrough_bits;
	u32 passthrough_cycles;
	dma_addr_t phys[2];
	u32 burst_size;
	int ret;

	infmt = &priv->format_mbus[CSI_SINK_PAD];
	incc = priv->cc[CSI_SINK_PAD];
	outfmt = &priv->format_mbus[CSI_SRC_PAD_IDMAC];

	ipu_cpmem_zero(priv->idmac_ch);

	memset(&image, 0, sizeof(image));
	image.pix = vdev->fmt.fmt.pix;
	image.rect = vdev->compose;

	csi_idmac_setup_vb2_buf(priv, phys);

	image.phys0 = phys[0];
	image.phys1 = phys[1];

	passthrough = requires_passthrough(&priv->upstream_ep, infmt, incc);
	passthrough_cycles = 1;

	/*
	 * If the field type at capture interface is interlaced, and
	 * the output IDMAC pad is sequential, enable interweave at
	 * the IDMAC output channel.
	 */
	interweave = V4L2_FIELD_IS_INTERLACED(image.pix.field) &&
		V4L2_FIELD_IS_SEQUENTIAL(outfmt->field);
	priv->interweave_swap = interweave &&
		image.pix.field == V4L2_FIELD_INTERLACED_BT;

	switch (image.pix.pixelformat) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_GREY:
		burst_size = 16;
		passthrough_bits = 8;
		break;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SGBRG16:
	case V4L2_PIX_FMT_SGRBG16:
	case V4L2_PIX_FMT_SRGGB16:
	case V4L2_PIX_FMT_Y16:
		burst_size = 8;
		passthrough_bits = 16;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
		burst_size = (image.pix.width & 0x3f) ?
			     ((image.pix.width & 0x1f) ?
			      ((image.pix.width & 0xf) ? 8 : 16) : 32) : 64;
		passthrough_bits = 16;
		/*
		 * Skip writing U and V components to odd rows (but not
		 * when enabling IDMAC interweaving, they are incompatible).
		 */
		if (!interweave)
			ipu_cpmem_skip_odd_chroma_rows(priv->idmac_ch);
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		burst_size = (image.pix.width & 0x1f) ?
			     ((image.pix.width & 0xf) ? 8 : 16) : 32;
		passthrough_bits = 16;
		break;
	case V4L2_PIX_FMT_RGB565:
		if (passthrough) {
			burst_size = 16;
			passthrough_bits = 8;
			passthrough_cycles = incc->cycles;
			break;
		}
		/* fallthrough - non-passthrough RGB565 (CSI-2 bus) */
	default:
		burst_size = (image.pix.width & 0xf) ? 8 : 16;
		passthrough_bits = 16;
		break;
	}

	if (passthrough) {
		if (priv->interweave_swap) {
			/* start interweave scan at 1st top line (2nd line) */
			image.phys0 += image.pix.bytesperline;
			image.phys1 += image.pix.bytesperline;
		}

		ipu_cpmem_set_resolution(priv->idmac_ch,
					 image.rect.width * passthrough_cycles,
					 image.rect.height);
		ipu_cpmem_set_stride(priv->idmac_ch, image.pix.bytesperline);
		ipu_cpmem_set_buffer(priv->idmac_ch, 0, image.phys0);
		ipu_cpmem_set_buffer(priv->idmac_ch, 1, image.phys1);
		ipu_cpmem_set_format_passthrough(priv->idmac_ch,
						 passthrough_bits);
	} else {
		if (priv->interweave_swap) {
			/* start interweave scan at 1st top line (2nd line) */
			image.rect.top = 1;
		}

		ret = ipu_cpmem_set_image(priv->idmac_ch, &image);
		if (ret)
			goto unsetup_vb2;
	}

	ipu_cpmem_set_burstsize(priv->idmac_ch, burst_size);

	/*
	 * Set the channel for the direct CSI-->memory via SMFC
	 * use-case to very high priority, by enabling the watermark
	 * signal in the SMFC, enabling WM in the channel, and setting
	 * the channel priority to high.
	 *
	 * Refer to the i.mx6 rev. D TRM Table 36-8: Calculated priority
	 * value.
	 *
	 * The WM's are set very low by intention here to ensure that
	 * the SMFC FIFOs do not overflow.
	 */
	ipu_smfc_set_watermark(priv->smfc, 0x02, 0x01);
	ipu_cpmem_set_high_priority(priv->idmac_ch);
	ipu_idmac_enable_watermark(priv->idmac_ch, true);
	ipu_cpmem_set_axi_id(priv->idmac_ch, 0);

	burst_size = passthrough ?
		(burst_size >> 3) - 1 : (burst_size >> 2) - 1;

	ipu_smfc_set_burstsize(priv->smfc, burst_size);

	if (interweave)
		ipu_cpmem_interlaced_scan(priv->idmac_ch,
					  priv->interweave_swap ?
					  -image.pix.bytesperline :
					  image.pix.bytesperline,
					  image.pix.pixelformat);

	ipu_idmac_set_double_buffer(priv->idmac_ch, true);

	return 0;

unsetup_vb2:
	csi_idmac_unsetup_vb2_buf(priv, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void csi_idmac_unsetup(struct csi_priv *priv,
			      enum vb2_buffer_state state)
{
	ipu_idmac_disable_channel(priv->idmac_ch);
	ipu_smfc_disable(priv->smfc);

	csi_idmac_unsetup_vb2_buf(priv, state);
}

static int csi_idmac_setup(struct csi_priv *priv)
{
	int ret;

	ret = csi_idmac_setup_channel(priv);
	if (ret)
		return ret;

	ipu_cpmem_dump(priv->idmac_ch);
	ipu_dump(priv->ipu);

	ipu_smfc_enable(priv->smfc);

	/* set buffers ready */
	ipu_idmac_select_buffer(priv->idmac_ch, 0);
	ipu_idmac_select_buffer(priv->idmac_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->idmac_ch);

	return 0;
}

static int csi_idmac_start(struct csi_priv *priv)
{
	struct imx_media_video_dev *vdev = priv->vdev;
	struct v4l2_pix_format *outfmt;
	int ret;

	ret = csi_idmac_get_ipu_resources(priv);
	if (ret)
		return ret;

	ipu_smfc_map_channel(priv->smfc, priv->csi_id, priv->vc_num);

	outfmt = &vdev->fmt.fmt.pix;

	ret = imx_media_alloc_dma_buf(priv->dev, &priv->underrun_buf,
				      outfmt->sizeimage);
	if (ret)
		goto out_put_ipu;

	priv->ipu_buf_num = 0;

	/* init EOF completion waitq */
	init_completion(&priv->last_eof_comp);
	priv->frame_sequence = 0;
	priv->last_eof = false;
	priv->nfb4eof = false;

	ret = csi_idmac_setup(priv);
	if (ret) {
		v4l2_err(&priv->sd, "csi_idmac_setup failed: %d\n", ret);
		goto out_free_dma_buf;
	}

	priv->nfb4eof_irq = ipu_idmac_channel_irq(priv->ipu,
						 priv->idmac_ch,
						 IPU_IRQ_NFB4EOF);
	ret = devm_request_irq(priv->dev, priv->nfb4eof_irq,
			       csi_idmac_nfb4eof_interrupt, 0,
			       "imx-smfc-nfb4eof", priv);
	if (ret) {
		v4l2_err(&priv->sd,
			 "Error registering NFB4EOF irq: %d\n", ret);
		goto out_unsetup;
	}

	priv->eof_irq = ipu_idmac_channel_irq(priv->ipu, priv->idmac_ch,
					      IPU_IRQ_EOF);

	ret = devm_request_irq(priv->dev, priv->eof_irq,
			       csi_idmac_eof_interrupt, 0,
			       "imx-smfc-eof", priv);
	if (ret) {
		v4l2_err(&priv->sd,
			 "Error registering eof irq: %d\n", ret);
		goto out_free_nfb4eof_irq;
	}

	/* start the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));

	return 0;

out_free_nfb4eof_irq:
	devm_free_irq(priv->dev, priv->nfb4eof_irq, priv);
out_unsetup:
	csi_idmac_unsetup(priv, VB2_BUF_STATE_QUEUED);
out_free_dma_buf:
	imx_media_free_dma_buf(priv->dev, &priv->underrun_buf);
out_put_ipu:
	csi_idmac_put_ipu_resources(priv);
	return ret;
}

static void csi_idmac_wait_last_eof(struct csi_priv *priv)
{
	unsigned long flags;
	int ret;

	/* mark next EOF interrupt as the last before stream off */
	spin_lock_irqsave(&priv->irqlock, flags);
	priv->last_eof = true;
	spin_unlock_irqrestore(&priv->irqlock, flags);

	/*
	 * and then wait for interrupt handler to mark completion.
	 */
	ret = wait_for_completion_timeout(
		&priv->last_eof_comp, msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));
	if (ret == 0)
		v4l2_warn(&priv->sd, "wait last EOF timeout\n");
}

static void csi_idmac_stop(struct csi_priv *priv)
{
	devm_free_irq(priv->dev, priv->eof_irq, priv);
	devm_free_irq(priv->dev, priv->nfb4eof_irq, priv);

	csi_idmac_unsetup(priv, VB2_BUF_STATE_ERROR);

	imx_media_free_dma_buf(priv->dev, &priv->underrun_buf);

	/* cancel the EOF timeout timer */
	del_timer_sync(&priv->eof_timeout_timer);

	csi_idmac_put_ipu_resources(priv);
}

static int csi_start(struct csi_priv *priv)
{
	int ret;

	/* FIXME: upstream must be started here */

	/* start the frame interval monitor */
	if (priv->fim) {
		struct v4l2_fract *output_fi =
			&priv->frame_interval[priv->active_output_pad];

		ret = imx_media_fim_set_stream(priv->fim, output_fi, true);
		if (ret)
			return ret;
	}

	ret = csi_idmac_start(priv);
	if (ret)
		goto fim_stop;

	return 0;

fim_stop:
	if (priv->fim)
		imx_media_fim_set_stream(priv->fim, NULL, false);
idmac_stop:
	csi_idmac_stop(priv);
}

static void csi_stop(struct csi_priv *priv)
{
	csi_idmac_wait_last_eof(priv);

	// FIXME: upstream must be stopped here!

	csi_idmac_stop(priv);

	/* stop the frame interval monitor */
	if (priv->fim)
		imx_media_fim_set_stream(priv->fim, NULL, false);
}

// FIXME: FIM stuff, not sure where this goes yet.
static int csi_init(struct csi_priv *priv)
{
	priv->fim = imx_media_fim_init(&priv->sd);
	if (IS_ERR(priv->fim)) {
		ret = PTR_ERR(priv->fim);
		goto put_csi;
	}

	if (priv->fim) {
		ret = imx_media_fim_add_controls(priv->fim);
		if (ret)
			goto free_fim;
	}

	return 0;

free_fim:
	if (priv->fim)
		imx_media_fim_free(priv->fim);
	return ret;
}

static void csi_remove(struct csi_priv *priv)
{
	if (priv->fim)
		imx_media_fim_free(priv->fim);
}

