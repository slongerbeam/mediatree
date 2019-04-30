#include "imx-media.h"

struct capture_priv {
	struct imx_media_video_dev vdev;

	struct v4l2_format    fmt;     /* the user format */
	const struct imx_media_pixfmt *cc;

	struct v4l2_rect      compose; /* the compose rectangle */

	struct v4l2_subdev    *src_sd;
	int                   src_sd_pad;
	struct device         *dev;

	struct imx_media_dev  *md;

	struct media_pad      vdev_pad;

	struct mutex          mutex;       /* capture device mutex */

	/* the videobuf2 queue */
	struct vb2_queue       q;
	/* list of ready imx_media_buffer's from q */
	struct list_head       ready_q;
	/* protect ready_q */
	spinlock_t             q_lock;

	/* controls inherited from subdevs */
	struct v4l2_ctrl_handler ctrl_hdlr;

	/* misc status */
	bool                  stop;          /* streaming is stopping */
};

#define to_capture_priv(v) container_of(v, struct capture_priv, vdev)
