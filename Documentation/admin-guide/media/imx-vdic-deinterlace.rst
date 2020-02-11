.. SPDX-License-Identifier: GPL-2.0

i.MX VDIC De-Interlace Driver
=============================

Introduction
------------

This is a mem2mem driver that performs hardware motion-compensated
de-interlacing using the Video De-Interlacing or Combining Block (VDIC).

The VDIC supports three different motion compensation modes: low,
medium, and high motion. This setting is a v4l2 control in the VDIC
sub-device.

The mem2mem driver accepts sequential field (seq-bt or seq-tb) or
individual field (altrenate) buffers at its output side, and delivers
motion-compensated de-interlaced frames at its capture side.

The mem2mem driver forwards the de-interlaced output frames from the
VDIC to the IC pre-process viewfinder task. Thus in addition to
de-interacing, the driver supports the IC functions of non-tiled scaling
(up to 1024x1024), CSC, and image rotation.

On i.MX5 and i.MX6DL (with a single IPU), there is a single mem2mem
device connected as a source to ipu1_vdic and as a sink from
ipu1_ic_prpvf.

On quad i.MX6 (two IPUs), there are two mem2mem devices, one connected
as above, and another connected to ipu2_vdic and ipu2_ic_prpvf.

Refer to imx.rst for more info on the ipuX_vdic and ipuX_ic_prpvf
sub-devices and the de-interlacing mode v4l2 control.

Configuring the VDIC Mem2mem Pipeline
-------------------------------------

This configures the mem2mem pipeline on IPU1:

.. code-block:: none

   media-ctl -l "'ipu1_vdic mem2mem-source':0 -> 'ipu1_vdic':1[1]"
   media-ctl -l "'ipu1_vdic':2 -> 'ipu1_ic_prp':0[1]"
   media-ctl -l "'ipu1_ic_prp':2 -> 'ipu1_ic_prpvf':0[1]"
   media-ctl -l "'ipu1_ic_prpvf':2 -> 'ipu1_vdic mem2mem-sink':0[1]"

This example configures the pad formats assuming the mem2mem device
receives alternate 720x240 fields, and scales the de-interlaced
720x480 frames up to 1024x768 with color-conversion to RGB:

.. code-block:: none

   media-ctl -V "'ipu1_vdic':1 [fmt:AYUV32/720x240 field:alternate]"
   media-ctl -V "'ipu1_vdic':2 [fmt:AYUV32/720x480 field:none]"
   media-ctl -V "'ipu1_ic_prp':2 [fmt:AYUV32/720x480 field:none]"
   media-ctl -V "'ipu1_ic_prpvf':2 [fmt:ARGB8888_1X32/1024x768 field:none]"

This example is similar, except that the mem2mem device receives seq-bt
frames without color-conversion :

.. code-block:: none

   # Configure pad formats
   media-ctl -V "'ipu1_vdic':1 [fmt:AYUV32/720x480 field:seq-bt]"
   media-ctl -V "'ipu1_vdic':2 [fmt:AYUV32/720x480 field:none]"
   media-ctl -V "'ipu1_ic_prp':2 [fmt:AYUV32/720x480 field:none]"
   media-ctl -V "'ipu1_ic_prpvf':2 [fmt:AYUV32/1024x768 field:none]"

A seq-bt source can then be fed to the mem2mem device. One way to create
such a source is the following example that configures a capture
pipeline from the ADV7180 on the i.MX6Q SabreAuto. The ADV7180 outputs
alternate fields, which the ipu1_csi0 sub-device combines to seq-bt at
its output:

.. code-block:: none

   # Setup links
   media-ctl -l "'adv7180 3-0021':0 -> 'ipu1_csi0_mux':1[1]"
   media-ctl -l "'ipu1_csi0_mux':2 -> 'ipu1_csi0':0[1]"
   media-ctl -l "'ipu1_csi0':2 -> 'ipu1_csi0 capture':0[1]"
   # Configure pad formats
   media-ctl -V "'adv7180 3-0021':0 [fmt:UYVY2X8/720x240 field:alternate]"
   media-ctl -V "'ipu1_csi0_mux':2 [fmt:UYVY2X8/720x240]"
   media-ctl -V "'ipu1_csi0':2 [fmt:AYUV32/720x480]"
   # Configure "ipu1_csi0 capture" interface (assumed at /dev/video0)
   v4l2-ctl -d0 --set-fmt-video=pixelformat=UYVY,field=seq-bt

The seq-bt, 720x480, UYVY buffers from /dev/video0 can then be fed into
the mem2mem device (here assumed at /dev/video12) using the following
gstreamer pipeline:

.. code-block:: none

   gst-launch-1.0 v4l2src device=/dev/video0 ! \
	"video/x-raw,format=UYVY,width=720,height=480,interlace-mode=seq-bt" ! \
	v4l2video12convert output-io-mode=dmabuf-import ! \
	"video/x-raw,format=UYVY,width=1024,height=768,interlace-mode=progressive" ! \
	kmssink connector-id=54 name=imx-drm sync=0 can-scale=false
