// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * S5P6818 CSI-2 receiver + VIP0 clipper.
 * Register sequences derived from Nexell's GPL nx-csi/nx-vip-primitive
 * drivers (Copyright 2016 Nexell, Sungwoo Park). The CSI channel-1 output
 * carries sensor virtual channel 0 to VIP0; buffers use linear 32-bit DMA.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define CSI_CTRL        0x00
#define CSI_DPHY        0x04
#define CSI_CONFIG0     0x08
#define CSI_INTMASK     0x10
#define CSI_INTSRC      0x14
#define CSI_CTRL2       0x18
#define CSI_DPHY1       0x24
#define CSI_CONFIG1     0x40
#define CSI_RESOL1      0x44
/* Shared CSI/DSI PLL; exclusive MMIO reservation excludes native DSI. */
#define MIPI_PLL        0x14c
#define MIPI_PLLTMR     0x150
#define MIPI_PHY        0x154
#define MIPI_PHY1       0x158
#define VIP_CONFIG      0x000
#define VIP_HVINT       0x004
#define VIP_SYNC        0x008
#define VIP_VBEGIN      0x010
#define VIP_VEND        0x014
#define VIP_HBEGIN      0x018
#define VIP_HEND        0x01c
#define VIP_FIFO        0x020
#define VIP_ENABLE      0x200
#define VIP_ODINT       0x204
#define VIP_WIDTH       0x208
#define VIP_HEIGHT      0x20c
#define VIP_LEFT        0x210
#define VIP_RIGHT       0x214
#define VIP_TOP         0x218
#define VIP_BOTTOM      0x21c
#define VIP_FORMAT      0x288
#define VIP_ADDR        0x28c
#define VIP_STRIDE      0x290
#define VIP_SCAN        0x2c0
#define NX_DMA_TAIL_BYTES 64

#define VIP_PORT        0x2d8

struct nx_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct nx_capture {
	struct device *dev;
	void __iomem *csi, *vip, *sram;
	struct clk_bulk_data clocks[3];
	struct reset_control_bulk_data resets[6];
	int irq;
	struct media_device media;
	struct v4l2_device v4l2;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *sensor;
	unsigned int sensor_pad;
	struct video_device video;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex mutex;
	spinlock_t lock;
	struct list_head buffers;
	struct completion stop_done;
	bool stopping;
	bool clocks_enabled;
	struct nx_buffer *active;
	struct v4l2_pix_format format;
	void *scratch;
	dma_addr_t scratch_dma;
	unsigned int sequence;
	bool streaming, powered, video_registered, ready, irq_enabled;
};

static void nx_dma_target(struct nx_capture *cap)
{
	dma_addr_t addr = cap->scratch_dma;

	cap->active = NULL;
	if (!list_empty(&cap->buffers)) {
		cap->active = list_first_entry(&cap->buffers, struct nx_buffer, list);
		list_del(&cap->active->list);
		addr = vb2_dma_contig_plane_dma_addr(&cap->active->vb.vb2_buf, 0);
	}
	writel(lower_32_bits(addr), cap->vip + VIP_ADDR);
}

static irqreturn_t nx_irq(int irq, void *data)
{
	struct nx_capture *cap = data;
	struct nx_buffer *done;
	unsigned long flags;
	u32 status;

	/* Never access peripheral registers while the capture clocks are off. */
	spin_lock_irqsave(&cap->lock, flags);
	if (!cap->powered) {
		spin_unlock_irqrestore(&cap->lock, flags);
		return IRQ_NONE;
	}
	status = readl(cap->vip + VIP_ODINT);
	if (!(status & BIT(0))) {
		spin_unlock_irqrestore(&cap->lock, flags);
		return IRQ_NONE;
	}
	writel((status & BIT(8)) | BIT(0), cap->vip + VIP_ODINT);
	if (cap->streaming) {
		if (cap->stopping) {
			/* Vendor stop is committed on the next frame-complete IRQ. */
			cap->streaming = false;
			writel(BIT(0), cap->vip + VIP_ODINT);
			writel(readl(cap->vip + VIP_CONFIG) & ~BIT(0),
			       cap->vip + VIP_CONFIG);
			writel(0, cap->vip + VIP_ENABLE);
			/* Complete VIP stop writes before waking the stream-off waiter. */
			wmb();
			complete(&cap->stop_done);
			spin_unlock_irqrestore(&cap->lock, flags);
			return IRQ_HANDLED;
		}
		/* Gate the clipper only when changing its DMA target. */
		writel(BIT(8), cap->vip + VIP_ENABLE);
		done = cap->active;
		nx_dma_target(cap);
		writel(BIT(8) | BIT(1), cap->vip + VIP_ENABLE);
		if (done) {
			done->vb.sequence = cap->sequence;
			done->vb.field = V4L2_FIELD_NONE;
			done->vb.vb2_buf.timestamp = ktime_get_ns();
			vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
		cap->sequence++;
	}
	spin_unlock_irqrestore(&cap->lock, flags);
	return IRQ_HANDLED;
}

static void nx_return_buffers(struct nx_capture *cap, enum vb2_buffer_state state)
{
	struct nx_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&cap->lock, flags);
	if (cap->active) {
		vb2_buffer_done(&cap->active->vb.vb2_buf, state);
		cap->active = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &cap->buffers, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&cap->lock, flags);
}

static int nx_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			  unsigned int *nplanes, unsigned int sizes[],
			  struct device *alloc_devs[])
{
	struct nx_capture *cap = vb2_get_drv_priv(q);

	if (*nplanes)
		return *nplanes == 1 && sizes[0] >= cap->format.sizeimage ? 0 : -EINVAL;
	*nplanes = 1;
	sizes[0] = cap->format.sizeimage;
	return 0;
}

static int nx_buffer_prepare(struct vb2_buffer *vb)
{
	struct nx_capture *cap = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < cap->format.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, cap->format.bytesperline * cap->format.height);
	return 0;
}

static void nx_buffer_queue(struct vb2_buffer *vb)
{
	struct nx_capture *cap = vb2_get_drv_priv(vb->vb2_queue);
	struct nx_buffer *buf = container_of(to_vb2_v4l2_buffer(vb), struct nx_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&cap->lock, flags);
	list_add_tail(&buf->list, &cap->buffers);
	spin_unlock_irqrestore(&cap->lock, flags);
}

static int nx_hw_start(struct nx_capture *cap)
{
	unsigned int w = cap->format.width, h = cap->format.height;
	u32 value;
	int ret;

	if (!cap->clocks_enabled) {
		ret = clk_bulk_prepare_enable(ARRAY_SIZE(cap->clocks), cap->clocks);
		if (ret)
			return ret;
		cap->clocks_enabled = true;
	}
	ret = reset_control_bulk_assert(ARRAY_SIZE(cap->resets), cap->resets);
	if (ret)
		goto clocks_off;
	usleep_range(10, 20);
	ret = reset_control_bulk_deassert(ARRAY_SIZE(cap->resets), cap->resets);
	if (ret)
		goto clocks_off;

	/* MIPI dual-port SRAM EMA[2:0], vendor silicon setting 3. */
	value = readl(cap->sram);
	writel((value & ~0xeU) | (3 << 1), cap->sram);

	/* CSI output channel 1 / sensor VC0, YUV422 8-bit, full 16-bit bus. */
	writel(0, cap->csi + CSI_INTMASK);
	writel(~0U, cap->csi + CSI_INTSRC);
	writel(0, cap->csi + CSI_DPHY1);
	writel((22 << 24) | 7, cap->csi + CSI_DPHY);
	writel(1, cap->csi + CSI_CONFIG0);
	writel((14 << 26) | (31 << 20) | (368 << 8) | (0x1e << 2),
	       cap->csi + CSI_CONFIG1);
	writel((w << 16) | h, cap->csi + CSI_RESOL1);
	writel(BIT(25), cap->csi + CSI_CTRL2);
	writel((2 << 22) | BIT(9) | BIT(2) | (15 << 16) | (15 << 12) | BIT(0),
	       cap->csi + CSI_CTRL);
	/* Preserve a PLL already running; no runtime writes to DSI video. */
	value = readl(cap->csi + MIPI_PLL);
	if (!(value & BIT(23))) {
		writel(0, cap->csi + MIPI_PHY);
		writel(0xffffffff, cap->csi + MIPI_PLLTMR);
		writel(0, cap->csi + MIPI_PHY1);
		writel((value & ~0x0fffffff) | (0xc << 24) | BIT(23) | (0x43e8 << 1),
		       cap->csi + MIPI_PLL);
	}

	if (!(readl(cap->csi + MIPI_PLL) & BIT(23))) {
		ret = -EIO;
		goto clocks_off;
	}

	/* Vendor 6818 external 16-bit MIPI sync, generated CSI porches. */
	writel(BIT(8), cap->vip + VIP_CONFIG); /* UYVY input order 0 */
	writel(BIT(12) | BIT(11) | BIT(10) | BIT(4) | BIT(2), cap->vip + VIP_SYNC);
	writel(0, cap->vip + VIP_VBEGIN);
	writel(0, cap->vip + VIP_VEND);
	writel(16, cap->vip + VIP_HBEGIN);
	writel(32, cap->vip + VIP_HEND);
	writel(w, cap->vip + VIP_WIDTH);
	writel(h, cap->vip + VIP_HEIGHT);
	writel(6, cap->vip + VIP_FIFO);
	writel(0, cap->vip + VIP_SCAN);
	writel(1, cap->vip + VIP_PORT); /* CSI is wired to VIP port B. */
	writel(0, cap->vip + VIP_LEFT);
	writel(w, cap->vip + VIP_RIGHT);
	writel(0, cap->vip + VIP_TOP);
	writel(h, cap->vip + VIP_BOTTOM);
	writel(3, cap->vip + VIP_FORMAT); /* packed YUYV */
	writel(cap->format.bytesperline, cap->vip + VIP_STRIDE);
	writel(0, cap->vip + VIP_HVINT);
	writel(BIT(0), cap->vip + VIP_ODINT);
	return 0;
clocks_off:
	clk_bulk_disable_unprepare(ARRAY_SIZE(cap->clocks), cap->clocks);
	cap->clocks_enabled = false;
	return ret;
}

static void nx_irq_disable(struct nx_capture *cap)
{
	if (cap->irq_enabled) {
		disable_irq(cap->irq);
		cap->irq_enabled = false;
	}
}

static void nx_hw_stop(struct nx_capture *cap)
{
	unsigned long flags;

	nx_irq_disable(cap);
	spin_lock_irqsave(&cap->lock, flags);
	cap->streaming = false;
	writel(BIT(0), cap->vip + VIP_ODINT);
	writel(readl(cap->vip + VIP_CONFIG) & ~BIT(0), cap->vip + VIP_CONFIG);
	writel(0, cap->vip + VIP_ENABLE);
	writel(0, cap->csi + CSI_INTMASK);
	writel(readl(cap->csi + CSI_CTRL) & ~BIT(0), cap->csi + CSI_CTRL);
	writel(readl(cap->csi + CSI_DPHY) & ~0x1f, cap->csi + CSI_DPHY);
	/* Complete receiver stop writes before its clocks can be gated. */
	wmb();
	cap->powered = false;
	spin_unlock_irqrestore(&cap->lock, flags);
	synchronize_irq(cap->irq);
	/* S5P6818 vendor stop retains the capture clocks and does not reset VIP. */
}

static int nx_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct nx_capture *cap = vb2_get_drv_priv(q);
	struct v4l2_subdev_format sensor_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = cap->sensor_pad,
	};
	unsigned long flags;
	int ret;

	if (!cap->sensor || !cap->ready) {
		ret = -ENODEV;
		goto return_buffers;
	}
	ret = v4l2_subdev_call_state_active(cap->sensor, pad, get_fmt, &sensor_fmt);
	if (ret)
		goto return_buffers;
	if (sensor_fmt.format.width != cap->format.width ||
	    sensor_fmt.format.height != cap->format.height ||
	    sensor_fmt.format.code != MEDIA_BUS_FMT_UYVY8_1X16) {
		ret = -EPIPE;
		goto return_buffers;
	}
	cap->scratch = dma_alloc_coherent(cap->dev, cap->format.sizeimage,
					&cap->scratch_dma, GFP_KERNEL);
	if (!cap->scratch) {
		ret = -ENOMEM;
		goto return_buffers;
	}
	ret = video_device_pipeline_start(&cap->video, &cap->video.pipe);
	if (ret)
		goto free_scratch;
	ret = nx_hw_start(cap);
	if (ret)
		goto pipeline_stop;
	spin_lock_irqsave(&cap->lock, flags);
	cap->powered = true;
	cap->sequence = 0;
	nx_dma_target(cap);
	cap->stopping = false;
	cap->streaming = true;
	writel(BIT(8) | BIT(0), cap->vip + VIP_ODINT);
	writel(BIT(8) | BIT(1), cap->vip + VIP_ENABLE);
	writel(readl(cap->vip + VIP_CONFIG) | BIT(0), cap->vip + VIP_CONFIG);
	spin_unlock_irqrestore(&cap->lock, flags);
	cap->irq_enabled = true;
	enable_irq(cap->irq);
	ret = v4l2_subdev_call(cap->sensor, video, s_stream, 1);
	if (!ret)
		return 0;
	nx_hw_stop(cap);
pipeline_stop:
	video_device_pipeline_stop(&cap->video);
free_scratch:
	dma_free_coherent(cap->dev, cap->format.sizeimage, cap->scratch, cap->scratch_dma);
	cap->scratch = NULL;
return_buffers:
	nx_return_buffers(cap, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void nx_stop_streaming(struct vb2_queue *q)
{
	struct nx_capture *cap = vb2_get_drv_priv(q);
	unsigned long flags;

	if (!cap->powered)
		return;
	reinit_completion(&cap->stop_done);
	spin_lock_irqsave(&cap->lock, flags);
	cap->stopping = true;
	spin_unlock_irqrestore(&cap->lock, flags);
	if (!wait_for_completion_timeout(&cap->stop_done, msecs_to_jiffies(2000)))
		dev_warn(cap->dev, "frame stop timed out\n");
	/* Quiesce receiver while the sensor still supplies its lane clock. */
	nx_hw_stop(cap);
	if (cap->sensor)
		v4l2_subdev_call(cap->sensor, video, s_stream, 0);
	video_device_pipeline_stop(&cap->video);
	nx_return_buffers(cap, VB2_BUF_STATE_ERROR);
	dma_free_coherent(cap->dev, cap->format.sizeimage, cap->scratch, cap->scratch_dma);
	cap->scratch = NULL;
}

static const struct vb2_ops nx_queue_ops = {
	.queue_setup = nx_queue_setup,
	.buf_prepare = nx_buffer_prepare,
	.buf_queue = nx_buffer_queue,
	.start_streaming = nx_start_streaming,
	.stop_streaming = nx_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int nx_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "s5p6818-capture", sizeof(cap->driver));
	strscpy(cap->card, "S5P6818 CSI/VIP", sizeof(cap->card));
	return 0;
}

static int nx_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_YUYV;
	return 0;
}

static int nx_get_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct nx_capture *cap = video_drvdata(file);

	f->fmt.pix = cap->format;
	return 0;
}

static int nx_sensor_fmt(struct nx_capture *cap, struct v4l2_pix_format *pix,
			 enum v4l2_subdev_format_whence which)
{
	struct v4l2_subdev_format fmt = {
		.which = which,
		.pad = cap->sensor_pad,
		.format = { .width = pix->width, .height = pix->height,
			.code = MEDIA_BUS_FMT_UYVY8_1X16, .field = V4L2_FIELD_NONE },
	};
	int ret;

	if (!cap->sensor)
		return -ENODEV;
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		ret = v4l2_subdev_call_state_try(cap->sensor, pad, set_fmt, &fmt);
	else
		ret = v4l2_subdev_call_state_active(cap->sensor, pad, set_fmt, &fmt);
	if (ret)
		return ret;
	if (fmt.format.code != MEDIA_BUS_FMT_UYVY8_1X16 ||
	    !fmt.format.width || !fmt.format.height ||
	    fmt.format.width > 2592 || fmt.format.height > 1944)
		return -EINVAL;
	memset(pix, 0, sizeof(*pix));
	pix->width = fmt.format.width;
	pix->height = fmt.format.height;
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	/*
	 * VIP can emit one final 64-byte burst beyond the visible packed frame.
	 * Include it in the advertised DMA capacity, never in bytesused.
	 */
	pix->sizeimage = pix->bytesperline * pix->height + NX_DMA_TAIL_BYTES;
	pix->colorspace = fmt.format.colorspace;
	pix->ycbcr_enc = fmt.format.ycbcr_enc;
	pix->quantization = fmt.format.quantization;
	pix->xfer_func = fmt.format.xfer_func;
	return 0;
}

static int nx_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	return nx_sensor_fmt(video_drvdata(file), &f->fmt.pix, V4L2_SUBDEV_FORMAT_TRY);
}

static int nx_set_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct nx_capture *cap = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&cap->queue))
		return -EBUSY;
	ret = nx_sensor_fmt(cap, &f->fmt.pix, V4L2_SUBDEV_FORMAT_ACTIVE);
	if (!ret)
		cap->format = f->fmt.pix;
	return ret;
}

static int nx_enum_size(struct file *file, void *priv, struct v4l2_frmsizeenum *f)
{
	struct nx_capture *cap = video_drvdata(file);
	struct v4l2_subdev_frame_size_enum size = {
		.index = f->index, .pad = cap->sensor_pad,
		.code = MEDIA_BUS_FMT_UYVY8_1X16, .which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	if (f->pixel_format != V4L2_PIX_FMT_YUYV || !cap->sensor)
		return -EINVAL;
	ret = v4l2_subdev_call(cap->sensor, pad, enum_frame_size, NULL, &size);
	if (ret)
		return ret;
	f->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	f->discrete.width = size.min_width;
	f->discrete.height = size.min_height;
	return 0;
}

static int nx_log_status(struct file *file, void *priv)
{
	struct nx_capture *cap = video_drvdata(file);

	if (cap->powered)
		dev_info(cap->dev,
			 "CSI ctrl=%08x dphy=%08x state=%08x errors=%08x VIP cfg=%08x irq=%08x count=%u/%u frames=%u\n",
			 readl(cap->csi + CSI_CTRL), readl(cap->csi + CSI_DPHY),
			 readl(cap->csi + 0x0c), readl(cap->csi + CSI_INTSRC),
			 readl(cap->vip + VIP_CONFIG), readl(cap->vip + VIP_ODINT),
			 readl(cap->vip + 0x24), readl(cap->vip + 0x28), cap->sequence);
	return 0;
}

static const struct v4l2_ioctl_ops nx_ioctl_ops = {
	.vidioc_log_status = nx_log_status,
	.vidioc_querycap = nx_querycap,
	.vidioc_enum_fmt_vid_cap = nx_enum_fmt,
	.vidioc_g_fmt_vid_cap = nx_get_fmt,
	.vidioc_try_fmt_vid_cap = nx_try_fmt,
	.vidioc_s_fmt_vid_cap = nx_set_fmt,
	.vidioc_enum_framesizes = nx_enum_size,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations nx_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static void nx_v4l2_release(struct v4l2_device *v4l2)
{
	struct nx_capture *cap = container_of(v4l2, struct nx_capture, v4l2);

	media_device_cleanup(&cap->media);
	mutex_destroy(&cap->mutex);
	kfree(cap);
}

static void nx_video_release(struct video_device *video)
{
	struct nx_capture *cap = video_get_drvdata(video);

	media_entity_cleanup(&video->entity);
	v4l2_device_put(&cap->v4l2);
}

static int nx_bound(struct v4l2_async_notifier *nf, struct v4l2_subdev *sd,
		    struct v4l2_async_connection *asc)
{
	struct nx_capture *cap = container_of(nf, struct nx_capture, notifier);
	int pad;

	pad = media_entity_get_fwnode_pad(&sd->entity, sd->fwnode, MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		return pad;
	cap->sensor = sd;
	cap->sensor_pad = pad;
	return 0;
}

static int nx_complete(struct v4l2_async_notifier *nf)
{
	struct nx_capture *cap = container_of(nf, struct nx_capture, notifier);
	int ret;

	if (cap->video_registered)
		return -EBUSY;
	cap->format.width = 1280;
	cap->format.height = 960;
	ret = nx_sensor_fmt(cap, &cap->format, V4L2_SUBDEV_FORMAT_ACTIVE);
	if (ret)
		return ret;
	ret = v4l2_device_register_subdev_nodes(&cap->v4l2);
	if (ret)
		return ret;
	v4l2_device_get(&cap->v4l2);
	ret = video_register_device(&cap->video, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_device_put(&cap->v4l2);
		return ret;
	}
	cap->video_registered = true;
	ret = media_create_pad_link(&cap->sensor->entity, cap->sensor_pad,
				   &cap->video.entity, 0,
				   MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		return ret;
	mutex_lock(&cap->mutex);
	cap->ready = true;
	mutex_unlock(&cap->mutex);
	dev_info(cap->dev, "OV5645 CSI/VIP capture ready as %s\n",
		 video_device_node_name(&cap->video));
	return 0;
}

static void nx_unbind(struct v4l2_async_notifier *nf, struct v4l2_subdev *sd,
		      struct v4l2_async_connection *asc)
{
	struct nx_capture *cap = container_of(nf, struct nx_capture, notifier);

	mutex_lock(&cap->mutex);
	if (cap->streaming)
		nx_stop_streaming(&cap->queue);
	if (cap->video_registered)
		vb2_queue_error(&cap->queue);
	cap->ready = false;
	cap->sensor = NULL;
	mutex_unlock(&cap->mutex);
}

static const struct v4l2_async_notifier_operations nx_notify_ops = {
	.bound = nx_bound,
	.complete = nx_complete,
	.unbind = nx_unbind,
};

static int nx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_fwnode_endpoint bus = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	struct nx_capture *cap;
	int ret;

	cap = kzalloc(sizeof(*cap), GFP_KERNEL);
	if (!cap)
		return -ENOMEM;
	cap->dev = dev;
	mutex_init(&cap->mutex);
	spin_lock_init(&cap->lock);
	init_completion(&cap->stop_done);
	INIT_LIST_HEAD(&cap->buffers);
	cap->csi = devm_platform_ioremap_resource_byname(pdev, "csi");
	if (IS_ERR(cap->csi)) {
		ret = PTR_ERR(cap->csi);
		goto free_cap;
	}
	cap->vip = devm_platform_ioremap_resource_byname(pdev, "vip");
	if (IS_ERR(cap->vip)) {
		ret = PTR_ERR(cap->vip);
		goto free_cap;
	}
	cap->sram = devm_platform_ioremap_resource_byname(pdev, "sram");
	if (IS_ERR(cap->sram)) {
		ret = PTR_ERR(cap->sram);
		goto free_cap;
	}
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto free_cap;
	cap->clocks[0].id = "mipi";
	cap->clocks[1].id = "vip";
	cap->clocks[2].id = "source";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(cap->clocks), cap->clocks);
	if (ret)
		goto free_cap;
	cap->resets[0].id = "mipi";
	cap->resets[1].id = "csi";
	cap->resets[2].id = "phy";
	cap->resets[3].id = "vip";
	cap->resets[4].id = "dsi";
	cap->resets[5].id = "phy-master";
	ret = devm_reset_control_bulk_get_exclusive(dev, ARRAY_SIZE(cap->resets), cap->resets);
	if (ret)
		goto free_cap;
	/* Both output domains use the same PLL2 / 2 clock, as on the BSP. */
	for (unsigned int i = 0; i < 2; i++) {
		ret = clk_set_parent(cap->clocks[i].clk, cap->clocks[2].clk);
		if (ret)
			goto free_cap;
		ret = clk_set_rate(cap->clocks[i].clk,
				   clk_get_rate(cap->clocks[2].clk) / 2);
		if (ret)
			goto free_cap;
	}
	cap->irq = platform_get_irq(pdev, 0);
	if (cap->irq < 0) {
		ret = cap->irq;
		goto free_cap;
	}
	ret = request_irq(cap->irq, nx_irq, IRQF_NO_AUTOEN, dev_name(dev), cap);
	if (ret)
		goto free_cap;

	media_device_init(&cap->media);
	cap->media.dev = dev;
	strscpy(cap->media.model, "S5P6818 CSI/VIP", sizeof(cap->media.model));
	cap->v4l2.mdev = &cap->media;
	cap->v4l2.release = nx_v4l2_release;
	ret = v4l2_device_register(dev, &cap->v4l2);
	if (ret)
		goto cleanup_media;
	ret = media_device_register(&cap->media);
	if (ret)
		goto unregister_v4l2;
	cap->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cap->queue.io_modes = VB2_MMAP | VB2_DMABUF;
	cap->queue.drv_priv = cap;
	cap->queue.buf_struct_size = sizeof(struct nx_buffer);
	cap->queue.ops = &nx_queue_ops;
	cap->queue.mem_ops = &vb2_dma_contig_memops;
	cap->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	cap->queue.lock = &cap->mutex;
	cap->queue.dev = dev;
	cap->queue.min_queued_buffers = 2;
	ret = vb2_queue_init(&cap->queue);
	if (ret)
		goto unregister_media;
	strscpy(cap->video.name, "s5p6818-capture", sizeof(cap->video.name));
	cap->video.v4l2_dev = &cap->v4l2;
	cap->video.fops = &nx_fops;
	cap->video.ioctl_ops = &nx_ioctl_ops;
	cap->video.release = nx_video_release;
	cap->video.lock = &cap->mutex;
	cap->video.queue = &cap->queue;
	cap->video.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->video.vfl_dir = VFL_DIR_RX;
	video_set_drvdata(&cap->video, cap);
	cap->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&cap->video.entity, 1, &cap->pad);
	if (ret)
		goto unregister_media;
	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep) {
		ret = -EINVAL;
		goto cleanup_entity;
	}
	ret = v4l2_fwnode_endpoint_parse(ep, &bus);
	if (ret || bus.bus.mipi_csi2.num_data_lanes != 2 ||
	    bus.bus.mipi_csi2.data_lanes[0] != 1 || bus.bus.mipi_csi2.data_lanes[1] != 2) {
		fwnode_handle_put(ep);
		ret = ret ?: -EINVAL;
		goto cleanup_entity;
	}
	v4l2_async_nf_init(&cap->notifier, &cap->v4l2);
	cap->notifier.ops = &nx_notify_ops;
	asc = v4l2_async_nf_add_fwnode_remote(&cap->notifier, ep, struct v4l2_async_connection);
	fwnode_handle_put(ep);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto cleanup_notifier;
	}
	platform_set_drvdata(pdev, cap);
	ret = v4l2_async_nf_register(&cap->notifier);
	if (!ret)
		return 0;
cleanup_notifier:
	v4l2_async_nf_unregister(&cap->notifier);
	v4l2_async_nf_cleanup(&cap->notifier);
cleanup_entity:
	if (cap->video_registered)
		vb2_video_unregister_device(&cap->video);
	else
		media_entity_cleanup(&cap->video.entity);
unregister_media:
	media_device_unregister(&cap->media);
unregister_v4l2:
	v4l2_device_unregister(&cap->v4l2);
	free_irq(cap->irq, cap);
	v4l2_device_put(&cap->v4l2);
	return ret;
cleanup_media:
	media_device_cleanup(&cap->media);
	free_irq(cap->irq, cap);
free_cap:
	mutex_destroy(&cap->mutex);
	kfree(cap);
	return dev_err_probe(dev, ret, "CSI/VIP probe failed\n");
}

static void nx_remove(struct platform_device *pdev)
{
	struct nx_capture *cap = platform_get_drvdata(pdev);

	if (cap->video_registered)
		vb2_video_unregister_device(&cap->video);
	v4l2_async_nf_unregister(&cap->notifier);
	v4l2_async_nf_cleanup(&cap->notifier);
	media_device_unregister(&cap->media);
	v4l2_device_unregister(&cap->v4l2);
	free_irq(cap->irq, cap);
	if (cap->clocks_enabled)
		clk_bulk_disable_unprepare(ARRAY_SIZE(cap->clocks), cap->clocks);
	v4l2_device_put(&cap->v4l2);
}

static const struct of_device_id nx_of_match[] = {
	{ .compatible = "nexell,s5p6818-capture" }, {}
};
MODULE_DEVICE_TABLE(of, nx_of_match);
static struct platform_driver nx_driver = {
	.probe = nx_probe,
	.remove = nx_remove,
	.driver = { .name = "s5p6818-capture", .of_match_table = nx_of_match },
};
module_platform_driver(nx_driver);
MODULE_DESCRIPTION("Nexell S5P6818 CSI-2/VIP capture");
MODULE_LICENSE("GPL");
