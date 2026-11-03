
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#if defined(CONFIG_ANDROID)
#include <linux/vmalloc.h>
#endif

#include "vvsensor.h"

/* --------------------------------------------------------------------------
 * Hardened usercopy helpers
 */
#ifdef CONFIG_HARDENED_USERCOPY
#define USER_TO_KERNEL(TYPE)						\
	do {								\
		TYPE tmp;						\
		unsigned long copy_ret;					\
		arg = (void *)(&tmp);					\
		copy_ret = copy_from_user(arg, arg_user, sizeof(TYPE));	\
	} while (0)

#define KERNEL_TO_USER(TYPE)						\
	do {								\
		unsigned long copy_ret;					\
		copy_ret = copy_to_user(arg_user, arg, sizeof(TYPE));	\
	} while (0)

#define USER_TO_KERNEL_VMALLOC(TYPE)					\
	do {								\
		unsigned long copy_ret;					\
		arg = vmalloc(sizeof(TYPE));				\
		copy_ret = copy_from_user(arg, arg_user, sizeof(TYPE));	\
	} while (0)

#define KERNEL_TO_USER_VMALLOC(TYPE)					\
	do {								\
		unsigned long copy_ret;					\
		copy_ret = copy_to_user(arg_user, arg, sizeof(TYPE));	\
		vfree(arg);						\
	} while (0)
#else
#define USER_TO_KERNEL(TYPE)
#define KERNEL_TO_USER(TYPE)
#define USER_TO_KERNEL_VMALLOC(TYPE)
#define KERNEL_TO_USER_VMALLOC(TYPE)
#endif

/* --------------------------------------------------------------------------
 * Register address encoding
 * Bits[31:16] = byte width, Bits[15:0] = address
 */
#define IMX462_REG_8BIT(n)		((1 << 16) | (n))
#define IMX462_REG_16BIT(n)		((2 << 16) | (n))
#define IMX462_REG_24BIT(n)		((3 << 16) | (n))
#define IMX462_REG_SIZE_SHIFT		16
#define IMX462_REG_ADDR_MASK		0xffff

/* --------------------------------------------------------------------------
 * Register definitions
 */
#define IMX462_STANDBY			IMX462_REG_8BIT(0x3000)
#define IMX462_REGHOLD			IMX462_REG_8BIT(0x3001)
#define IMX462_XMSTA			IMX462_REG_8BIT(0x3002)
#define IMX462_ADBIT			IMX462_REG_8BIT(0x3005)
#define IMX462_ADBIT_10BIT		(0 << 0)
#define IMX462_ADBIT_12BIT		(1 << 0)
#define IMX462_CTRL_07			IMX462_REG_8BIT(0x3007)
#define IMX462_VREVERSE			BIT(0)
#define IMX462_HREVERSE			BIT(1)
#define IMX462_WINMODE_1080P		(0 << 4)
#define IMX462_WINMODE_720P		(1 << 4)
#define IMX462_WINMODE_CROP		(4 << 4)
#define IMX462_FR_FDG_SEL		IMX462_REG_8BIT(0x3009)
#define IMX462_BLKLEVEL			IMX462_REG_16BIT(0x300a)
#define IMX462_GAIN			IMX462_REG_8BIT(0x3014)
#define IMX462_VMAX			IMX462_REG_24BIT(0x3018)
#define IMX462_VMAX_MAX			0x3ffff
#define IMX462_HMAX			IMX462_REG_8BIT(0x301c)
#define IMX462_HMAX_MAX			0xffff
#define IMX462_SHS1			IMX462_REG_16BIT(0x3020)
#define IMX462_WINWV_OB			IMX462_REG_8BIT(0x303a)
#define IMX462_WINPV			IMX462_REG_16BIT(0x303c)
#define IMX462_WINWV			IMX462_REG_16BIT(0x303e)
#define IMX462_WINPH			IMX462_REG_16BIT(0x3040)
#define IMX462_WINWH			IMX462_REG_16BIT(0x3042)
#define IMX462_OUT_CTRL			IMX462_REG_8BIT(0x3046)
#define IMX462_ODBIT_10BIT		(0 << 0)
#define IMX462_ODBIT_12BIT		(1 << 0)
#define IMX462_XSOUTSEL			IMX462_REG_8BIT(0x304b)
#define IMX462_XSOUTSEL_XVSOUTSEL_HIGH	(0 << 0)
#define IMX462_XSOUTSEL_XVSOUTSEL_VSYNC	(2 << 0)
#define IMX462_XSOUTSEL_XHSOUTSEL_HIGH	(0 << 2)
#define IMX462_XSOUTSEL_XHSOUTSEL_HSYNC	(2 << 2)
#define IMX462_INCKSEL1			IMX462_REG_8BIT(0x305c)
#define IMX462_INCKSEL2			IMX462_REG_8BIT(0x305d)
#define IMX462_INCKSEL3			IMX462_REG_8BIT(0x305e)
#define IMX462_INCKSEL4			IMX462_REG_8BIT(0x305f)
#define IMX462_PGCTRL			IMX462_REG_8BIT(0x308c)
#define IMX462_ADBIT1			IMX462_REG_8BIT(0x3129)
#define IMX462_ADBIT1_10BIT		0x1d
#define IMX462_ADBIT1_12BIT		0x00
#define IMX462_INCKSEL5			IMX462_REG_8BIT(0x315e)
#define IMX462_INCKSEL6			IMX462_REG_8BIT(0x3164)
#define IMX462_ADBIT2			IMX462_REG_8BIT(0x317c)
#define IMX462_ADBIT2_10BIT		0x12
#define IMX462_ADBIT2_12BIT		0x00
#define IMX462_ADBIT3			IMX462_REG_8BIT(0x31ec)
#define IMX462_ADBIT3_10BIT		0x37
#define IMX462_ADBIT3_12BIT		0x0e
#define IMX462_REPETITION		IMX462_REG_8BIT(0x3405)
#define IMX462_PHY_LANE_NUM		IMX462_REG_8BIT(0x3407)
#define IMX462_OPB_SIZE_V		IMX462_REG_8BIT(0x3414)
#define IMX462_Y_OUT_SIZE		IMX462_REG_16BIT(0x3418)
#define IMX462_CSI_DT_FMT		IMX462_REG_16BIT(0x3441)
#define IMX462_CSI_DT_FMT_RAW10		0x0a0a
#define IMX462_CSI_DT_FMT_RAW12		0x0c0c
#define IMX462_CSI_LANE_MODE		IMX462_REG_8BIT(0x3443)
#define IMX462_EXTCK_FREQ		IMX462_REG_16BIT(0x3444)
#define IMX462_TCLKPOST			IMX462_REG_16BIT(0x3446)
#define IMX462_THSZERO			IMX462_REG_16BIT(0x3448)
#define IMX462_THSPREPARE		IMX462_REG_16BIT(0x344a)
#define IMX462_TCLKTRAIL		IMX462_REG_16BIT(0x344c)
#define IMX462_THSTRAIL			IMX462_REG_16BIT(0x344e)
#define IMX462_TCLKZERO			IMX462_REG_16BIT(0x3450)
#define IMX462_TCLKPREPARE		IMX462_REG_16BIT(0x3452)
#define IMX462_TLPX			IMX462_REG_16BIT(0x3454)
#define IMX462_X_OUT_SIZE		IMX462_REG_16BIT(0x3472)
#define IMX462_INCKSEL7			IMX462_REG_8BIT(0x3480)

/* Pixel array geometry */
#define IMX462_PIXEL_ARRAY_WIDTH		1945
#define IMX462_PIXEL_ARRAY_HEIGHT		1097
#define IMX462_PIXEL_ARRAY_MARGIN_LEFT		12
#define IMX462_PIXEL_ARRAY_MARGIN_RIGHT		13
#define IMX462_PIXEL_ARRAY_MARGIN_TOP		8
#define IMX462_PIXEL_ARRAY_MARGIN_BOTTOM	9
#define IMX462_PIXEL_ARRAY_RECORDING_WIDTH	1920
#define IMX462_PIXEL_ARRAY_RECORDING_HEIGHT	1080

#define IMX462_BLACK_LEVEL_DEFAULT	3840
#define IMX462_EXPOSURE_OFFSET		2
#define IMX462_PIXEL_RATE		148500000

#define client_to_imx462(client) \
	container_of(i2c_get_clientdata(client), struct imx462, sd)

/* --------------------------------------------------------------------------
 * Data structures
 */
struct imx462_reg {
	u32 address;
	u32 val;
};

enum imx462_colour_variant {
	IMX462_VARIANT_COLOUR = 0,
	IMX462_VARIANT_MONO,
	IMX462_VARIANT_MAX
};

enum imx462_clk_freq {
	IMX462_CLK_37_125,
	IMX462_CLK_74_25,
	IMX462_NUM_CLK
};

struct imx462_clk_cfg {
	u8 incksel1;
	u8 incksel2;
	u8 incksel3;
	u8 incksel4;
	u8 incksel5;
	u8 incksel6;
};

struct imx462_mode {
	u32 width;
	u32 height;
	u32 hmax_min;
	u32 vmax_min;
	u8  link_freq_index;
	u8  ctrl_07;
	const struct imx462_reg *data;
	u32 data_size;
	const struct imx462_clk_cfg *clk_cfg;
};

struct imx462_csi_cfg {
	u16 repetition;
	u16 tclkpost;
	u16 thszero;
	u16 thsprepare;
	u16 tclktrail;
	u16 thstrail;
	u16 tclkzero;
	u16 tclkprepare;
	u16 tlpx;
};

struct imx462_model_info {
	enum imx462_colour_variant colour_variant;
	const struct imx462_reg *init_regs;
	size_t init_regs_num;
	const char *name;
};

struct imx462_capture_properties {
	__u64 max_lane_frequency;
	__u64 max_pixel_frequency;
	__u64 max_data_rate;
};

struct imx462 {
	struct device		*dev;
	struct clk		*xclk;
	struct regmap		*regmap;
	enum imx462_clk_freq	 xclk_idx;
	u8			 nlanes;
	const struct imx462_model_info *model;

	struct v4l2_subdev	 sd;
	struct media_pad	 pad;

	const struct imx462_mode *current_mode;

	struct regulator_bulk_data supplies[3];
	struct gpio_desc	*rst_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*exposure;
	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};
	struct mutex		 lock;
	vvcam_mode_info_t	 custom_cur_mode;
	struct imx462_capture_properties ocp;
	struct i2c_client	*i2c_client;
	/*
	 * sensor->format replaces v4l2_subdev active state which is not
	 * available in this BSP kernel (6.6.52-compulab).
	 */
	struct v4l2_mbus_framefmt format;
	unsigned int		 csi_id;
	u32			 stream_status;
	u32			 resume_status;
};

static inline struct imx462 *to_imx462(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx462, sd);
}

/* --------------------------------------------------------------------------
 * vvcam mode table
 */
static struct vvcam_mode_info_s imx462_mode_info[] = {
	{
		.index          = 0,
		.size = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = { .enable = 0 },
		.bayer_pattern  = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = 0x465,
			.curr_frm_len_lines    = 0x465,
			.one_line_exp_time_ns  = 14800,
			.max_integration_line  = 0x465 - 2,
			.min_integration_line  = 2,
			.max_again             = 30 * (1 << SENSOR_FIX_FRACBITS),
			.min_again             = 1  * (1 << SENSOR_FIX_FRACBITS),
			.max_dgain             = 42 * (1 << SENSOR_FIX_FRACBITS),
			.min_dgain             = 1  * (1 << SENSOR_FIX_FRACBITS),
			.start_exposure        = 8 * 400 * (1 << SENSOR_FIX_FRACBITS),
			.cur_fps               = 60 * (1 << SENSOR_FIX_FRACBITS),
			.max_fps               = 60 * (1 << SENSOR_FIX_FRACBITS),
			.min_fps               = 5  * (1 << SENSOR_FIX_FRACBITS),
			.min_afps              = 5  * (1 << SENSOR_FIX_FRACBITS),
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = { .mipi_lane = 4 },
	},
};

/* --------------------------------------------------------------------------
 * Regmap config
 */
static const struct regmap_config imx462_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

/* --------------------------------------------------------------------------
 * Low-level I/O helpers
 */
static int imx462_regmap_write(struct imx462 *sensor, u32 addr, u32 value)
{
	u8 data[3] = {value & 0xff, (value >> 8) & 0xff, value >> 16};
	int ret;

	ret = regmap_raw_write(sensor->regmap, addr & IMX462_REG_ADDR_MASK,
			       data, (addr >> IMX462_REG_SIZE_SHIFT) & 3);
	if (ret < 0)
		dev_err_ratelimited(sensor->dev,
				    "%u-bit write to 0x%04x failed: %d\n",
				    ((addr >> IMX462_REG_SIZE_SHIFT) & 3) * 8,
				    addr & IMX462_REG_ADDR_MASK, ret);
	return ret;
}

static int imx462_multi_reg_write(struct imx462 *sensor,
				  const struct imx462_reg *regs,
				  unsigned int num_regs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_regs; i++) {
		ret = imx462_regmap_write(sensor, regs[i].address, regs[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

/* Raw I2C helpers (used by vvcam IOCTL read/write reg) */
static int imx462_read_reg(struct imx462 *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 2;
	msg[0].buf   = buf;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 1;
	msg[1].buf   = val;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(sensor->dev, "Read reg error: reg=0x%x ret=%d\n",
			reg, ret);
		return ret;
	}
	if (ret != 2)
		return -EIO;

	return 0;
}

static int imx462_write_reg(struct imx462 *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	u8 buf[3] = {reg >> 8, reg & 0xff, val};

	if (i2c_master_send(client, buf, 3) < 0) {
		dev_err(sensor->dev, "Write reg error: reg=0x%x val=0x%x\n",
			reg, val);
		return -EIO;
	}
	return 0;
}

static int imx462_set_register_array(struct imx462 *sensor,
				     const struct imx462_reg *settings,
				     unsigned int num_settings)
{
	int ret;

	ret = imx462_multi_reg_write(sensor, settings, num_settings);
	if (ret < 0)
		return ret;

	usleep_range(10000, 11000);
	return 0;
}

/* --------------------------------------------------------------------------
 * Init register tables
 */
static const struct imx462_reg imx462_global_init_settings[] = {
	{ IMX462_WINWV_OB, 12 },
	{ IMX462_WINPH,    0 },
	{ IMX462_WINPV,    0 },
	{ IMX462_WINWH,    1948 },
	{ IMX462_WINWV,    1097 },
	{ IMX462_XSOUTSEL, IMX462_XSOUTSEL_XVSOUTSEL_VSYNC |
			   IMX462_XSOUTSEL_XHSOUTSEL_HSYNC },
	{ IMX462_REG_8BIT(0x3011), 0x02 },
	{ IMX462_REG_8BIT(0x3012), 0x64 },
	{ IMX462_REG_8BIT(0x3013), 0x00 },
};

static const struct imx462_reg imx462_global_init_settings_462[] = {
	{ IMX462_REG_8BIT(0x300f), 0x00 },
	{ IMX462_REG_8BIT(0x300e), 0x01 },
	{ IMX462_REG_8BIT(0x3009), 0x01 },   //Added for 60/50fps
	{ IMX462_REG_8BIT(0x3010), 0x21 },
	{ IMX462_REG_8BIT(0x3016), 0x09 },
	{ IMX462_REG_8BIT(0x301d), 0x0a },
	{ IMX462_REG_8BIT(0x3070), 0x02 },
	{ IMX462_REG_8BIT(0x3071), 0x11 },
	{ IMX462_REG_8BIT(0x309b), 0x10 },
	{ IMX462_REG_8BIT(0x309c), 0x22 },
	{ IMX462_REG_8BIT(0x30a2), 0x02 },
	{ IMX462_REG_8BIT(0x30a6), 0x20 },
	{ IMX462_REG_8BIT(0x30a8), 0x20 },
	{ IMX462_REG_8BIT(0x30aa), 0x20 },
	{ IMX462_REG_8BIT(0x30ac), 0x20 },
	{ IMX462_REG_8BIT(0x30b0), 0x43 },
	{ IMX462_REG_8BIT(0x3119), 0x9e },
	{ IMX462_REG_8BIT(0x311c), 0x1e },
	{ IMX462_REG_8BIT(0x311e), 0x08 },
	{ IMX462_REG_8BIT(0x3128), 0x05 },
	{ IMX462_REG_8BIT(0x313d), 0x83 },
	{ IMX462_REG_8BIT(0x3150), 0x03 },
	{ IMX462_REG_8BIT(0x317e), 0x00 },
	{ IMX462_REG_8BIT(0x3257), 0x03 },
	{ IMX462_REG_8BIT(0x3264), 0x1a },
	{ IMX462_REG_8BIT(0x3265), 0xb0 },
	{ IMX462_REG_8BIT(0x3266), 0x02 },
	{ IMX462_REG_8BIT(0x326b), 0x10 },
	{ IMX462_REG_8BIT(0x3274), 0x1b },
	{ IMX462_REG_8BIT(0x3275), 0xa0 },
	{ IMX462_REG_8BIT(0x3276), 0x02 },
	{ IMX462_REG_8BIT(0x32b8), 0x50 },
	{ IMX462_REG_8BIT(0x32b9), 0x10 },
	{ IMX462_REG_8BIT(0x32ba), 0x00 },
	{ IMX462_REG_8BIT(0x32bb), 0x04 },
	{ IMX462_REG_8BIT(0x32c8), 0x50 },
	{ IMX462_REG_8BIT(0x32c9), 0x10 },
	{ IMX462_REG_8BIT(0x32ca), 0x00 },
	{ IMX462_REG_8BIT(0x32cb), 0x04 },
	{ IMX462_REG_8BIT(0x332c), 0xd3 },
	{ IMX462_REG_8BIT(0x332d), 0x10 },
	{ IMX462_REG_8BIT(0x332e), 0x0d },
	{ IMX462_REG_8BIT(0x3358), 0x06 },
	{ IMX462_REG_8BIT(0x3359), 0xe1 },
	{ IMX462_REG_8BIT(0x335a), 0x11 },
	{ IMX462_REG_8BIT(0x3360), 0x1e },
	{ IMX462_REG_8BIT(0x3361), 0x61 },
	{ IMX462_REG_8BIT(0x3362), 0x10 },
	{ IMX462_REG_8BIT(0x33b0), 0x50 },
	{ IMX462_REG_8BIT(0x33b2), 0x1a },
	{ IMX462_REG_8BIT(0x33b3), 0x04 },
};

#define IMX462_NUM_CLK_REGS 2
static const struct imx462_reg xclk_regs[][IMX462_NUM_CLK_REGS] = {
	[IMX462_CLK_37_125] = {
		{ IMX462_EXTCK_FREQ, (37125 * 256) / 1000 },
		{ IMX462_INCKSEL7,   0x49 },
	},
	[IMX462_CLK_74_25] = {
		{ IMX462_EXTCK_FREQ, (74250 * 256) / 1000 },
		{ IMX462_INCKSEL7,   0x92 },
	},
};

static const struct imx462_reg imx462_1080p_settings[] = {
	{ IMX462_WINWV_OB,   12 },
	{ IMX462_OPB_SIZE_V, 10 },
	{ IMX462_X_OUT_SIZE, 1948 },
	{ IMX462_Y_OUT_SIZE, 1097 },
};

static const struct imx462_reg imx462_720p_settings[] = {
	{ IMX462_WINWV_OB,   6 },
	{ IMX462_OPB_SIZE_V, 4 },
	{ IMX462_X_OUT_SIZE, 1308 },
	{ IMX462_Y_OUT_SIZE, 729 },
};

static const struct imx462_reg imx462_10bit_settings[] = {
	{ IMX462_ADBIT,      IMX462_ADBIT_10BIT },
	{ IMX462_OUT_CTRL,   IMX462_ODBIT_10BIT },
	{ IMX462_ADBIT1,     IMX462_ADBIT1_10BIT },
	{ IMX462_ADBIT2,     IMX462_ADBIT2_10BIT },
	{ IMX462_ADBIT3,     IMX462_ADBIT3_10BIT },
	{ IMX462_CSI_DT_FMT, IMX462_CSI_DT_FMT_RAW10 },
};

static const struct imx462_reg imx462_12bit_settings[] = {
	{ IMX462_ADBIT,      IMX462_ADBIT_12BIT },
	{ IMX462_OUT_CTRL,   IMX462_ODBIT_12BIT },
	{ IMX462_ADBIT1,     IMX462_ADBIT1_12BIT },
	{ IMX462_ADBIT2,     IMX462_ADBIT2_12BIT },
	{ IMX462_ADBIT3,     IMX462_ADBIT3_12BIT },
	{ IMX462_CSI_DT_FMT, IMX462_CSI_DT_FMT_RAW12 },
};

/* --------------------------------------------------------------------------
 * CSI timing tables
 */
static const struct imx462_csi_cfg imx462_csi_222_75mhz = {
	.repetition = 0x10, .tclkpost = 0x57,  .thszero = 0x37,
	.thsprepare = 0x1f,   .tclktrail = 0x1f, .thstrail = 0x1f,
	.tclkzero   = 0x77,   .tclkprepare = 0x1f, .tlpx = 0x17,
};

static const struct imx462_csi_cfg imx462_csi_445_5mhz = {
	.repetition = 0x00, .tclkpost = 119, .thszero = 103,
	.thsprepare = 71,   .tclktrail = 55, .thstrail = 63,
	.tclkzero   = 255,  .tclkprepare = 63, .tlpx = 55,
};

static const struct imx462_csi_cfg imx462_csi_148_5mhz = {
	.repetition = 0x10, .tclkpost = 79,  .thszero = 47,
	.thsprepare = 23,   .tclktrail = 23, .thstrail = 23,
	.tclkzero   = 87,   .tclkprepare = 23, .tlpx = 23,
};

static const struct imx462_csi_cfg imx462_csi_297mhz = {
	.repetition = 0x00, .tclkpost = 103, .thszero = 87,
	.thsprepare = 47,   .tclktrail = 39, .thstrail = 47,
	.tclkzero   = 191,  .tclkprepare = 47, .tlpx = 39,
};

/* --------------------------------------------------------------------------
 * Link frequencies
 */
#define FREQ_INDEX_1080P 0
#define FREQ_INDEX_720P  1

static const s64 imx462_link_freq_2lanes[] = {
	[FREQ_INDEX_1080P] = 445500000,
	[FREQ_INDEX_720P]  = 297000000,
};

static const s64 imx462_link_freq_4lanes[] = {
	[FREQ_INDEX_1080P] = 222750000,
	[FREQ_INDEX_720P]  = 148500000,
};

static inline const s64 *imx462_link_freqs_ptr(const struct imx462 *sensor)
{
	return (sensor->nlanes == 2) ? imx462_link_freq_2lanes
				     : imx462_link_freq_4lanes;
}

static inline int imx462_link_freqs_num(const struct imx462 *sensor)
{
	return (sensor->nlanes == 2) ? ARRAY_SIZE(imx462_link_freq_2lanes)
				     : ARRAY_SIZE(imx462_link_freq_4lanes);
}

/* --------------------------------------------------------------------------
 * Clock configs
 */
static const struct imx462_clk_cfg imx462_1080p_clock_config[] = {
	[IMX462_CLK_37_125] = {
		.incksel1 = 0x18, .incksel2 = 0x03,
		.incksel3 = 0x20, .incksel4 = 0x01,
		.incksel5 = 0x1a, .incksel6 = 0x1a,
	},
	[IMX462_CLK_74_25] = {
		.incksel1 = 0x0c, .incksel2 = 0x03,
		.incksel3 = 0x10, .incksel4 = 0x01,
		.incksel5 = 0x1b, .incksel6 = 0x1b,
	},
};

static const struct imx462_clk_cfg imx462_720p_clock_config[] = {
	[IMX462_CLK_37_125] = {
		.incksel1 = 0x20, .incksel2 = 0x00,
		.incksel3 = 0x20, .incksel4 = 0x01,
		.incksel5 = 0x1a, .incksel6 = 0x1a,
	},
	[IMX462_CLK_74_25] = {
		.incksel1 = 0x10, .incksel2 = 0x00,
		.incksel3 = 0x10, .incksel4 = 0x01,
		.incksel5 = 0x1b, .incksel6 = 0x1b,
	},
};

/* --------------------------------------------------------------------------
 * Mode tables
 */
static const struct imx462_mode imx462_modes_2lanes[] = {
	{
		.width = 1920, .height = 1080,
		.hmax_min = 2200, .vmax_min = 1125,
		.link_freq_index = FREQ_INDEX_1080P,
		.ctrl_07   = IMX462_WINMODE_1080P,
		.data      = imx462_1080p_settings,
		.data_size = ARRAY_SIZE(imx462_1080p_settings),
		.clk_cfg   = imx462_1080p_clock_config,
	},
	{
		.width = 1280, .height = 720,
		.hmax_min = 3300, .vmax_min = 750,
		.link_freq_index = FREQ_INDEX_720P,
		.ctrl_07   = IMX462_WINMODE_720P,
		.data      = imx462_720p_settings,
		.data_size = ARRAY_SIZE(imx462_720p_settings),
		.clk_cfg   = imx462_720p_clock_config,
	},
};

static const struct imx462_mode imx462_modes_4lanes[] = {
	{
		.width = 1920, .height = 1080,
		.hmax_min = 2200, .vmax_min = 1125,
		.link_freq_index = FREQ_INDEX_1080P,
		.ctrl_07   = IMX462_WINMODE_1080P,
		.data      = imx462_1080p_settings,
		.data_size = ARRAY_SIZE(imx462_1080p_settings),
		.clk_cfg   = imx462_1080p_clock_config,
	},
	{
		.width = 1280, .height = 720,
		.hmax_min = 3300, .vmax_min = 750,
		.link_freq_index = FREQ_INDEX_720P,
		.ctrl_07   = IMX462_WINMODE_720P,
		.data      = imx462_720p_settings,
		.data_size = ARRAY_SIZE(imx462_720p_settings),
		.clk_cfg   = imx462_720p_clock_config,
	},
};

static inline const struct imx462_mode *imx462_modes_ptr(const struct imx462 *s)
{
	return (s->nlanes == 2) ? imx462_modes_2lanes : imx462_modes_4lanes;
}

static inline int imx462_modes_num(const struct imx462 *s)
{
	return (s->nlanes == 2) ? ARRAY_SIZE(imx462_modes_2lanes)
				: ARRAY_SIZE(imx462_modes_4lanes);
}

/* --------------------------------------------------------------------------
 * Format info
 */
struct imx462_format_info {
	u32 code[IMX462_VARIANT_MAX];
	u8  bpp;
	const struct imx462_reg *regs;
	unsigned int num_regs;
};

static const struct imx462_format_info imx462_formats[] = {
	{
		.code = {
			[IMX462_VARIANT_COLOUR] = MEDIA_BUS_FMT_SRGGB10_1X10,
			[IMX462_VARIANT_MONO]   = MEDIA_BUS_FMT_Y10_1X10,
		},
		.bpp      = 10,
		.regs     = imx462_10bit_settings,
		.num_regs = ARRAY_SIZE(imx462_10bit_settings),
	},
	{
		.code = {
			[IMX462_VARIANT_COLOUR] = MEDIA_BUS_FMT_SGBRG12_1X12,
			[IMX462_VARIANT_MONO]   = MEDIA_BUS_FMT_Y12_1X12,
		},
		.bpp      = 12,
		.regs     = imx462_12bit_settings,
		.num_regs = ARRAY_SIZE(imx462_12bit_settings),
	},
};

static const struct imx462_format_info *
imx462_format_info(const struct imx462 *sensor, u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx462_formats); i++) {
		if (imx462_formats[i].code[sensor->model->colour_variant] == code)
			return &imx462_formats[i];
	}
	return NULL;
}

/* --------------------------------------------------------------------------
 * Default format initialiser
 */
static void imx462_set_default_format(struct imx462 *sensor)
{
	sensor->format.width         = 1920;
	sensor->format.height        = 1080;
	sensor->format.code          = imx462_formats[0].code[sensor->model->colour_variant];
	sensor->format.field         = V4L2_FIELD_NONE;
	sensor->format.colorspace    = V4L2_COLORSPACE_RAW;
	sensor->format.ycbcr_enc     = V4L2_YCBCR_ENC_601;
	sensor->format.quantization  = V4L2_QUANTIZATION_FULL_RANGE;
	sensor->format.xfer_func     = V4L2_XFER_FUNC_NONE;
}

/* --------------------------------------------------------------------------
 * Sensor setup helpers
 */
static int imx462_set_clock(struct imx462 *sensor)
{
	const struct imx462_mode *mode = sensor->current_mode;
	enum imx462_clk_freq clk_idx = sensor->xclk_idx;
	const struct imx462_clk_cfg *clk_cfg = &mode->clk_cfg[clk_idx];
	int ret;

	ret = imx462_set_register_array(sensor, xclk_regs[clk_idx],
					IMX462_NUM_CLK_REGS);
	ret |= imx462_regmap_write(sensor, IMX462_INCKSEL1, clk_cfg->incksel1);
	ret |= imx462_regmap_write(sensor, IMX462_INCKSEL2, clk_cfg->incksel2);
	ret |= imx462_regmap_write(sensor, IMX462_INCKSEL3, clk_cfg->incksel3);
	ret |= imx462_regmap_write(sensor, IMX462_INCKSEL4, clk_cfg->incksel4);
	ret |= imx462_regmap_write(sensor, IMX462_INCKSEL5, clk_cfg->incksel5);
	ret |= imx462_regmap_write(sensor, IMX462_INCKSEL6, clk_cfg->incksel6);
	return ret;
}

static int imx462_set_data_lanes(struct imx462 *sensor)
{
	int ret;

	ret  = imx462_regmap_write(sensor, IMX462_PHY_LANE_NUM,
				   sensor->nlanes - 1);
	ret |= imx462_regmap_write(sensor, IMX462_CSI_LANE_MODE,
				   sensor->nlanes - 1);
	return ret;
}

static int imx462_set_black_level(struct imx462 *sensor,
				  const struct v4l2_mbus_framefmt *format,
				  unsigned int black_level, int *err)
{
	unsigned int bpp = imx462_format_info(sensor, format->code)->bpp;

	return imx462_regmap_write(sensor, IMX462_BLKLEVEL,
				   black_level >> (16 - bpp));
}

static int imx462_set_csi_config(struct imx462 *sensor)
{
	const s64 *link_freqs = imx462_link_freqs_ptr(sensor);
	const struct imx462_csi_cfg *csi_cfg;
	int ret = 0;

	switch (link_freqs[sensor->current_mode->link_freq_index]) {
	case 445500000:
		csi_cfg = &imx462_csi_445_5mhz;
		break;
	case 297000000:
		csi_cfg = &imx462_csi_297mhz;
		break;
	case 222750000:
		printk("link frequency = 222750000\n");
		csi_cfg = &imx462_csi_222_75mhz;
		break;
	case 148500000:
		csi_cfg = &imx462_csi_148_5mhz;
		break;
	default:
		return -EINVAL;
	}

	ret |= imx462_regmap_write(sensor, IMX462_REPETITION,  csi_cfg->repetition);
	ret |= imx462_regmap_write(sensor, IMX462_TCLKPOST,    csi_cfg->tclkpost);
	ret |= imx462_regmap_write(sensor, IMX462_THSZERO,     csi_cfg->thszero);
	ret |= imx462_regmap_write(sensor, IMX462_THSPREPARE,  csi_cfg->thsprepare);
	ret |= imx462_regmap_write(sensor, IMX462_TCLKTRAIL,   csi_cfg->tclktrail);
	ret |= imx462_regmap_write(sensor, IMX462_THSTRAIL,    csi_cfg->thstrail);
	ret |= imx462_regmap_write(sensor, IMX462_TCLKZERO,    csi_cfg->tclkzero);
	ret |= imx462_regmap_write(sensor, IMX462_TCLKPREPARE, csi_cfg->tclkprepare);
	ret |= imx462_regmap_write(sensor, IMX462_TLPX,        csi_cfg->tlpx);
	return ret;
}

static int imx462_setup_format(struct imx462 *sensor,
			       const struct v4l2_mbus_framefmt *format)
{
	const struct imx462_format_info *info;
	int ret;

	info = imx462_format_info(sensor, format->code);
	ret = imx462_set_register_array(sensor, info->regs, info->num_regs);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set format registers\n");
		return ret;
	}
	return imx462_set_black_level(sensor, format,
				      IMX462_BLACK_LEVEL_DEFAULT, &ret);
}

/* --------------------------------------------------------------------------
 * V4L2 controls
 */
static void imx462_exposure_update(struct imx462 *sensor,
				   const struct imx462_mode *mode)
{
	unsigned int exposure_max;

	exposure_max = sensor->vblank->val + mode->height - IMX462_EXPOSURE_OFFSET;
	__v4l2_ctrl_modify_range(sensor->exposure, 1, exposure_max, 1,
				 exposure_max);
}

static int imx462_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx462 *sensor = container_of(ctrl->handler,
					     struct imx462, ctrls);
	/*
	 * BSP kernel does not have v4l2_subdev_state_get_format().
	 * Read current format from sensor->format instead.
	 */
	const struct v4l2_mbus_framefmt *format = &sensor->format;
	int ret = 0, vmax;

	if (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)
		return 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		imx462_exposure_update(sensor, sensor->current_mode);

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx462_regmap_write(sensor, IMX462_GAIN, ctrl->val);
		break;

	case V4L2_CID_VBLANK:
		ret = imx462_regmap_write(sensor, IMX462_VMAX,
					  ctrl->val + sensor->current_mode->height);
		ctrl = sensor->exposure;
		fallthrough;
	case V4L2_CID_EXPOSURE:
		vmax = sensor->vblank->val + sensor->current_mode->height;
		ret = imx462_regmap_write(sensor, IMX462_SHS1, 0x02);
		break;

	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val) {
			imx462_set_black_level(sensor, format, 0, &ret);
			usleep_range(10000, 11000);
		} else {
			usleep_range(10000, 11000);
			imx462_set_black_level(sensor, format,
					       IMX462_BLACK_LEVEL_DEFAULT, &ret);
		}
		break;

	case V4L2_CID_HBLANK:
		ret = imx462_regmap_write(sensor, IMX462_HMAX, 0x50);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP: {
		u32 reg = sensor->current_mode->ctrl_07;

		if (sensor->hflip->val)
			reg |= IMX462_HREVERSE;
		if (sensor->vflip->val)
			reg |= IMX462_VREVERSE;
		ret = imx462_regmap_write(sensor, IMX462_CTRL_07, reg);
		break;
	}

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops imx462_ctrl_ops = {
	.s_ctrl = imx462_set_ctrl,
};

static const char * const imx462_test_pattern_menu[] = {
	"Disabled",
	"Sequence Pattern 1",
	"Horizontal Color-bar Chart",
	"Vertical Color-bar Chart",
	"Sequence Pattern 2",
	"Gradation Pattern 1",
	"Gradation Pattern 2",
	"000/555h Toggle Pattern",
};

static void imx462_ctrl_update(struct imx462 *sensor,
			       const struct imx462_mode *mode)
{
	unsigned int hblank_min = mode->hmax_min - mode->width;
	unsigned int hblank_max = IMX462_HMAX_MAX - mode->width;
	unsigned int vblank_min = mode->vmax_min - mode->height;
	unsigned int vblank_max = IMX462_VMAX_MAX - mode->height;

	__v4l2_ctrl_s_ctrl(sensor->link_freq, mode->link_freq_index);
	__v4l2_ctrl_modify_range(sensor->hblank, hblank_min, hblank_max, 1,
				 hblank_min);
	__v4l2_ctrl_modify_range(sensor->vblank, vblank_min, vblank_max, 1,
				 vblank_min);
}

static int imx462_ctrl_init(struct imx462 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 11);

	v4l2_ctrl_new_std(&sensor->ctrls, &imx462_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 238, 1, 0);

	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &imx462_ctrl_ops,
					     V4L2_CID_EXPOSURE, 1, 65535, 1,
					     65535);

	sensor->link_freq =
		v4l2_ctrl_new_int_menu(&sensor->ctrls, &imx462_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       imx462_link_freqs_num(sensor) - 1, 0,
				       imx462_link_freqs_ptr(sensor));
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(&sensor->ctrls, &imx462_ctrl_ops,
			  V4L2_CID_PIXEL_RATE,
			  IMX462_PIXEL_RATE, IMX462_PIXEL_RATE, 1,
			  IMX462_PIXEL_RATE);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &imx462_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx462_test_pattern_menu) - 1,
				     0, 0, imx462_test_pattern_menu);

	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx462_ctrl_ops,
					   V4L2_CID_HBLANK, 1, 1, 1, 1);
	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx462_ctrl_ops,
					   V4L2_CID_VBLANK, 1, 1, 1, 1);

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx462_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx462_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &sensor->hflip);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &imx462_ctrl_ops,
					&props);

	sensor->sd.ctrl_handler = &sensor->ctrls;

	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}
	return 0;
}

/* --------------------------------------------------------------------------
 * Streaming
 */
static int imx462_start_streaming(struct imx462 *sensor)
{
	int ret;

	ret = imx462_set_register_array(sensor, imx462_global_init_settings,
					ARRAY_SIZE(imx462_global_init_settings));
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set init registers\n");
		return ret;
	}

	ret = imx462_set_register_array(sensor, sensor->model->init_regs,
					sensor->model->init_regs_num);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set model-specific init registers\n");
		return ret;
	}

	ret = imx462_set_clock(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set clocks: %d\n", ret);
		return ret;
	}

	ret = imx462_set_data_lanes(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set data lanes: %d\n", ret);
		return ret;
	}

	ret = imx462_set_csi_config(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set CSI config: %d\n", ret);
		return ret;
	}

	/* Use sensor->format; BSP kernel has no active subdev state API */
	ret = imx462_setup_format(sensor, &sensor->format);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set frame format: %d\n", ret);
		return ret;
	}

	ret = imx462_set_register_array(sensor,
					sensor->current_mode->data,
					sensor->current_mode->data_size);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not set current mode: %d\n", ret);
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(sensor->sd.ctrl_handler);
	if (ret) {
		dev_err(sensor->dev, "Could not sync v4l2 controls: %d\n", ret);
		return ret;
	}

	ret = imx462_regmap_write(sensor, IMX462_STANDBY, 0x00);
	msleep(30);
	return imx462_regmap_write(sensor, IMX462_XMSTA, 0x00);
}

static int imx462_stop_streaming(struct imx462 *sensor)
{
	int ret;

	ret = imx462_regmap_write(sensor, IMX462_STANDBY, 0x01);
	msleep(30);
	return imx462_regmap_write(sensor, IMX462_XMSTA, 0x01);
}

static int imx462_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx462 *sensor = to_imx462(sd);
	int ret = 0;

	dev_info(sensor->dev, "%s enable=%d\n", __func__, enable);

	//mutex_lock(&sensor->lock);

	if (enable) {
		ret = imx462_start_streaming(sensor);
		msleep(100);
		if (ret) {
			dev_err(sensor->dev, "Start stream failed\n");
			goto unlock;
		}
		sensor->stream_status = 1;
	} else {
		imx462_stop_streaming(sensor);
		sensor->stream_status = 0;
	}

	__v4l2_ctrl_grab(sensor->vflip, enable);
	__v4l2_ctrl_grab(sensor->hflip, enable);

unlock:
	//mutex_unlock(&sensor->lock);
	return ret;
}

/* --------------------------------------------------------------------------
 * Pad ops
 */
static int imx462_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct imx462 *sensor = to_imx462(sd);

	if (code->index >= ARRAY_SIZE(imx462_formats))
		return -EINVAL;

	code->code = imx462_formats[code->index].code[sensor->model->colour_variant];
	return 0;
}

static int imx462_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx462 *sensor = to_imx462(sd);
	const struct imx462_mode *modes = imx462_modes_ptr(sensor);

	if (!imx462_format_info(sensor, fse->code))
		return -EINVAL;
	if (fse->index >= imx462_modes_num(sensor))
		return -EINVAL;

	fse->min_width  = modes[fse->index].width;
	fse->max_width  = modes[fse->index].width;
	fse->min_height = modes[fse->index].height;
	fse->max_height = modes[fse->index].height;
	return 0;
}

static int imx462_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx462 *sensor = to_imx462(sd);

	//mutex_lock(&sensor->lock);
	fmt->format = sensor->format;
	//mutex_unlock(&sensor->lock);
	return 0;
}

static int imx462_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx462 *sensor = to_imx462(sd);
	const struct imx462_mode *mode;

	mode = v4l2_find_nearest_size(imx462_modes_ptr(sensor),
				      imx462_modes_num(sensor),
				      width, height,
				      fmt->format.width, fmt->format.height);

	fmt->format.width        = mode->width;
	fmt->format.height       = mode->height;

	if (!imx462_format_info(sensor, fmt->format.code))
		fmt->format.code =
			imx462_formats[0].code[sensor->model->colour_variant];

	fmt->format.field        = V4L2_FIELD_NONE;
	fmt->format.colorspace   = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc    = V4L2_YCBCR_ENC_601;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func    = V4L2_XFER_FUNC_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		//mutex_lock(&sensor->lock);
		sensor->current_mode = mode;
		sensor->format = fmt->format;
		imx462_ctrl_update(sensor, mode);
		imx462_exposure_update(sensor, mode);
		//mutex_unlock(&sensor->lock);
	}

	return 0;
}

static int imx462_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct imx462 *sensor = to_imx462(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		//mutex_lock(&sensor->lock);
		sel->r.top    = IMX462_PIXEL_ARRAY_MARGIN_TOP
			      + (IMX462_PIXEL_ARRAY_RECORDING_HEIGHT - sensor->format.height) / 2
			      + sensor->vflip->val;
		sel->r.left   = IMX462_PIXEL_ARRAY_MARGIN_LEFT
			      + (IMX462_PIXEL_ARRAY_RECORDING_WIDTH - sensor->format.width) / 2
			      + sensor->hflip->val;
		sel->r.width  = sensor->format.width;
		sel->r.height = sensor->format.height;
		//mutex_unlock(&sensor->lock);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top    = 0;
		sel->r.left   = 0;
		sel->r.width  = IMX462_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX462_PIXEL_ARRAY_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top    = IMX462_PIXEL_ARRAY_MARGIN_TOP;
		sel->r.left   = IMX462_PIXEL_ARRAY_MARGIN_LEFT;
		sel->r.width  = IMX462_PIXEL_ARRAY_RECORDING_WIDTH;
		sel->r.height = IMX462_PIXEL_ARRAY_RECORDING_HEIGHT;
		return 0;

	default:
		return -EINVAL;
	}
}

/* --------------------------------------------------------------------------
 * vvcam private IOCTL helpers
 */
static int imx462_get_clk(struct imx462 *sensor, void *clk)
{
	struct vvcam_clk_s vvcam_clk;
	int ret;

	vvcam_clk.sensor_mclk       = clk_get_rate(sensor->xclk);
	vvcam_clk.csi_max_pixel_clk = sensor->ocp.max_pixel_frequency;

	ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
	return ret ? -EINVAL : 0;
}

static int imx462_query_capability(struct imx462 *sensor, void *arg)
{
	struct v4l2_capability *pcap;

	if (!sensor || !arg)
		return -EINVAL;

	pcap = (struct v4l2_capability *)arg;
	strcpy((char *)pcap->driver, "imx462");
	sprintf((char *)pcap->bus_info, "csi%d", sensor->csi_id);
	pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
		sensor->i2c_client->adapter ?
		(__u8)sensor->i2c_client->adapter->nr : 0xFF;
	return 0;
}

static int imx462_query_supports(struct imx462 *sensor, void *parry)
{
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;

	psensor_mode_arry->count = ARRAY_SIZE(imx462_mode_info);
	memcpy(&psensor_mode_arry->modes, imx462_mode_info,
	       sizeof(imx462_mode_info));
	return 0;
}

static int imx462_get_sensor_mode(struct imx462 *sensor, void *pmode)
{
	int ret;

	ret = copy_to_user(pmode, &sensor->custom_cur_mode,
			   sizeof(struct vvcam_mode_info_s));
	return ret ? -ENOMEM : 0;
}

static int imx462_set_sensor_mode(struct imx462 *sensor, void *pmode)
{
	struct vvcam_mode_info_s sensor_mode;
	int ret, i;

	ret = copy_from_user(&sensor_mode, pmode,
			     sizeof(struct vvcam_mode_info_s));
	if (ret)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(imx462_mode_info); i++) {
		if (imx462_mode_info[i].index == sensor_mode.index) {
			memcpy(&sensor->custom_cur_mode, &imx462_mode_info[i],
			       sizeof(struct vvcam_mode_info_s));
			return 0;
		}
	}
	dev_err(sensor->dev, "No matching mode for index=%u\n",
		sensor_mode.index);
	return -ENXIO;
}

static int imx462_get_fps(struct imx462 *sensor, u32 *pfps)
{
	*pfps = sensor->custom_cur_mode.ae_info.cur_fps;
	return 0;
}

/* --------------------------------------------------------------------------
 * Private IOCTL dispatcher
 */
static long imx462_priv_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
			      void *arg_user)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx462 *sensor = client_to_imx462(client);
	struct vvcam_sccb_data_s sensor_reg;
	long ret = 0;
	void *arg = arg_user;

	if (!sensor)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	switch (cmd) {
	case VVSENSORIOC_S_POWER:
		ret = 0;
		break;
	case VVSENSORIOC_S_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_G_CLK:
		ret = imx462_get_clk(sensor, arg);
		break;
	case VVSENSORIOC_RESET:
		ret = 0;
		break;
	case VIDIOC_QUERYCAP:
		ret = imx462_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		USER_TO_KERNEL_VMALLOC(struct vvcam_mode_info_array_s);
		ret = imx462_query_supports(sensor, arg);
		KERNEL_TO_USER_VMALLOC(struct vvcam_mode_info_array_s);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = imx462_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = imx462_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		USER_TO_KERNEL(int);
		ret = imx462_s_stream(&sensor->sd, *(int *)arg);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret  = copy_from_user(&sensor_reg, arg,
				      sizeof(struct vvcam_sccb_data_s));
		ret |= imx462_write_reg(sensor, sensor_reg.addr, sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret  = copy_from_user(&sensor_reg, arg,
				      sizeof(struct vvcam_sccb_data_s));
		ret |= imx462_read_reg(sensor, sensor_reg.addr,
				       (u8 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg,
				    sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_G_FPS:
		USER_TO_KERNEL(u32);
		ret = imx462_get_fps(sensor, (u32 *)arg);
		KERNEL_TO_USER(u32);
		break;
	case VVSENSORIOC_S_FPS:
		USER_TO_KERNEL(u32);
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		break;
	default:
		dev_warn(sensor->dev, "Unknown ioctl cmd=0x%x\n", cmd);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

/* --------------------------------------------------------------------------
 * Subdev / media ops tables
 */
static int imx462_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations imx462_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.link_setup    = imx462_link_setup,
};

static const struct v4l2_subdev_core_ops imx462_core_ops = {
	.subscribe_event   = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl             = imx462_priv_ioctl,
};

static const struct v4l2_subdev_video_ops imx462_video_ops = {
	.s_stream = imx462_s_stream,
};

static const struct v4l2_subdev_pad_ops imx462_pad_ops = {
	.enum_mbus_code  = imx462_enum_mbus_code,
	.enum_frame_size = imx462_enum_frame_size,
	.get_fmt         = imx462_get_fmt,
	.set_fmt         = imx462_set_fmt,
	.get_selection   = imx462_get_selection,
};

static const struct v4l2_subdev_ops imx462_subdev_ops = {
	.core  = &imx462_core_ops,
	.video = &imx462_video_ops,
	.pad   = &imx462_pad_ops,
};

/*
 * The following APIs are NOT available in 6.6.52-compulab BSP kernel
 * and are intentionally omitted:
 *   - v4l2_subdev_init_finalize / v4l2_subdev_cleanup
 *   - v4l2_subdev_lock_and_get_active_state
 *   - v4l2_subdev_get_locked_active_state / v4l2_subdev_unlock_state
 *   - v4l2_subdev_state_get_format
 *   - .init_state / .state_lock in internal_ops
 */
static const struct v4l2_subdev_internal_ops imx462_internal_ops = {
};

/* --------------------------------------------------------------------------
 * Subdev init / cleanup
 */
static int imx462_subdev_init(struct imx462 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	sensor->current_mode = &imx462_modes_ptr(sensor)[0];
	imx462_set_default_format(sensor);

	v4l2_i2c_subdev_init(&sensor->sd, client, &imx462_subdev_ops);
	sensor->sd.internal_ops    = &imx462_internal_ops;
	sensor->sd.flags          |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				     V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->sd.dev             = sensor->dev;
	sensor->sd.entity.ops      = &imx462_subdev_entity_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not register media entity\n");
		return ret;
	}

	ret = imx462_ctrl_init(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "Control initialization error %d\n", ret);
		goto err_media;
	}

	imx462_ctrl_update(sensor, sensor->current_mode);

	return 0;

err_media:
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static void imx462_subdev_cleanup(struct imx462 *sensor)
{
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

/* --------------------------------------------------------------------------
 * Clock init
 */
static int imx462_init_clk(struct imx462 *sensor)
{
	u32 xclk_freq;
	int ret;

	ret = device_property_read_u32(sensor->dev, "clock-frequency",
				       &xclk_freq);
	if (ret) {
		dev_err(sensor->dev, "Could not get xclk frequency\n");
		return ret;
	}

	switch (xclk_freq) {
	case 37125000:
		sensor->xclk_idx = IMX462_CLK_37_125;
		break;
	case 74250000:
		sensor->xclk_idx = IMX462_CLK_74_25;
		break;
	default:
		dev_err(sensor->dev,
			"External clock frequency %u is not supported\n",
			xclk_freq);
		return -EINVAL;
	}

	return clk_set_rate(sensor->xclk, xclk_freq);
}

/* --------------------------------------------------------------------------
 * DT parsing
 */
static int imx462_retrieve_capture_properties(
		struct imx462 *sensor,
		struct imx462_capture_properties *ocp)
{
	struct fwnode_handle *ep;
	__u64 mpf = 0;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!ep) {
		dev_err(sensor->dev, "missing endpoint node\n");
		return -ENODEV;
	}

	ret = fwnode_property_read_u64(ep, "max-pixel-frequency", &mpf);
	if (ret || mpf == 0)
		dev_dbg(sensor->dev, "no limit for max-pixel-frequency\n");

	fwnode_handle_put(ep);

	ocp->max_lane_frequency  = 0;
	ocp->max_pixel_frequency = mpf;
	ocp->max_data_rate       = 0;
	return 0;
}

static s64 imx462_check_link_freqs(const struct imx462 *sensor,
				   const struct v4l2_fwnode_endpoint *ep)
{
	const s64 *freqs = imx462_link_freqs_ptr(sensor);
	int freqs_count  = imx462_link_freqs_num(sensor);
	int i, j;

	for (i = 0; i < freqs_count; i++) {
		for (j = 0; j < ep->nr_of_link_frequencies; j++)
			if (freqs[i] == ep->link_frequencies[j])
				break;
		if (j == ep->nr_of_link_frequencies)
			return freqs[i];
	}
	return 0;
}

static int imx462_parse_dt(struct imx462 *sensor)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint;
	s64 fq;
	int ret;

	sensor->model = of_device_get_match_data(sensor->dev);

	ret = of_property_read_u32(sensor->dev->of_node, "csi_id",
				   &sensor->csi_id);
	if (ret) {
		dev_err(sensor->dev, "csi_id missing or invalid\n");
		return ret;
	}

	ret = imx462_retrieve_capture_properties(sensor, &sensor->ocp);
	if (ret)
		dev_warn(sensor->dev, "retrieve capture properties error\n");

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!endpoint) {
		dev_err(sensor->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret == -ENXIO) {
		dev_err(sensor->dev, "Unsupported bus type, should be CSI2\n");
		goto done;
	} else if (ret) {
		dev_err(sensor->dev, "Parsing endpoint node failed\n");
		goto done;
	}

	sensor->nlanes = ep.bus.mipi_csi2.num_data_lanes;
	if (sensor->nlanes != 2 && sensor->nlanes != 4) {
		dev_err(sensor->dev, "Invalid data lanes: %d\n", sensor->nlanes);
		ret = -EINVAL;
		goto done;
	}

	if (!ep.nr_of_link_frequencies) {
		dev_err(sensor->dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto done;
	}

	fq = imx462_check_link_freqs(sensor, &ep);
	if (fq) {
		dev_err(sensor->dev,
			"Link frequency %lld is not supported\n", fq);
		ret = -EINVAL;
		goto done;
	}

	ret = 0;
done:
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

/* --------------------------------------------------------------------------
 * Model table
 */
static const struct imx462_model_info imx462_models[] = {
	[0] = {
		.colour_variant = IMX462_VARIANT_COLOUR,
		.init_regs      = imx462_global_init_settings_462,
		.init_regs_num  = ARRAY_SIZE(imx462_global_init_settings_462),
		.name           = "imx462",
	},
};

/* --------------------------------------------------------------------------
 * Probe / remove
 */
static int imx462_probe(struct i2c_client *client)
{
	struct imx462 *sensor;
	int ret;

	dev_info(&client->dev, "IMX462: probing at addr=0x%02x\n",
		 client->addr);

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;
	sensor->dev        = &client->dev;

	sensor->regmap = devm_regmap_init_i2c(client, &imx462_regmap_config);
	if (IS_ERR(sensor->regmap)) {
		dev_err(sensor->dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	ret = imx462_parse_dt(sensor);
	if (ret)
		return ret;

	sensor->xclk = devm_clk_get(sensor->dev, "csi_mclk");
	if (IS_ERR(sensor->xclk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->xclk),
				     "Could not get csi_mclk\n");

	ret = imx462_init_clk(sensor);
	if (ret)
		return ret;

	memcpy(&sensor->custom_cur_mode, &imx462_mode_info[0],
	       sizeof(struct vvcam_mode_info_s));

	mutex_init(&sensor->lock);

	ret = imx462_subdev_init(sensor);
	if (ret)
		goto err_pm;

	v4l2_i2c_subdev_set_name(&sensor->sd, client,
				 sensor->model->name, NULL);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret < 0) {
		dev_err(sensor->dev, "Could not register v4l2 device\n");
		goto err_subdev;
	}

	dev_info(sensor->dev, "IMX462: probe success\n");
	return 0;

err_subdev:
	imx462_subdev_cleanup(sensor);
err_pm:
	mutex_destroy(&sensor->lock);
	return ret;
}

static void imx462_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx462 *sensor  = to_imx462(sd);

	v4l2_async_unregister_subdev(sd);
	imx462_subdev_cleanup(sensor);

	mutex_destroy(&sensor->lock);
}

/* --------------------------------------------------------------------------
 * Module registration
 */
static const struct i2c_device_id imx462_id[] = {
	{ "imx462", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, imx462_id);

static const struct of_device_id imx462_of_match[] = {
	{
		.compatible = "sony,imx462",
		.data       = &imx462_models[0],
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx462_of_match);

static struct i2c_driver imx462_i2c_driver = {
	.probe    = imx462_probe,
	.remove   = imx462_remove,
	.id_table = imx462_id,
	.driver   = {
		.name           = "imx462",
		.of_match_table = imx462_of_match,
	},
};

module_i2c_driver(imx462_i2c_driver);

MODULE_DESCRIPTION("Sony IMX462 CMOS Image Sensor Driver Modified and ported for the compulab/nxp IMX8MPlus variant");
MODULE_AUTHOR("Vishnu <siddavatamvishnureddy@gmail.com>");
MODULE_LICENSE("GPL v2");
