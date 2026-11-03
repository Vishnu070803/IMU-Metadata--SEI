#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/v4l2-mediabus.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#if defined(CONFIG_ANDROID)
#include <linux/vmalloc.h>
#endif
#include "vvsensor.h"

#include "IMX415_DOL_HDR.h"
#include "IMX415_4K_LINEAR.h"
#include "IMX415_1080P.h"


#define IMX415_PIXEL_ARRAY_TOP 0
#define IMX415_PIXEL_ARRAY_LEFT 0
#define IMX415_PIXEL_ARRAY_WIDTH 1920
#define IMX415_PIXEL_ARRAY_HEIGHT 1080
#define IMX415_PIXEL_ARRAY_VBLANK 58

#define IMX415_NUM_CLK_PARAM_REGS 11

/* Register size encoding macros (from kernel 6.6 API) */
#define IMX415_REG_8BIT(n) ((1 << 16) | (n))
#define IMX415_REG_16BIT(n) ((2 << 16) | (n))
#define IMX415_REG_24BIT(n) ((3 << 16) | (n))
#define IMX415_REG_SIZE_SHIFT 16
#define IMX415_REG_ADDR_MASK 0xffff

/* Register structure for register arrays */
struct imx415_reg {
  u32 address;
  u32 val;
};

/* Regmap configuration */
static const struct regmap_config imx415_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
};

#define IMX415_MODE IMX415_REG_8BIT(0x3000)
#define IMX415_MODE_OPERATING (0)
#define IMX415_MODE_STANDBY BIT(0)
#define IMX415_REGHOLD IMX415_REG_8BIT(0x3001)
#define IMX415_REGHOLD_INVALID (0)
#define IMX415_REGHOLD_VALID BIT(0)
#define IMX415_XMSTA IMX415_REG_8BIT(0x3002)
#define IMX415_XMSTA_START (0)
#define IMX415_XMSTA_STOP BIT(0)
#define IMX415_BCWAIT_TIME IMX415_REG_16BIT(0x3008)
#define IMX415_CPWAIT_TIME IMX415_REG_16BIT(0x300a)
#define IMX415_WINMODE IMX415_REG_8BIT(0x301c)
#define IMX415_ADDMODE IMX415_REG_8BIT(0x3022)
#define IMX415_REVERSE IMX415_REG_8BIT(0x3030)
#define IMX415_HREVERSE_SHIFT (0)
#define IMX415_VREVERSE_SHIFT BIT(0)
#define IMX415_ADBIT IMX415_REG_8BIT(0x3031)
#define IMX415_MDBIT IMX415_REG_8BIT(0x3032)
#define IMX415_SYS_MODE IMX415_REG_8BIT(0x3033)
#define IMX415_OUTSEL IMX415_REG_8BIT(0x30c0)
#define IMX415_DRV IMX415_REG_8BIT(0x30c1)
#define IMX415_VMAX IMX415_REG_24BIT(0x3024)
#define IMX415_HMAX IMX415_REG_16BIT(0x3028)
#define IMX415_SHR0 IMX415_REG_24BIT(0x3050)
#define IMX415_GAIN_PCG_0 IMX415_REG_16BIT(0x3090)
#define IMX415_AGAIN_MIN 0
#define IMX415_AGAIN_MAX 100
#define IMX415_AGAIN_STEP 1
#define IMX415_BLKLEVEL IMX415_REG_16BIT(0x30e2)
#define IMX415_BLKLEVEL_DEFAULT 50
#define IMX415_TPG_EN_DUOUT IMX415_REG_8BIT(0x30e4)
#define IMX415_TPG_PATSEL_DUOUT IMX415_REG_8BIT(0x30e6)
#define IMX415_TPG_COLORWIDTH IMX415_REG_8BIT(0x30e8)
#define IMX415_TESTCLKEN_MIPI IMX415_REG_8BIT(0x3110)
#define IMX415_INCKSEL1 IMX415_REG_8BIT(0x3115)
#define IMX415_INCKSEL2 IMX415_REG_8BIT(0x3116)
#define IMX415_INCKSEL3 IMX415_REG_16BIT(0x3118)
#define IMX415_INCKSEL4 IMX415_REG_16BIT(0x311a)
#define IMX415_INCKSEL5 IMX415_REG_8BIT(0x311e)
#define IMX415_DIG_CLP_MODE IMX415_REG_8BIT(0x32c8)
#define IMX415_WRJ_OPEN IMX415_REG_8BIT(0x3390)
#define IMX415_SENSOR_INFO IMX415_REG_16BIT(0x3f12)
#define IMX415_SENSOR_INFO_MASK 0xfff
#define IMX415_CHIP_ID 0x514
#define IMX415_LANEMODE IMX415_REG_16BIT(0x4001)
#define IMX415_LANEMODE_2 1
#define IMX415_LANEMODE_4 3
#define IMX415_TXCLKESC_FREQ IMX415_REG_16BIT(0x4004)
#define IMX415_INCKSEL6 IMX415_REG_8BIT(0x400c)
#define IMX415_TCLKPOST IMX415_REG_16BIT(0x4018)
#define IMX415_TCLKPREPARE IMX415_REG_16BIT(0x401a)
#define IMX415_TCLKTRAIL IMX415_REG_16BIT(0x401c)
#define IMX415_TCLKZERO IMX415_REG_16BIT(0x401e)
#define IMX415_THSPREPARE IMX415_REG_16BIT(0x4020)
#define IMX415_THSZERO IMX415_REG_16BIT(0x4022)
#define IMX415_THSTRAIL IMX415_REG_16BIT(0x4024)
#define IMX415_THSEXIT IMX415_REG_16BIT(0x4026)
#define IMX415_TLPX IMX415_REG_16BIT(0x4028)
#define IMX415_INCKSEL7 IMX415_REG_8BIT(0x4074)

// Added registers
#define IMX415_HADD IMX415_REG_8BIT(0x3020)
#define IMX415_VADD IMX415_REG_8BIT(0x3021)
#define IMX415_ADDMODE IMX415_REG_8BIT(0x3022)
#define IMX415_DIG_CLP_VSTART IMX415_REG_8BIT(0x30D9)
#define IMX415_DIG_CLP_VNUM IMX415_REG_8BIT(0x30DA)

#define IMX415_SENS_PAD_SOURCE 0
#define IMX415_SENS_PADS_NUM 1

#define client_to_imx415(client)                                               \
  container_of(i2c_get_clientdata(client), struct imx415, subdev)

/*
Use USER_TO_KERNEL/KERNEL_TO_USER to fix "uaccess" exception on run time.
Also, use "copy_ret" to fix the build issue as below.
error: ignoring return value of function declared with 'warn_unused_result'
attribute.
*/

#ifdef CONFIG_HARDENED_USERCOPY
#define USER_TO_KERNEL(TYPE)                                                   \
  do {                                                                         \
    TYPE tmp;                                                                  \
    unsigned long copy_ret;                                                    \
    arg = (void *)(&tmp);                                                      \
    copy_ret = copy_from_user(arg, arg_user, sizeof(TYPE));                    \
  } while (0)

#define KERNEL_TO_USER(TYPE)                                                   \
  do {                                                                         \
    unsigned long copy_ret;                                                    \
    copy_ret = copy_to_user(arg_user, arg, sizeof(TYPE));                      \
  } while (0)

#define USER_TO_KERNEL_VMALLOC(TYPE)                                           \
  do {                                                                         \
    unsigned long copy_ret;                                                    \
    arg = vmalloc(sizeof(TYPE));                                               \
    copy_ret = copy_from_user(arg, arg_user, sizeof(TYPE));                    \
  } while (0)

#define KERNEL_TO_USER_VMALLOC(TYPE)                                           \
  do {                                                                         \
    unsigned long copy_ret;                                                    \
    copy_ret = copy_to_user(arg_user, arg, sizeof(TYPE));                      \
    vfree(arg);                                                                \
  } while (0)

#else
#define USER_TO_KERNEL(TYPE)
#define KERNEL_TO_USER(TYPE)
#define USER_TO_KERNEL_VMALLOC(TYPE)
#define KERNEL_TO_USER_VMALLOC(TYPE)
#endif

/*
 * The IMX415 data sheet uses lane rates but v4l2 uses link frequency to
 * describe MIPI CSI-2 speed. This driver uses lane rates wherever possible
 * and converts them to link frequencies by a factor of two when needed.
 */
static const s64 link_freq_menu_items[] = {
    594000000 / 2, 720000000 / 2, 891000000 / 2, 1440000000 / 2, 1485000000 / 2,
};

struct imx415_clk_params {
  u64 lane_rate;
  u64 inck;
  struct imx415_reg regs[IMX415_NUM_CLK_PARAM_REGS];
};

/* INCK Settings - includes all lane rate and INCK dependent registers */
static const struct imx415_clk_params imx415_clk_params[] = {
    {
        .lane_rate = 594000000UL,
        .inck = 27000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x05D},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x042},
        .regs[2] = {IMX415_SYS_MODE, 0x7},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x084},
        .regs[6] = {IMX415_INCKSEL4, 0x0E7},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x06C0},
    },
    {
        .lane_rate = 594000000UL,
        .inck = 37125000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x07F},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x05B},
        .regs[2] = {IMX415_SYS_MODE, 0x7},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x24},
        .regs[5] = {IMX415_INCKSEL3, 0x080},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x24},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0984},
    },
    {
        .lane_rate = 594000000UL,
        .inck = 74250000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0FF},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B6},
        .regs[2] = {IMX415_SYS_MODE, 0x7},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x080},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1290},
    },
    {
        .lane_rate = 720000000UL,
        .inck = 24000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x054},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x03B},
        .regs[2] = {IMX415_SYS_MODE, 0x9},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x0B4},
        .regs[6] = {IMX415_INCKSEL4, 0x0FC},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0600},
    },
    {
        .lane_rate = 720000000UL,
        .inck = 72000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0F8},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B0},
        .regs[2] = {IMX415_SYS_MODE, 0x9},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x0A0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1200},
    },
    {
        .lane_rate = 891000000UL,
        .inck = 27000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x05D},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x042},
        .regs[2] = {IMX415_SYS_MODE, 0x5},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x0C6},
        .regs[6] = {IMX415_INCKSEL4, 0x0E7},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x06C0},
    },
    {
        .lane_rate = 891000000UL,
        .inck = 37125000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x07F},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x05B},
        .regs[2] = {IMX415_SYS_MODE, 0x5},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x24},
        .regs[5] = {IMX415_INCKSEL3, 0x0C0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x24},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0948},
    },
    {
        .lane_rate = 891000000UL,
        .inck = 74250000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0FF},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B6},
        .regs[2] = {IMX415_SYS_MODE, 0x5},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x0C0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x0},
        .regs[9] = {IMX415_INCKSEL7, 0x1},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1290},
    },
    {
        .lane_rate = 1440000000UL,
        .inck = 24000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x054},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x03B},
        .regs[2] = {IMX415_SYS_MODE, 0x8},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x0B4},
        .regs[6] = {IMX415_INCKSEL4, 0x0FC},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0600},
    },
    {
        .lane_rate = 1440000000UL,
        .inck = 72000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0F8},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B0},
        .regs[2] = {IMX415_SYS_MODE, 0x8},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x0A0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1200},
    },
    {
        .lane_rate = 1485000000UL,
        .inck = 27000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x05D},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x042},
        .regs[2] = {IMX415_SYS_MODE, 0x8},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x0A5},
        .regs[6] = {IMX415_INCKSEL4, 0x0E7},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x06C0},
    },
    {
        .lane_rate = 1485000000UL,
        .inck = 37125000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x07F},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x05B},
        .regs[2] = {IMX415_SYS_MODE, 0x8},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x24},
        .regs[5] = {IMX415_INCKSEL3, 0x0A0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x24},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0948},
    },
    {
        .lane_rate = 1485000000UL,
        .inck = 74250000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0FF},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B6},
        .regs[2] = {IMX415_SYS_MODE, 0x8},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x0A0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1290},
    },
    {
        .lane_rate = 1782000000UL,
        .inck = 27000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x05D},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x042},
        .regs[2] = {IMX415_SYS_MODE, 0x4},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x0C6},
        .regs[6] = {IMX415_INCKSEL4, 0x0E7},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x06C0},
    },
    {
        .lane_rate = 1782000000UL,
        .inck = 37125000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x07F},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x05B},
        .regs[2] = {IMX415_SYS_MODE, 0x4},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x24},
        .regs[5] = {IMX415_INCKSEL3, 0x0C0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x24},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0948},
    },
    {
        .lane_rate = 1782000000UL,
        .inck = 74250000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0FF},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B6},
        .regs[2] = {IMX415_SYS_MODE, 0x4},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x0C0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1290},
    },
    {
        .lane_rate = 2079000000UL,
        .inck = 27000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x05D},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x042},
        .regs[2] = {IMX415_SYS_MODE, 0x2},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x0E7},
        .regs[6] = {IMX415_INCKSEL4, 0x0E7},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x06C0},
    },
    {
        .lane_rate = 2079000000UL,
        .inck = 37125000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x07F},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x05B},
        .regs[2] = {IMX415_SYS_MODE, 0x2},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x24},
        .regs[5] = {IMX415_INCKSEL3, 0x0E0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x24},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0948},
    },
    {
        .lane_rate = 2079000000UL,
        .inck = 74250000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0FF},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B6},
        .regs[2] = {IMX415_SYS_MODE, 0x2},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x0E0},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1290},
    },
    {
        .lane_rate = 2376000000UL,
        .inck = 27000000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x05D},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x042},
        .regs[2] = {IMX415_SYS_MODE, 0x0},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x23},
        .regs[5] = {IMX415_INCKSEL3, 0x108},
        .regs[6] = {IMX415_INCKSEL4, 0x0E7},
        .regs[7] = {IMX415_INCKSEL5, 0x23},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x06C0},
    },
    {
        .lane_rate = 2376000000UL,
        .inck = 37125000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x07F},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x05B},
        .regs[2] = {IMX415_SYS_MODE, 0x0},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x24},
        .regs[5] = {IMX415_INCKSEL3, 0x100},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x24},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x0948},
    },
    {
        .lane_rate = 2376000000UL,
        .inck = 74250000,
        .regs[0] = {IMX415_BCWAIT_TIME, 0x0FF},
        .regs[1] = {IMX415_CPWAIT_TIME, 0x0B6},
        .regs[2] = {IMX415_SYS_MODE, 0x0},
        .regs[3] = {IMX415_INCKSEL1, 0x00},
        .regs[4] = {IMX415_INCKSEL2, 0x28},
        .regs[5] = {IMX415_INCKSEL3, 0x100},
        .regs[6] = {IMX415_INCKSEL4, 0x0E0},
        .regs[7] = {IMX415_INCKSEL5, 0x28},
        .regs[8] = {IMX415_INCKSEL6, 0x1},
        .regs[9] = {IMX415_INCKSEL7, 0x0},
        .regs[10] = {IMX415_TXCLKESC_FREQ, 0x1290},
    },
};

/* all-pixel 2-lane 720 Mbps 15.74 Hz mode */
static const struct imx415_reg imx415_mode_2_720[] = {
    {IMX415_VMAX, 0x08CA},
    {IMX415_HMAX, 0x07F0},
    {IMX415_LANEMODE, IMX415_LANEMODE_2},
    {IMX415_TCLKPOST, 0x006F},
    {IMX415_TCLKPREPARE, 0x002F},
    {IMX415_TCLKTRAIL, 0x002F},
    {IMX415_TCLKZERO, 0x00BF},
    {IMX415_THSPREPARE, 0x002F},
    {IMX415_THSZERO, 0x0057},
    {IMX415_THSTRAIL, 0x002F},
    {IMX415_THSEXIT, 0x004F},
    {IMX415_TLPX, 0x0027},
};

/* all-pixel 2-lane 1440 Mbps 30.01 Hz mode */
static const struct imx415_reg imx415_mode_2_1440[] = {
    {IMX415_VMAX, 0x08CA},
    {IMX415_HMAX, 0x042A},
    {IMX415_LANEMODE, IMX415_LANEMODE_2},
    {IMX415_TCLKPOST, 0x009F},
    {IMX415_TCLKPREPARE, 0x0057},
    {IMX415_TCLKTRAIL, 0x0057},
    {IMX415_TCLKZERO, 0x0187},
    {IMX415_THSPREPARE, 0x005F},
    {IMX415_THSZERO, 0x00A7},
    {IMX415_THSTRAIL, 0x005F},
    {IMX415_THSEXIT, 0x0097},
    {IMX415_TLPX, 0x004F},
};

/* all-pixel 4-lane 891 Mbps 30 Hz mode */
static const struct imx415_reg imx415_mode_4_891[] = {
    {IMX415_VMAX, 0x08CA},
    {IMX415_HMAX, 0x044C},
    {IMX415_LANEMODE, IMX415_LANEMODE_4},
    {IMX415_TCLKPOST, 0x007F},
    {IMX415_TCLKPREPARE, 0x0037},
    {IMX415_TCLKTRAIL, 0x0037},
    {IMX415_TCLKZERO, 0x00F7},
    {IMX415_THSPREPARE, 0x003F},
    {IMX415_THSZERO, 0x006F},
    {IMX415_THSTRAIL, 0x003F},
    {IMX415_THSEXIT, 0x005F},
    {IMX415_TLPX, 0x002F},
};

/* all-pixel 4-lane 1440 Mbps 30.01 Hz mode */
static const struct imx415_reg imx415_mode_4_1440[] = {
    {IMX415_VMAX, 0x08CA},
    {IMX415_HMAX, 0x042A},
    {IMX415_LANEMODE, IMX415_LANEMODE_4},
    {IMX415_TCLKPOST, 0x009F},
    {IMX415_TCLKPREPARE, 0x0057},
    {IMX415_TCLKTRAIL, 0x0057},
    {IMX415_TCLKZERO, 0x0187},
    {IMX415_THSPREPARE, 0x005F},
    {IMX415_THSZERO, 0x00A7},
    {IMX415_THSTRAIL, 0x005F},
    {IMX415_THSEXIT, 0x0097},
    {IMX415_TLPX, 0x004F},
};

/* all-pixel 4-lane 1485 Mbps 15.00 Hz mode */
static const struct imx415_reg imx415_mode_4_1485[] = {
    {IMX415_VMAX, 0x0F3A},
    {IMX415_HMAX, 0x7B},
    {IMX415_LANEMODE, IMX415_LANEMODE_4},
    {IMX415_TCLKPOST, 0x00A7},
    {IMX415_TCLKPREPARE, 0x0057},
    {IMX415_TCLKTRAIL, 0x005F},
    {IMX415_TCLKZERO, 0x0097},
    {IMX415_THSPREPARE, 0x005F},
    {IMX415_THSZERO, 0x00AF},
    {IMX415_THSTRAIL, 0x005F},
    {IMX415_THSEXIT, 0x009F},
    {IMX415_TLPX, 0x004F},
};

/* 2/2 line binning 4-lane 720 Mbps 30fps mode */
static const struct imx415_reg imx415_mode_4_720[] = {
    {IMX415_VMAX, 0x08CB},
    {IMX415_HMAX, 0x42A},
    {IMX415_LANEMODE, IMX415_LANEMODE_4},
    {IMX415_TCLKPOST, 0x006F},
    {IMX415_TCLKPREPARE, 0x002F},
    {IMX415_TCLKTRAIL, 0x002F},
    {IMX415_TCLKZERO, 0x00BF},
    {IMX415_THSPREPARE, 0x002F},
    {IMX415_THSZERO, 0x0057},
    {IMX415_THSTRAIL, 0x002F},
    {IMX415_THSEXIT, 0x004F},
    {IMX415_TLPX, 0x0027},
};

struct imx415_mode_reg_list {
  u32 num_of_regs;
  const struct imx415_reg *regs;
};

/*
 * Mode : number of lanes, lane rate and frame rate dependent settings
 *
 * pixel_rate and hmax_pix are needed to calculate hblank for the v4l2 ctrl
 * interface. These values can not be found in the data sheet and should be
 * treated as virtual values. Use following table when adding new modes.
 *
 * lane_rate  lanes    fps     hmax_pix   pixel_rate
 *
 *     594      2     10.000     4400       99000000
 *     891      2     15.000     4400      148500000
 *     720      2     15.748     4064      144000000
 *    1782      2     30.000     4400      297000000
 *    2079      2     30.000     4400      297000000
 *    1440      2     30.019     4510      304615385
 *
 *     594      4     20.000     5500      247500000
 *     594      4     25.000     4400      247500000
 *     720      4     25.000     4400      247500000
 *     720      4     30.019     4510      304615385
 *     891      4     30.000     4400      297000000
 *    1440      4     30.019     4510      304615385
 *    1440      4     60.038     4510      609230769
 *    1485      4     60.000     4400      594000000
 *    1782      4     60.000     4400      594000000
 *    2079      4     60.000     4400      594000000
 *    2376      4     90.164     4392      891000000
 */
struct imx415_mode {
  u64 lane_rate;
  u32 lanes;
  u32 hmax_pix;
  u64 pixel_rate;
  struct imx415_mode_reg_list reg_list;
};

/* mode configs */
static const struct imx415_mode supported_modes[] = {
    {
        .lane_rate = 720000000,
        .lanes = 2,
        .hmax_pix = 4064,
        .pixel_rate = 144000000,
        .reg_list =
            {
                .num_of_regs = ARRAY_SIZE(imx415_mode_2_720),
                .regs = imx415_mode_2_720,
            },
    },
    {
        .lane_rate = 1440000000,
        .lanes = 2,
        .hmax_pix = 4510,
        .pixel_rate = 304615385,
        .reg_list =
            {
                .num_of_regs = ARRAY_SIZE(imx415_mode_2_1440),
                .regs = imx415_mode_2_1440,
            },
    },
    {
        .lane_rate = 891000000,
        .lanes = 4,
        .hmax_pix = 4400,
        .pixel_rate = 297000000,
        .reg_list =
            {
                .num_of_regs = ARRAY_SIZE(imx415_mode_4_891),
                .regs = imx415_mode_4_891,
            },
    },

    {
        .lane_rate = 1440000000,
        .lanes = 4,
        .hmax_pix = 4510,
        .pixel_rate = 304615385,
        .reg_list =
            {
                .num_of_regs = ARRAY_SIZE(imx415_mode_4_1440),
                .regs = imx415_mode_4_1440,
            },
    },
    {
        .lane_rate = 1485000000,
        .lanes = 4,
        .hmax_pix = 4400,
        .pixel_rate = 148500000, // for 15 fps
        .reg_list =
            {
                .num_of_regs = ARRAY_SIZE(imx415_mode_4_1485),
                .regs = imx415_mode_4_1485,
            },
    },
    {
        .lane_rate = 720000000,
        .lanes = 4,
        .hmax_pix = 4510,
        .pixel_rate = 304615385, // 720@30FPS
        .reg_list =
            {
                .num_of_regs = ARRAY_SIZE(imx415_mode_4_720),
                .regs = imx415_mode_4_720,
            },
    },
    {
        .lane_rate = 720000000,
        .lanes = 4,
        .hmax_pix = 4400,
        .pixel_rate = 148500000, // 720@15FPS
        .reg_list =
            {
                .num_of_regs = ARRAY_SIZE(imx415_mode_4_720),
                .regs = imx415_mode_4_720,
            },
    },
};

static const char *const imx415_test_pattern_menu[] = {
    "disabled",
    "solid black",
    "solid white",
    "solid dark gray",
    "solid light gray",
    "stripes light/dark grey",
    "stripes dark/light grey",
    "stripes black/dark grey",
    "stripes dark grey/black",
    "stripes black/white",
    "stripes white/black",
    "horizontal color bar",
    "vertical color bar",
};
struct imx415_capture_properties {
  __u64 max_lane_frequency;
  __u64 max_pixel_frequency;
  __u64 max_data_rate;
};

struct imx415 {
  struct device *dev;
  struct clk *clk;
  struct regmap *regmap;

  const struct imx415_clk_params *clk_params;

  struct v4l2_subdev subdev;
  struct media_pad pads[IMX415_SENS_PADS_NUM];

  struct v4l2_ctrl_handler ctrls;
  struct v4l2_ctrl *vblank;
  struct v4l2_ctrl *hflip;
  struct v4l2_ctrl *vflip;
  struct v4l2_ctrl *exposure;
  struct mutex lock;
  vvcam_mode_info_t custom_cur_mode;
  struct imx415_capture_properties ocp;
  struct i2c_client *i2c_client;
  struct v4l2_mbus_framefmt format;
  unsigned int csi_id;
  unsigned int mclk;
  unsigned int mclk_source;
  u32 stream_status;
  u32 resume_status;
  unsigned int cur_mode;
  unsigned int num_data_lanes;
};

static struct vvcam_mode_info_s imx415_mode_info[] = {
    /* Mode 0: All pixel mode 891mbps iclk = 27MHz 30fps 12-bit */
    {
        .index = 0,
        .size =
            {
                .bounds_width = 1920,
                .bounds_height = 1080,
                .top = 0,
                .left = 0,
                .width = 1920,
                .height = 1080,
            },
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .data_compress =
            {
                .enable = 0,
            },
        .bayer_pattern = BAYER_GBRG,
        .ae_info =
            {
                .def_frm_len_lines = 0x8CA, /* VMAX = 2250 lines */
                .curr_frm_len_lines = 0x8CA,
                .one_line_exp_time_ns = 7500, /* 1H period at 60fps */

                .max_integration_line = 0x8CA - 8, // Integration time (lines) = (VMAX - SHR0)
                .min_integration_line = 4,

                .max_again =
                    30 * (1 << SENSOR_FIX_FRACBITS), /* 30dB analog gain */
                .min_again = 1 * (1 << SENSOR_FIX_FRACBITS),
                .max_dgain =
                    42 * (1 << SENSOR_FIX_FRACBITS), /* 42dB digital gain */
                .min_dgain = 1 * (1 << SENSOR_FIX_FRACBITS),
                .start_exposure = 3 * 400 * (1 << SENSOR_FIX_FRACBITS), // why
                .cur_fps = 60 * (1 << SENSOR_FIX_FRACBITS),
                .max_fps = 60 * (1 << SENSOR_FIX_FRACBITS),
                .min_fps = 5 * (1 << SENSOR_FIX_FRACBITS),
                .min_afps = 5 * (1 << SENSOR_FIX_FRACBITS),
                .int_update_delay_frm = 1,
                .gain_update_delay_frm = 1,
            },
        .mipi_info =
            {
                .mipi_lane = 4,
            },
        .preg_data = imx415_init_setting_1080p,
        .reg_data_count = ARRAY_SIZE(imx415_init_setting_1080p),
    },
    /*MODE:1*/
    {
        .index = 1,
        .size =
            {
                .bounds_width = 3840,
                .bounds_height = 2160,
                .top = 0,
                .left = 0,
                .width = 3840,
                .height = 2160,
            },
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .data_compress =
            {
                .enable = 0,
            },
        .bayer_pattern = BAYER_GBRG,
        .ae_info =
            {
                .def_frm_len_lines = 0x8CA, /* VMAX = 2250 lines */
                .curr_frm_len_lines = 0x8CA,
                .one_line_exp_time_ns = 14900, /* 1H period at 30fps */

                .max_integration_line =
                    0x8CA - 8, // Integration time (lines) = (VMAX - SHR0)
                .min_integration_line = 4,

                .max_again =
                    30 * (1 << SENSOR_FIX_FRACBITS), /* 30dB analog gain */
                .min_again = 1 * (1 << SENSOR_FIX_FRACBITS),
                .max_dgain =
                    42 * (1 << SENSOR_FIX_FRACBITS), /* 42dB digital gain */
                .min_dgain = 1 * (1 << SENSOR_FIX_FRACBITS),
                .start_exposure = 3 * 400 * (1 << SENSOR_FIX_FRACBITS), // why
                .cur_fps = 30 * (1 << SENSOR_FIX_FRACBITS),
                .max_fps = 30 * (1 << SENSOR_FIX_FRACBITS),
                .min_fps = 5 * (1 << SENSOR_FIX_FRACBITS),
                .min_afps = 5 * (1 << SENSOR_FIX_FRACBITS),
                .int_update_delay_frm = 1,
                .gain_update_delay_frm = 1,
            },
        .mipi_info =
            {
                .mipi_lane = 4,
            },
        .preg_data = IMX415_init_setting_4k_linear,
        .reg_data_count = ARRAY_SIZE(IMX415_init_setting_4k_linear),
    },
    /* Mode: 2*/
    {
        .index = 2,
        .size =
            {
                .bounds_width = 3840,
                .bounds_height = 2160,
                .top = 0,
                .left = 0,
                .width = 3840,
                .height = 2160,
            },
        .hdr_mode = SENSOR_MODE_HDR_STITCH,
        .stitching_mode = SENSOR_STITCHING_2DOL,
        .bit_width = 10,
        .data_compress =
            {
                .enable = 0,
            },
        .bayer_pattern = BAYER_GBRG,
        .ae_info =
            {
                .def_frm_len_lines = 0x8CA, /* VMAX = 2250 lines */
                .curr_frm_len_lines = 0x8CA,
                .one_line_exp_time_ns = 29700, /* 1H period at 15fps */

                .max_integration_line = 0x8CA - 8,
                .min_integration_line = 8,

                .max_again = 30 * (1 << SENSOR_FIX_FRACBITS),
                .min_again = 1 * (1 << SENSOR_FIX_FRACBITS),
                .max_dgain = 42 * (1 << SENSOR_FIX_FRACBITS),
                .min_dgain = 1 * (1 << SENSOR_FIX_FRACBITS),

                .start_exposure = 3 * 400 * (1 << SENSOR_FIX_FRACBITS),
                .cur_fps = 15 * (1 << SENSOR_FIX_FRACBITS),
                .max_fps = 15 * (1 << SENSOR_FIX_FRACBITS),
                .min_fps = 5 * (1 << SENSOR_FIX_FRACBITS),
                .min_afps = 5 * (1 << SENSOR_FIX_FRACBITS),
                .int_update_delay_frm = 1,
                .gain_update_delay_frm = 1,
            },
        .mipi_info =
            {
                .mipi_lane = 4,
            },
        .preg_data = imx415_init_setting_DOL_HDR,
        .reg_data_count = ARRAY_SIZE(imx415_init_setting_DOL_HDR),
    },
   
};

/*
 * This table includes fixed register settings and a bunch of undocumented
 * registers that have to be set to another value than default.
 */
static const struct imx415_reg imx415_init_table[] = {
	{ IMX415_MODE, 0x01 },
	{ IMX415_XMSTA, 0x01 },	
	
 
	{ IMX415_ADBIT, 0x00 },	
	{ IMX415_MDBIT, 0x00 },		
   	{ IMX415_SHR0, 0x08 }, 
   	{ IMX415_DRV, 0x00 }, // XVS_DRV[1:0]	- setting xvs and xhs as output in master mode	 	
   	   	
 /* Sony Magic Registers Start */
	{IMX415_REG_8BIT(0x32D4), 0x21},
	{IMX415_REG_8BIT(0x32EC), 0xA1},
	{IMX415_REG_8BIT(0x3452), 0x7F},
	{IMX415_REG_8BIT(0x3453), 0x03},
	{IMX415_REG_8BIT(0x358A), 0x04}, 
	{IMX415_REG_8BIT(0x35A1), 0x02}, 
	{IMX415_REG_8BIT(0x36BC), 0x0C},
	{IMX415_REG_8BIT(0x36CC), 0x53},
	{IMX415_REG_8BIT(0x36CD), 0x00},
	{IMX415_REG_8BIT(0x36CE), 0x3C}, 
	{IMX415_REG_8BIT(0x36D0), 0x8C}, 
	{IMX415_REG_8BIT(0x36D1), 0x00},
	{IMX415_REG_8BIT(0x36D2), 0x71}, 
	{IMX415_REG_8BIT(0x36D4), 0x3C}, 
	{IMX415_REG_8BIT(0x36D6), 0x53},
	{IMX415_REG_8BIT(0x36D7), 0x00},
	{IMX415_REG_8BIT(0x36D8), 0x71},
	{IMX415_REG_8BIT(0x36DA), 0x8C},
	{IMX415_REG_8BIT(0x36DB), 0x00},
	{IMX415_REG_8BIT(0x3701), 0x00},	// ADBIT1[7:0]
	{IMX415_REG_8BIT(0x3724), 0x02}, 
	{IMX415_REG_8BIT(0x3726), 0x02},
	{IMX415_REG_8BIT(0x3732), 0x02},
	{IMX415_REG_8BIT(0x3734), 0x03},
	{IMX415_REG_8BIT(0x3736), 0x03},
	{IMX415_REG_8BIT(0x3742), 0x03},
	{IMX415_REG_8BIT(0x3862), 0xE0},
	{IMX415_REG_8BIT(0x38CC), 0x30},
	{IMX415_REG_8BIT(0x38CD), 0x2F}, 
	{IMX415_REG_8BIT(0x395C), 0x0C}, 
	{IMX415_REG_8BIT(0x3A42), 0xD1},
	{IMX415_REG_8BIT(0x3A4C), 0x77},
	{IMX415_REG_8BIT(0x3AE0), 0x02},  
	{IMX415_REG_8BIT(0x3AEC), 0x0C},
	{IMX415_REG_8BIT(0x3B00), 0x2E},
	{IMX415_REG_8BIT(0x3B06), 0x29},
	{IMX415_REG_8BIT(0x3B98), 0x25},
	{IMX415_REG_8BIT(0x3B99), 0x21},
	{IMX415_REG_8BIT(0x3B9B), 0x13},
	{IMX415_REG_8BIT(0x3B9C), 0x13},
	{IMX415_REG_8BIT(0x3B9D), 0x13},
	{IMX415_REG_8BIT(0x3B9E), 0x13},
	{IMX415_REG_8BIT(0x3BA1), 0x00},
	{IMX415_REG_8BIT(0x3BA2), 0x06},
	{IMX415_REG_8BIT(0x3BA3), 0x0B},
	{IMX415_REG_8BIT(0x3BA4), 0x10},
	{IMX415_REG_8BIT(0x3BA5), 0x14},
	{IMX415_REG_8BIT(0x3BA6), 0x18},
	{IMX415_REG_8BIT(0x3BA7), 0x1A},  
	{IMX415_REG_8BIT(0x3BA8), 0x1A},
	{IMX415_REG_8BIT(0x3BA9), 0x1A},
	{IMX415_REG_8BIT(0x3BAC), 0xED},
	{IMX415_REG_8BIT(0x3BAD), 0x01},
	{IMX415_REG_8BIT(0x3BAE), 0xF6},
	{IMX415_REG_8BIT(0x3BAF), 0x02},
	{IMX415_REG_8BIT(0x3BB0), 0xA2},
	{IMX415_REG_8BIT(0x3BB1), 0x03},
	{IMX415_REG_8BIT(0x3BB2), 0xE0}, 
	{IMX415_REG_8BIT(0x3BB3), 0x03}, 	
	{IMX415_REG_8BIT(0x3BB4), 0xE0},
	{IMX415_REG_8BIT(0x3BB5), 0x03},
	{IMX415_REG_8BIT(0x3BB6), 0xE0},
	{IMX415_REG_8BIT(0x3BB7), 0x03},
	{IMX415_REG_8BIT(0x3BB8), 0xE0},
	{IMX415_REG_8BIT(0x3BBA), 0xE0}, 
	{IMX415_REG_8BIT(0x3BBC), 0xDA},
	{IMX415_REG_8BIT(0x3BBE), 0x88},
	{IMX415_REG_8BIT(0x3BC0), 0x44},
	{IMX415_REG_8BIT(0x3BC2), 0x7B},
	{IMX415_REG_8BIT(0x3BC4), 0xA2},
	{IMX415_REG_8BIT(0x3BC8), 0xBD},
	{IMX415_REG_8BIT(0x3BCA), 0xBD},
/* Sony Magic Registers End */	

	{ IMX415_MODE, 0x00 },
	{ IMX415_XMSTA, 0x00 },	
	
};

static inline struct imx415 *to_imx415(struct v4l2_subdev *sd) {
  return container_of(sd, struct imx415, subdev);
}

/* Regmap-based read function (from kernel 6.6 API) */
static int imx415_regmap_read(struct imx415 *sensor, u32 addr) {
  u8 data[3] = {0};
  int ret;

  ret = regmap_raw_read(sensor->regmap, addr & IMX415_REG_ADDR_MASK, data,
                        (addr >> IMX415_REG_SIZE_SHIFT) & 3);
  if (ret < 0)
    return ret;

  return (data[2] << 16) | (data[1] << 8) | data[0];
}

/* Regmap-based write function (from kernel 6.6 API) */
static int imx415_regmap_write(struct imx415 *sensor, u32 addr, u32 value) {
  u8 data[3] = {value & 0xff, (value >> 8) & 0xff, value >> 16};
  int ret;

  ret = regmap_raw_write(sensor->regmap, addr & IMX415_REG_ADDR_MASK, data,
                         (addr >> IMX415_REG_SIZE_SHIFT) & 3);
  if (ret < 0)
    dev_err_ratelimited(sensor->dev, "%u-bit write to 0x%04x failed: %d\n",
                        ((addr >> IMX415_REG_SIZE_SHIFT) & 3) * 8,
                        addr & IMX415_REG_ADDR_MASK, ret);

  return ret;
}

/* Write multiple registers at once */
/*
static int imx415_multi_reg_write(struct imx415 *sensor,
                                  const struct imx415_reg *regs,
                                  unsigned int num_regs) {
  unsigned int i;
  int ret = 0;

  for (i = 0; i < num_regs; i++) {
    ret = imx415_regmap_write(sensor, regs[i].address, regs[i].val);
    if (ret)
      return ret;
  }

  return 0;
}
*/
static int imx415_set_testpattern(struct imx415 *sensor, int val) {
  dev_info(sensor->dev, "Enter imx415_set_testpattern\n");
  int ret = 0;

  if (val) {
    dev_info(sensor->dev, "Enable imx415_set_testpattern\n");
    ret = imx415_regmap_write(sensor, IMX415_BLKLEVEL, 0x00);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_TPG_EN_DUOUT, 0x01);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_TPG_PATSEL_DUOUT, val - 1);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_TPG_COLORWIDTH, 0x01);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_TESTCLKEN_MIPI, 0x20);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_DIG_CLP_MODE, 0x00);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_WRJ_OPEN, 0x00);
    if (ret)
      return ret;
  } else {
    ret = imx415_regmap_write(sensor, IMX415_BLKLEVEL, IMX415_BLKLEVEL_DEFAULT);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_TPG_EN_DUOUT, 0x00);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_TESTCLKEN_MIPI, 0x00);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_DIG_CLP_MODE, 0x01);
    if (ret)
      return ret;
    ret = imx415_regmap_write(sensor, IMX415_WRJ_OPEN, 0x01);
    if (ret)
      return ret;
  }
  return 0;
}

static int imx415_s_ctrl(struct v4l2_ctrl *ctrl) {
  printk("Enter imx415_s_ctrl\n");
  struct imx415 *sensor = container_of(ctrl->handler, struct imx415, ctrls);
  
  int ret;

  if (!pm_runtime_get_if_in_use(sensor->dev))
    return 0;

  pm_runtime_put(sensor->dev);

  return ret;
}

static const struct v4l2_ctrl_ops imx415_ctrl_ops = {
    .s_ctrl = imx415_s_ctrl,
};

static int imx415_ctrls_init(struct imx415 *sensor) {
  dev_info(sensor->dev, "Enter imx415_ctrls_init\n");
  struct v4l2_fwnode_device_properties props;
  struct v4l2_ctrl *ctrl;
  u64 pixel_rate = supported_modes[sensor->cur_mode].pixel_rate;
  u64 lane_rate = supported_modes[sensor->cur_mode].lane_rate;
  u32 exposure_max = IMX415_PIXEL_ARRAY_HEIGHT + IMX415_PIXEL_ARRAY_VBLANK - 8;
  u32 hblank;
  unsigned int i;
  int ret;

  ret = v4l2_fwnode_device_parse(sensor->dev, &props);
  if (ret < 0)
    return ret;

  v4l2_ctrl_handler_init(&sensor->ctrls, 10);

  for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); ++i) {
    if (lane_rate == link_freq_menu_items[i] * 2)
      break;
  }
  if (i == ARRAY_SIZE(link_freq_menu_items)) {
    return dev_err_probe(sensor->dev, -EINVAL, "lane rate %llu not supported\n",
                         lane_rate);
  }

  ctrl = v4l2_ctrl_new_int_menu(
      &sensor->ctrls, &imx415_ctrl_ops, V4L2_CID_LINK_FREQ,
      ARRAY_SIZE(link_freq_menu_items) - 1, i, link_freq_menu_items);

  if (ctrl)
    ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

  v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops, V4L2_CID_EXPOSURE, 4,
                    exposure_max, 1, exposure_max);

  v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                    IMX415_AGAIN_MIN, IMX415_AGAIN_MAX, IMX415_AGAIN_STEP,
                    IMX415_AGAIN_MIN);

  hblank =
      supported_modes[sensor->cur_mode].hmax_pix - IMX415_PIXEL_ARRAY_WIDTH;
  ctrl = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops, V4L2_CID_HBLANK,
                           hblank, hblank, 1, hblank);
  if (ctrl)
    ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

  sensor->vblank =
      v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops, V4L2_CID_VBLANK,
                        IMX415_PIXEL_ARRAY_VBLANK, IMX415_PIXEL_ARRAY_VBLANK, 1,
                        IMX415_PIXEL_ARRAY_VBLANK);
  if (sensor->vblank)
    sensor->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

  v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE, pixel_rate,
                    pixel_rate, 1, pixel_rate);

  sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
                                    V4L2_CID_HFLIP, 0, 1, 1, 0);
  sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
                                    V4L2_CID_VFLIP, 0, 1, 1, 0);

  v4l2_ctrl_new_std_menu_items(
      &sensor->ctrls, &imx415_ctrl_ops, V4L2_CID_TEST_PATTERN,
      ARRAY_SIZE(imx415_test_pattern_menu) - 1, 0, 0, imx415_test_pattern_menu);

  v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &imx415_ctrl_ops, &props);

  if (sensor->ctrls.error) {
    dev_err_probe(sensor->dev, sensor->ctrls.error, "failed to add controls\n");
    v4l2_ctrl_handler_free(&sensor->ctrls);
    return sensor->ctrls.error;
  }
  sensor->subdev.ctrl_handler = &sensor->ctrls;

  return 0;
}

static int imx415_wakeup(struct imx415 *sensor) {
  dev_info(sensor->dev, "Enter imx415_wakeup\n");
  int ret;

  ret = imx415_regmap_write(sensor, IMX415_MODE, IMX415_MODE_OPERATING);
  if (ret) {
    dev_err(sensor->dev, "Error setting sensor in operating mode\n");
    return ret;
  }
  
  msleep(80);

  return 0;
}


static int imx415_read_reg(struct imx415 *sensor, u16 reg, u8 *val) {
  struct device *dev = &sensor->i2c_client->dev;
  struct i2c_client *client = sensor->i2c_client;
  struct i2c_msg msg[2];
  u8 buf[2];
  int ret;

  /* First message: write register address */
  buf[0] = reg >> 8;
  buf[1] = reg & 0xff;

  msg[0].addr = client->addr;
  msg[0].flags = 0; /* Write */
  msg[0].len = 2;
  msg[0].buf = buf;

  /* Second message: read register value */
  msg[1].addr = client->addr;
  msg[1].flags = I2C_M_RD; /* Read */
  msg[1].len = 1;
  msg[1].buf = val;

  ret = i2c_transfer(client->adapter, msg, 2);
  if (ret < 0) {
    dev_err(dev, "Read reg error: reg=0x%x, ret=%d\n", reg, ret);
    return ret;
  }

  if (ret != 2) {
    dev_err(dev, "Read reg incomplete: reg=0x%x, ret=%d\n", reg, ret);
    return -EIO;
  }

  dev_info(dev, "REG[0x%04x] = 0x%02x\n", reg, *val);

  return 0;
}

static int imx415_read_reg_arry(struct imx415 *sensor,
                                struct vvcam_sccb_data_s *reg_arry, u32 size) {
  int i;
  int ret = 0;
  u8 val;

  if (!reg_arry || size == 0) {
    dev_err(sensor->dev, "Invalid register array\n");
    return -EINVAL;
  }

  // Read registers one by one
  for (i = 0; i < size; i++) {
    ret = imx415_read_reg(sensor, reg_arry[i].addr, &val);
    if (ret < 0) {
      dev_err(sensor->dev, "Failed to read reg 0x%x\n", reg_arry[i].addr);
      return ret;
    }
    reg_arry[i].data = val;
  }

  return 0;
}

static int imx415_stream_on(struct imx415 *sensor) {
  int ret;
  u64 reg_val;

  dev_info(sensor->dev, "Starting sensor stream...\n");

  ret = imx415_regmap_write(sensor, IMX415_MODE, IMX415_MODE_OPERATING);
  if (ret) {
    dev_err(sensor->dev, "Error setting sensor in operating mode\n");
    return ret;
  }

  // Start streaming
  ret = imx415_regmap_write(sensor, IMX415_XMSTA, IMX415_XMSTA_START);
  if (ret) {
    dev_err(sensor->dev, "Failed to start streaming: %d\n", ret);
    return ret;
  }

  // Verify streaming started
  reg_val = imx415_regmap_read(sensor, IMX415_XMSTA);
  dev_info(sensor->dev, "XMSTA register = 0x%llx (should be 0x00)\n", reg_val);

  reg_val = imx415_regmap_read(sensor, IMX415_MODE);
  dev_info(sensor->dev, "MODE register = 0x%llx (should be 0x00 = operating)\n",
           reg_val);

  // re-check sensor register values
  ret = imx415_read_reg_arry(
      sensor, (struct vvcam_sccb_data_s *)sensor->custom_cur_mode.preg_data,
      sensor->custom_cur_mode.reg_data_count);

  if (ret < 0) {
    printk("%s:imx415_read_reg_arry error\n", __func__);
  }

  return 0;
}

static int imx415_stream_off(struct imx415 *sensor) {
  dev_info(sensor->dev, "Enter imx415_stream_off\n");
  int ret;

  ret = imx415_regmap_write(sensor, IMX415_XMSTA, IMX415_XMSTA_STOP);
  return imx415_regmap_write(sensor, IMX415_MODE, IMX415_MODE_STANDBY);
}

static int imx415_write_reg(struct imx415 *sensor, u16 reg, u8 val) {
  struct device *dev = &sensor->i2c_client->dev;
  struct i2c_client *client = sensor->i2c_client;
  u8 buf[3] = {0};

  u8 readback_val = 0;
  int ret;

  buf[0] = reg >> 8;
  buf[1] = reg & 0xff;
  buf[2] = val;

  if (i2c_master_send(client, buf, 3) < 0) {
    dev_err(dev, "Write reg error: reg=0x%x, val=0x%x\n", reg, val);
    return -EIO;
  }

  /* Read back the value to verify */
  ret = imx415_read_reg(sensor, reg, &readback_val);
  if (ret == 0) {
    dev_info(dev, "Reg 0x%04x: wrote 0x%02x, read back 0x%02x\n", reg, val,
             readback_val);
  } else {
    dev_err(dev, "Failed to read back reg 0x%04x after write\n", reg);
  }

  return 0;
}

// TO BE ADDED IF WE WANT TO WRITE REGISTER SEQUENCE FROM HEADER FILE...

static int imx415_write_reg_arry(struct imx415 *sensor,
                                  struct vvcam_sccb_data_s *reg_arry,
                                  u32 size)
{
    int i;
    int ret = 0;

    if (!reg_arry || size == 0) {
        dev_err(sensor->dev, "Invalid register array\n");
        return -EINVAL;
    }

    // Write registers one by one - simpler and more reliable
    for (i = 0; i < size; i++) {
        ret = imx415_write_reg(sensor, reg_arry[i].addr, reg_arry[i].data);
        if (ret < 0) {
            dev_err(sensor->dev, "Failed to write reg 0x%x = 0x%x\n",
                    reg_arry[i].addr, reg_arry[i].data);
            return ret;
        }
    }

    return 0;
}


static int imx415_s_stream(struct v4l2_subdev *sd, int enable) {
  msleep(200);
  printk("%s (enter) enable = %d\n", __func__, enable);
  struct imx415 *sensor = to_imx415(sd);

  int ret;
  
  if (!enable) {
    ret = imx415_stream_off(sensor);

    pm_runtime_put_autosuspend(sensor->dev);

    goto unlock;
  }

  ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
  if (ret < 0)
    goto err_pm;

  ret = imx415_stream_on(sensor);
  if (ret)
    goto err_pm;

  ret = 0;

  sensor->stream_status = enable;
  dev_info(sensor->dev, "Stream enable success!\n");
unlock:

  return ret;

err_pm:
  /*
   * In case of error, turn the power off synchronously as the device
   * likely has no other chance to recover.
   */
  pm_runtime_put_sync(sensor->dev);

  goto unlock;
}

static int imx415_enum_frame_size(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_state *state,
                                  struct v4l2_subdev_frame_size_enum *fse) {
  printk("Enter imx415_enum_frame_size\n");
  const struct v4l2_mbus_framefmt *format;

  format = v4l2_subdev_get_pad_format(sd, state, fse->pad);

  if (fse->index > 0 || fse->code != format->code)
    return -EINVAL;

  fse->min_width = IMX415_PIXEL_ARRAY_WIDTH;
  fse->max_width = fse->min_width;
  fse->min_height = IMX415_PIXEL_ARRAY_HEIGHT;
  fse->max_height = fse->min_height;
  return 0;
}

static int imx415_get_format_code(struct imx415 *sensor, u32 *code) {
  switch (sensor->custom_cur_mode.bayer_pattern) {
  case BAYER_RGGB:
    if (sensor->custom_cur_mode.bit_width == 8) {
      *code = MEDIA_BUS_FMT_SRGGB8_1X8;
    } else if (sensor->custom_cur_mode.bit_width == 10) {
      *code = MEDIA_BUS_FMT_SRGGB10_1X10;
    } else {
      *code = MEDIA_BUS_FMT_SRGGB12_1X12;
    }
    break;
  case BAYER_GRBG:
    if (sensor->custom_cur_mode.bit_width == 8) {
      *code = MEDIA_BUS_FMT_SGRBG8_1X8;
    } else if (sensor->custom_cur_mode.bit_width == 10) {
      *code = MEDIA_BUS_FMT_SGRBG10_1X10;
    } else {
      *code = MEDIA_BUS_FMT_SGRBG12_1X12;
    }
    break;
  case BAYER_GBRG:
    if (sensor->custom_cur_mode.bit_width == 8) {
      *code = MEDIA_BUS_FMT_SGBRG8_1X8;
    } else if (sensor->custom_cur_mode.bit_width == 10) {
      *code = MEDIA_BUS_FMT_SGBRG10_1X10;
    } else {
      *code = MEDIA_BUS_FMT_SGBRG12_1X12;
    }
    break;
  case BAYER_BGGR:
    if (sensor->custom_cur_mode.bit_width == 8) {
      *code = MEDIA_BUS_FMT_SBGGR8_1X8;
    } else if (sensor->custom_cur_mode.bit_width == 10) {
      *code = MEDIA_BUS_FMT_SBGGR10_1X10;
    } else {
      *code = MEDIA_BUS_FMT_SBGGR12_1X12;
    }
    break;
  default:
    /*nothing need to do*/
    break;
  }
  return 0;
}

static int imx415_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code) {
  printk("Enter imx415_enum_mbus_code\n");
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct imx415 *sensor = client_to_imx415(client);

  u32 cur_code = MEDIA_BUS_FMT_SGBRG10_1X10;

  if (code->index > 0)
    return -EINVAL;

  imx415_get_format_code(sensor, &cur_code);

  code->code = cur_code;

  return 0;
}

static int imx415_set_format(struct v4l2_subdev *sd,
                             struct v4l2_subdev_state *state,
                             struct v4l2_subdev_format *fmt) {
  printk("Enter imx415_set_format\n");
  int ret = 0;
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  
  struct imx415 *sensor = client_to_imx415(client);
  
  mutex_lock(&sensor->lock);
  
  if ((fmt->format.width != sensor->custom_cur_mode.size.bounds_width) ||
	    (fmt->format.height != sensor->custom_cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
			__func__,fmt->format.width,fmt->format.height);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}
  
  ret = imx415_regmap_write(sensor, IMX415_MODE, IMX415_MODE_STANDBY);
  if (ret) {
    dev_err(sensor->dev, "Error setting sensor in operating mode\n");
    return ret;
  }

  ret = imx415_write_reg_arry(sensor,
		(struct vvcam_sccb_data_s *)sensor->custom_cur_mode.preg_data,
		sensor->custom_cur_mode.reg_data_count);
	if (ret < 0) {
		pr_err("%s:imx415_write_reg_arry error\n",__func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}
	
  imx415_get_format_code(sensor, &fmt->format.code);
  
  struct v4l2_mbus_framefmt *format;
	
  format = v4l2_subdev_get_pad_format(sd, state, fmt->pad);

  format->width = fmt->format.width;
  format->height = fmt->format.height;
  format->code = MEDIA_BUS_FMT_SGBRG10_1X10;
  format->field = V4L2_FIELD_NONE;
  format->colorspace = V4L2_COLORSPACE_RAW;
  format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
  format->quantization = V4L2_QUANTIZATION_DEFAULT;
  format->xfer_func = V4L2_XFER_FUNC_NONE;

  fmt->format = *format;
  mutex_unlock(&sensor->lock);

  printk("IMX415 set_fmt: code=0x%x width=%d height=%d\n", format->code,
         fmt->format.width, fmt->format.height);

  return 0;
}

static int imx415_get_format(struct v4l2_subdev *sd,
                             struct v4l2_subdev_state *state,
                             struct v4l2_subdev_format *fmt) {

  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct imx415 *sensor = client_to_imx415(client);
  dev_err(sensor->dev, "Enter imx415_get_format\n");

  mutex_lock(&sensor->lock);
  fmt->format = sensor->format;
  mutex_unlock(&sensor->lock);
  return 0;
}

static int imx415_get_selection(struct v4l2_subdev *sd,
                                struct v4l2_subdev_state *sd_state,
                                struct v4l2_subdev_selection *sel) {
  printk("Enter imx415_get_selection\n");
  switch (sel->target) {
  case V4L2_SEL_TGT_CROP:
  case V4L2_SEL_TGT_CROP_DEFAULT:
  case V4L2_SEL_TGT_CROP_BOUNDS:
    sel->r.top = IMX415_PIXEL_ARRAY_TOP;
    sel->r.left = IMX415_PIXEL_ARRAY_LEFT;
    sel->r.width = IMX415_PIXEL_ARRAY_WIDTH;
    sel->r.height = IMX415_PIXEL_ARRAY_HEIGHT;

    return 0;
  }

  return -EINVAL;
}

static int imx415_link_setup(struct media_entity *entity,
                             const struct media_pad *local,
                             const struct media_pad *remote, u32 flags) {
  return 0;
}

// NOT SURE WHETHER TO ADD OR NOT
static const struct media_entity_operations imx415_sd_media_ops = {
    .link_setup = imx415_link_setup,
};

static const struct v4l2_subdev_video_ops imx415_subdev_video_ops = {
    .s_stream = imx415_s_stream,
};

static const struct v4l2_subdev_pad_ops imx415_subdev_pad_ops = {
    .enum_mbus_code = imx415_enum_mbus_code,
    .enum_frame_size = imx415_enum_frame_size,
    .get_fmt = imx415_get_format,
    .set_fmt = imx415_set_format,
    .get_selection = imx415_get_selection,
};

static int imx415_s_power(struct v4l2_subdev *sd, int on) {

  printk("enter %s\n", __func__);

  return 0;
}

static int imx415_get_clk(struct imx415 *sensor, void *clk) {
  struct vvcam_clk_s vvcam_clk;
  int ret = 0;
  vvcam_clk.sensor_mclk = clk_get_rate(sensor->clk);
  vvcam_clk.csi_max_pixel_clk = sensor->ocp.max_pixel_frequency;

  dev_info(sensor->dev, "sensor_mclk = %lu Hz, csi_max_pixel_clk = %lu Hz\n",
           vvcam_clk.sensor_mclk, vvcam_clk.csi_max_pixel_clk);

  ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
  if (ret != 0)
    ret = -EINVAL;
  return ret;
}

static int imx415_query_capability(struct imx415 *sensor, void *arg) {
  if (!sensor || !arg) {
    printk("Sensor struct or arg is NULL\n");
    return -EINVAL;
  }

  struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

  strcpy((char *)pcap->driver, "imx415");
  sprintf((char *)pcap->bus_info, "csi%d", sensor->csi_id);
  if (sensor->i2c_client->adapter) {
    pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
        (__u8)sensor->i2c_client->adapter->nr;
  } else {
    pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
  }

  return 0;
}

static int imx415_query_supports(struct imx415 *sensor, void *parry) {
  struct vvcam_mode_info_array_s *psensor_mode_arry = parry;

  psensor_mode_arry->count = ARRAY_SIZE(imx415_mode_info);
  memcpy((void *)&psensor_mode_arry->modes, (void *)imx415_mode_info,
         sizeof(imx415_mode_info));

  return 0;
}

static int imx415_get_sensor_mode(struct imx415 *sensor, void *pmode) {
  int ret = 0;
  
  vvcam_mode_info_t *mode;

	/* Point to current mode stored in sensor */
	mode = &sensor->custom_cur_mode;

	/* Read fields directly */
	pr_info("IMX415 current mode info:\n");
	pr_info(" index            : %u\n", mode->index);
	pr_info(" bounds            : %ux%u\n",mode->size.bounds_width,mode->size.bounds_height);
	pr_info(" active size       : %ux%u\n",mode->size.width,mode->size.height);
	pr_info(" crop (top,left)   : (%u,%u)\n",mode->size.top,mode->size.left);
	pr_info(" hdr_mode          : %u\n", mode->hdr_mode);
	pr_info(" stitching_mode    : %u\n", mode->stitching_mode);
	pr_info(" bit_width         : %u\n", mode->bit_width);
	pr_info(" bayer_pattern     : %u\n", mode->bayer_pattern);
	pr_info(" reg_data_count    : %u\n", mode->reg_data_count);

	/* Example: read MIPI info */
	pr_info(" mipi lanes        : %u\n", mode->mipi_info.mipi_lane);
  
  ret = copy_to_user(pmode, &sensor->custom_cur_mode,
                     sizeof(struct vvcam_mode_info_s));
  if (ret != 0) {
    printk("No memory defined\n");
    ret = -ENOMEM;
  }
  return ret;
}

static int imx415_set_sensor_mode(struct imx415 *sensor, void *pmode) {
  int ret = 0;
  int i = 0;
  struct vvcam_mode_info_s sensor_mode;
  ret = copy_from_user(&sensor_mode, pmode, sizeof(struct vvcam_mode_info_s));
  if (ret != 0)
    return -ENOMEM;
  for (i = 0; i < ARRAY_SIZE(imx415_mode_info); i++) {
    if (imx415_mode_info[i].index == sensor_mode.index) {
      dev_err(sensor->dev, "Enter imx415_set_sensor_mode mode is %u\n",
              imx415_mode_info[i].index);
      memcpy(&sensor->custom_cur_mode, &imx415_mode_info[i],
             sizeof(struct vvcam_mode_info_s));
      return 0;
    }
  }
  dev_err(sensor->dev, "No sensor mode defined\n");
  return -ENXIO;
}

static int imx415_get_fps(struct imx415 *sensor, u32 *pfps) {
  *pfps = sensor->custom_cur_mode.ae_info.cur_fps;
  return 0;
}

static int imx415_get_sensor_id(struct imx415 *sensor, void *pchip_id) {
  int ret;
  u64 chip_id;

  /*
   * While most registers can be read when the sensor is in standby, this
   * is not the case of the sensor info register :-(
   */
  ret = imx415_wakeup(sensor);
  if (ret)
    return dev_err_probe(sensor->dev, ret,
                         "failed to get sensor out of standby\n");

  ret = imx415_regmap_read(sensor, IMX415_SENSOR_INFO);
  if (ret < 0) {
    dev_err_probe(sensor->dev, ret, "failed to read sensor information\n");
    return ret;
  }
  chip_id = ret;

  chip_id = chip_id & IMX415_SENSOR_INFO_MASK;

  if (chip_id != IMX415_CHIP_ID) {
    dev_err(sensor->dev, "Failed to read sensor ID : %llu\n", chip_id);
  }

  ret = copy_to_user(pchip_id, &chip_id, sizeof(u32));
  if (ret != 0)
    ret = -ENOMEM;
  return ret;
}

static long imx415_priv_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
                              void *arg_user) {

  struct i2c_client *client = v4l2_get_subdevdata(sd);
  struct imx415 *sensor = client_to_imx415(client);
  dev_err(sensor->dev, "Enter imx415_priv_ioctl\n");
  long ret = 0;
  void *arg = arg_user;
  struct vvcam_sccb_data_s sensor_reg;

  if (!sensor) {
    dev_err(sensor->dev, "struct imx415 is NULL\n");
    return -EINVAL;
  }
  mutex_lock(&sensor->lock);
  switch (cmd) {
  case VVSENSORIOC_S_POWER:
    dev_err(sensor->dev, "Inside VVSENSORIOC_S_POWER\n");
    ret = 0;
    break;
  case VVSENSORIOC_S_CLK:
    dev_err(sensor->dev, "Inside VVSENSORIOC_S_CLK\n");
    ret = 0;
    break;
  case VVSENSORIOC_G_CLK:
    dev_err(sensor->dev, "Inside VVSENSORIOC_G_CLK\n");
    ret = imx415_get_clk(sensor, arg);
    break;
  case VVSENSORIOC_RESET:
    dev_err(sensor->dev, "Inside VVSENSORIOC_RESET\n");
    ret = 0;
    break;
  case VIDIOC_QUERYCAP:
    dev_err(sensor->dev, "Inside VIDIOC_QUERYCAP\n");
    ret = imx415_query_capability(sensor, arg);
    if (ret != 0) {
      dev_err(sensor->dev, "RET value is not 0\n");
    }
    break;
  case VVSENSORIOC_QUERY:
    dev_err(sensor->dev, "Inside VVSENSORIOC_QUERY\n");
    USER_TO_KERNEL_VMALLOC(struct vvcam_mode_info_array_s);
    ret = imx415_query_supports(sensor, arg);
    KERNEL_TO_USER_VMALLOC(struct vvcam_mode_info_array_s);
    break;
  case VVSENSORIOC_G_CHIP_ID:
    dev_err(sensor->dev, "Inside VVSENSORIOC_G_CHIP_ID\n");
    ret = imx415_get_sensor_id(sensor, arg);
    if (ret != 0) {
      dev_err(sensor->dev, "Failed to read chip ID\n");
    }
    break;
  case VVSENSORIOC_G_SENSOR_MODE:
    dev_err(sensor->dev, "Inside VVSENSORIOC_G_SENSOR_MODE\n");
    ret = imx415_get_sensor_mode(sensor, arg);
    break;
  case VVSENSORIOC_S_SENSOR_MODE:
    dev_err(sensor->dev, "Inside VVSENSORIOC_S_SENSOR_MODE\n");
    ret = imx415_set_sensor_mode(sensor, arg);
    break;
  case VVSENSORIOC_S_STREAM:
    dev_err(sensor->dev, "Inside VVSENSORIOC_S_STREAM\n");
    USER_TO_KERNEL(int);
    ret = imx415_s_stream(&sensor->subdev, *(int *)arg);
    break;
  case VVSENSORIOC_WRITE_REG:
    dev_err(sensor->dev, "Inside VVSENSORIOC_WRITE_REG\n");
    ret = copy_from_user(&sensor_reg, arg, sizeof(struct vvcam_sccb_data_s));
    ret |= imx415_write_reg(sensor, sensor_reg.addr, sensor_reg.data);
    break;
  case VVSENSORIOC_READ_REG:
    dev_err(sensor->dev, "Inside VVSENSORIOC_READ_REG\n");
    ret = copy_from_user(&sensor_reg, arg, sizeof(struct vvcam_sccb_data_s));
    ret |= imx415_read_reg(sensor, sensor_reg.addr, (u8 *)&sensor_reg.data);
    ret |= copy_to_user(arg, &sensor_reg, sizeof(struct vvcam_sccb_data_s));
    break;
  case VVSENSORIOC_G_FPS:
    dev_err(sensor->dev, "Inside VVSENSORIOC_G_FPS\n");
    USER_TO_KERNEL(u32);
    ret = imx415_get_fps(sensor, (u32 *)arg);
    KERNEL_TO_USER(u32);
    break;
  case VVSENSORIOC_S_FPS:
    dev_err(sensor->dev, "Inside VVSENSORIOC_S_FPS\n");
    USER_TO_KERNEL(u32);
    // ret = imx415_set_fps(sensor, *(u32 *)arg);
    break;
  case VVSENSORIOC_S_TEST_PATTERN:
    dev_err(sensor->dev, "Inside VVSENSORIOC_S_TEST_PATTERN\n");
    ret = imx415_set_testpattern(sensor, *(int *)arg);
    break;
  default:
    ret = -EINVAL;
    break;
  }

  mutex_unlock(&sensor->lock);
  return ret;
}

static struct v4l2_subdev_core_ops imx415_subdev_core_ops = {
    .s_power = imx415_s_power,
    .ioctl = imx415_priv_ioctl,
};

static const struct v4l2_subdev_ops imx415_subdev_ops = {
    .core = &imx415_subdev_core_ops,
    .video = &imx415_subdev_video_ops,
    .pad = &imx415_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx415_internal_ops = {
    /* .init_state not available in 6.6 - use init_cfg instead */
};

static int imx415_subdev_init(struct imx415 *sensor) {
  struct i2c_client *client = to_i2c_client(sensor->dev);
  int ret;
  struct v4l2_subdev *sd;
  sd = &sensor->subdev;
  v4l2_i2c_subdev_init(sd, client, &imx415_subdev_ops);
  sensor->subdev.internal_ops = &imx415_internal_ops;

  ret = imx415_ctrls_init(sensor);
  if (ret)
    return ret;

  sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
  sd->dev = &client->dev;
  sd->entity.ops = &imx415_sd_media_ops,
  sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
  sensor->pads[IMX415_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
  ret = media_entity_pads_init(&sd->entity, IMX415_SENS_PADS_NUM, sensor->pads);
  if (ret < 0) {
    v4l2_ctrl_handler_free(&sensor->ctrls);
    return ret;
  }

  sensor->subdev.state_lock = sensor->subdev.ctrl_handler->lock;
  v4l2_subdev_init_finalize(&sensor->subdev);

  return 0;
}

static void imx415_subdev_cleanup(struct imx415 *sensor) {
  dev_info(sensor->dev, "Enter imx415_subdev_cleanup\n");
  media_entity_cleanup(&sensor->subdev.entity);
  v4l2_ctrl_handler_free(&sensor->ctrls);
}

static int imx415_identify_model(struct imx415 *sensor) {
  int model, ret;
  u64 chip_id;

  /*
   * While most registers can be read when the sensor is in standby, this
   * is not the case of the sensor info register :-(
   */
  ret = imx415_wakeup(sensor);
  if (ret)
    return dev_err_probe(sensor->dev, ret,
                         "failed to get sensor out of standby\n");

  ret = imx415_regmap_read(sensor, IMX415_SENSOR_INFO);
  if (ret < 0) {
    dev_err_probe(sensor->dev, ret, "failed to read sensor information\n");
    goto done;
  }
  chip_id = ret;

  model = chip_id & IMX415_SENSOR_INFO_MASK;

  switch (model) {
  case IMX415_CHIP_ID:
    dev_info(sensor->dev, "Detected IMX415 image sensor\n");
    break;
  default:
    ret = dev_err_probe(sensor->dev, -ENODEV, "invalid device model 0x%04x\n",
                        model);
    goto done;
  }

  ret = 0;

done:
  imx415_regmap_write(sensor, IMX415_MODE, IMX415_MODE_STANDBY);
  return ret;
}

static int imx415_check_inck(unsigned long inck, u64 link_frequency) {
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE(imx415_clk_params); ++i) {
    if ((imx415_clk_params[i].lane_rate == link_frequency * 2) &&
        imx415_clk_params[i].inck == inck)
      break;
  }

  if (i == ARRAY_SIZE(imx415_clk_params))
    return -EINVAL;
  else
    return 0;
}

static int
imx415_retrieve_capture_properties(struct imx415 *sensor,
                                   struct imx415_capture_properties *ocp) {
  struct device *dev = &sensor->i2c_client->dev;
  __u64 mlf = 0;
  __u64 mpf = 0;
  __u64 mdr = 0;

  struct device_node *ep;
  int ret;
  /*Collecting the information about limits of capture path
   * has been centralized to the sensor
   * * also into the sensor endpoint itself.
   */

  ep = of_graph_get_next_endpoint(dev->of_node, NULL);
  if (!ep) {
    dev_err(dev, "missing endpoint node\n");
    return -ENODEV;
  }

  ret = fwnode_property_read_u64(of_fwnode_handle(ep), "max-pixel-frequency",
                                 &mpf);
  if (ret || mpf == 0) {
    dev_dbg(dev, "no limit for max-pixel-frequency\n");
  }

  ocp->max_lane_frequency = mlf;
  ocp->max_pixel_frequency = mpf;
  ocp->max_data_rate = mdr;

  return ret;
}

static int imx415_parse_hw_config(struct imx415 *sensor) {
  struct v4l2_fwnode_endpoint bus_cfg = {
      .bus_type = V4L2_MBUS_CSI2_DPHY,
  };
  struct fwnode_handle *ep;
  u64 lane_rate;
  unsigned long inck;
  unsigned int i, j;
  int ret, retval;

  sensor->clk = devm_clk_get(sensor->dev, "csi_mclk");
  if (IS_ERR(sensor->clk))
    return dev_err_probe(sensor->dev, PTR_ERR(sensor->clk),
                         "failed to get clock\n");

  retval =
      of_property_read_u32(sensor->dev->of_node, "csi_id", &(sensor->csi_id));
  if (retval) {
    dev_err(sensor->dev, "csi id missing or invalid\n");
    return retval;
  }

  retval = imx415_retrieve_capture_properties(sensor, &sensor->ocp);
  if (retval) {
    dev_warn(sensor->dev, "retrive capture properties error\n");
  }

  ep = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);

  if (!ep)
    return -ENXIO;

  ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
  fwnode_handle_put(ep);
  dev_err(sensor->dev, "parse_hw_config : lanes=%d link_freqs=%d mclk=%lu\n",
          bus_cfg.bus.mipi_csi2.num_data_lanes, bus_cfg.nr_of_link_frequencies,
          clk_get_rate(sensor->clk));
  if (ret)
    return ret;

  switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
  case 2:
  case 4:
    sensor->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
    break;
  default:
    ret = dev_err_probe(sensor->dev, -EINVAL,
                        "invalid number of CSI2 data lanes %d\n",
                        bus_cfg.bus.mipi_csi2.num_data_lanes);
    goto done_endpoint_free;
  }

  if (!bus_cfg.nr_of_link_frequencies) {
    ret = dev_err_probe(sensor->dev, -EINVAL, "no link frequencies defined");
    goto done_endpoint_free;
  }
  /*
   * Check if there exists a sensor mode defined for current INCK,
   * number of lanes and given lane rates.
   */
  inck = clk_get_rate(sensor->clk);
  for (i = 0; i < bus_cfg.nr_of_link_frequencies; ++i) {
    if (imx415_check_inck(inck, bus_cfg.link_frequencies[i])) {
      dev_err(sensor->dev, "INCK %lu Hz not supported for this link freq",
              inck);
      continue;
    }

    for (j = 0; j < ARRAY_SIZE(supported_modes); ++j) {
      if (sensor->num_data_lanes != supported_modes[j].lanes)
        continue;
      if (bus_cfg.link_frequencies[i] * 2 != supported_modes[j].lane_rate)
        continue;
      sensor->cur_mode = j;
      break;
    }
    if (j < ARRAY_SIZE(supported_modes))
      break;
  }
  if (i == bus_cfg.nr_of_link_frequencies) {
    ret = dev_err_probe(sensor->dev, -EINVAL, "no valid sensor mode defined\n");
    goto done_endpoint_free;
  }
  lane_rate = supported_modes[sensor->cur_mode].lane_rate;
  for (i = 0; i < ARRAY_SIZE(imx415_clk_params); ++i) {
    if (lane_rate == imx415_clk_params[i].lane_rate &&
        inck == imx415_clk_params[i].inck) {
      sensor->clk_params = &imx415_clk_params[i];
      break;
    }
  }
  if (i == ARRAY_SIZE(imx415_clk_params)) {
    ret = dev_err_probe(sensor->dev, -EINVAL, "Mode %d not supported\n",
                        sensor->cur_mode);
    goto done_endpoint_free;
  }

  ret = 0;
  dev_err(sensor->dev,
          "clock: %lu Hz, lane_rate: %llu bps, lanes: %d, Mode: %d\n", inck,
          lane_rate, sensor->num_data_lanes, sensor->cur_mode);

done_endpoint_free:
  v4l2_fwnode_endpoint_free(&bus_cfg);

  return ret;
}

static int imx415_probe(struct i2c_client *client) {
  struct imx415 *sensor;
  struct v4l2_subdev *sd;
  int ret;

  sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
  if (!sensor)
    return -ENOMEM;

  sensor->i2c_client = client;

  sensor->dev = &client->dev;

  ret = imx415_parse_hw_config(sensor);
  if (ret)
    return ret;

  sensor->regmap = devm_regmap_init_i2c(client, &imx415_regmap_config);
  if (IS_ERR(sensor->regmap)) {
    dev_warn(sensor->dev, "regmap init failed: %ld\n", PTR_ERR(sensor->regmap));
    return PTR_ERR(sensor->regmap);
  }

  /*
   * Enable power management. The driver supports runtime PM, but needs to
   * work when runtime PM is disabled in the kernel. To that end, power
   * the sensor on manually here, identify it, and fully initialize it.
   */

  ret = imx415_identify_model(sensor);
  if (ret) {
    dev_err(sensor->dev, "imx415_identify_model\n");
    goto err_power;
  }

  // custom code
  memcpy(&sensor->custom_cur_mode, &imx415_mode_info[0],
         sizeof(struct vvcam_mode_info_s));

  ret = imx415_subdev_init(sensor);
  if (ret) {
    dev_err(sensor->dev, "imx415_subdev_init\n");
    goto err_power;
  }

  /*
   * Enable runtime PM. As the device has been powered manually, mark it
   * as active, and increase the usage count without resuming the device.
   */
  pm_runtime_set_active(sensor->dev);
  pm_runtime_get_noresume(sensor->dev);
  pm_runtime_enable(sensor->dev);

  ret = v4l2_async_register_subdev_sensor(&sensor->subdev);
  if (ret < 0) {
    dev_err(sensor->dev, "v4l2_async_register_subdev_sensor\n");
    goto err_pm;
  }

   mutex_init(&sensor->lock);

  /*
   * Finally, enable autosuspend and decrease the usage count. The device
   * will get suspended after the autosuspend delay, turning the power
   * off.
   */
  dev_err(sensor->dev, " Probed succesfully\n");

  pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
  pm_runtime_use_autosuspend(sensor->dev);
  pm_runtime_put_autosuspend(sensor->dev);

  return 0;

err_pm:
  pm_runtime_disable(sensor->dev);
  pm_runtime_put_noidle(sensor->dev);
  imx415_subdev_cleanup(sensor);
  media_entity_cleanup(&sd->entity);

err_power:
  printk("PROBE FAILED\n");

  return ret;
}

static void imx415_remove(struct i2c_client *client) {
  struct v4l2_subdev *sd = i2c_get_clientdata(client);
  struct imx415 *sensor = client_to_imx415(client);
  pr_info("enter %s\n", __func__);

  v4l2_async_unregister_subdev(sd);

  imx415_subdev_cleanup(sensor);

  media_entity_cleanup(&sd->entity);

  pm_runtime_disable(sensor->dev);

  pm_runtime_set_suspended(sensor->dev);

  mutex_destroy(&sensor->lock);
}

static int imx415_runtime_suspend(struct device *dev) {
  struct i2c_client *client = to_i2c_client(dev);

  struct imx415 *sensor = client_to_imx415(client);

  dev_info(sensor->dev, "Enter imx415_runtime_suspend\n");

  sensor->resume_status = sensor->stream_status;
  if (sensor->resume_status) {
    imx415_s_stream(&sensor->subdev, 0);
  }

  return 0;
}

static int imx415_runtime_resume(struct device *dev) {
  struct i2c_client *client = to_i2c_client(dev);
  struct imx415 *sensor = client_to_imx415(client);
  dev_info(sensor->dev, "Enter imx415_runtime_resume\n");
  if (sensor->resume_status) {
    imx415_s_stream(&sensor->subdev, 1);
  }

  return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx415_pm_ops, imx415_runtime_suspend,
                                 imx415_runtime_resume, NULL);

static const struct i2c_device_id imx415_id[] = {
    {"imx415", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, imx415_id);

static const struct of_device_id imx415_of_match[] = {
    {.compatible = "sony,imx415"}, {/* sentinel */}};

MODULE_DEVICE_TABLE(of, imx415_of_match);

static struct i2c_driver imx415_i2c_driver = {

    .probe = imx415_probe,
    .remove = imx415_remove,
    .id_table = imx415_id,
    .driver =
        {
            .owner = THIS_MODULE,
            .name = "imx415",
            .of_match_table = imx415_of_match,
            .pm = &imx415_pm_ops,
        },
};

module_i2c_driver(imx415_i2c_driver);

MODULE_DESCRIPTION("Sony IMX415 image sensor driver developed and ported onto compulab/nxp IMX8MPlus ");
MODULE_AUTHOR("Vishnu <siddavatamvishnureddy@gmail.com>");
MODULE_LICENSE("GPL");
