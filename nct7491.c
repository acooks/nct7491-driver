/*
 * nct7491 - Thermal sensor driver for the NCT7491 chip
 *
 * Copyright (C) 2018 Andrew Cooks <andrew.cooks@opengear.com>
 *
 * Derived from the adt7475 driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/util_macros.h>
#include <linux/acpi.h>

/* Indexes for the sysfs hooks */

#define INPUT		0
#define MIN		1
#define MAX		2
#define CONTROL		3
#define OFFSET		3
#define AUTOMIN		4
#define THERM		5
#define HYSTERSIS	6

#define SMB_SLAVE_ADDRESS 0
#define SMB_SLAVE_REG 1
#define SMB_SLAVE_TEMP_VALUE 2

/*
 * These are unique identifiers for the sysfs functions - unlike the
 * numbers above, these are not also indexes into an array
 */

#define ALARM		9
#define FAULT		10

#define ONSEMI_VENDID		0x1A	/* NCT7491 vendor ID */
#define NCT7491_DEVID		0x91	/* NCT7491 device ID */
#define NCT7491_VERSION		0x6C	/* NCT7491 version */

/* Registers */

#define REG_CONFIG6		0x10
#define REG_CONFIG7		0x11
#define REG_STATUS6		0x12
#define REG_CONFIG8		0x13

#define REG_THERM_CONFIG1	0x16
#define REG_DEVID		0x1D	/* power-on value = NCT7491_DEVID */

#define REG_VTT			0x1E
#define REG_EXTEND3		0x1F

#define REG_VOLTAGE_BASE	0x20
#define REG_TEMP_BASE		0x25
#define REG_TACH_BASE		0x28
#define REG_PWM_BASE		0x30
#define REG_PWM_MAX_BASE	0x38

#define REG_PECI_TMIN		0x3B
#define REG_PECI_TRANGE		0x3C
#define REG_PECI_CTRL		0x3D

#define REG_VENDID		0x3E	/* power-on value = ONSEMI_VENDID */
#define REG_VERSION		0x3F	/* power-on value = NCT7491_VERSION */

#define REG_CONFIG1		0x40

#define REG_STATUS1		0x41
#define REG_STATUS2		0x42
#define REG_STATUS3		0x43

#define REG_VOLTAGE_MIN_BASE	0x44
#define REG_VOLTAGE_MAX_BASE	0x45

#define REG_TEMP_MIN_BASE	0x4E
#define REG_TEMP_MAX_BASE	0x4F

#define REG_TACH_MIN_BASE	0x54

#define REG_PWM_CONFIG_BASE	0x5C

#define REG_TEMP_TRANGE_BASE	0x5F

#define REG_ENHANCE_ACOUSTICS1	0x62
#define REG_ENHANCE_ACOUSTICS2	0x63

#define REG_PWM_MIN_BASE	0x64

#define REG_TEMP_TMIN_BASE	0x67
#define REG_TEMP_THERM_BASE	0x6A

#define REG_REMOTE1_HYSTERSIS	0x6D
#define REG_REMOTE2_HYSTERSIS	0x6E

#define REG_TEMP_OFFSET_BASE	0x70

#define REG_CONFIG2		0x73

#define REG_EXTEND1		0x76
#define REG_EXTEND2		0x77

#define REG_CONFIG3		0x78
#define REG_CONFIG5		0x7C
#define REG_CONFIG4		0x7D

#define REG_GPIO		0x80
#define REG_STATUS4		0x81

#define REG_VTT_MIN		0x84
#define REG_VTT_MAX		0x86

#define REG_CONFIG9		0x87

#define REG_PECI_CONFIG2	0x88

#define REG_STATUS7		0x89

#define REG_PWM_SOURCE_BASE	0x8A

#define REG_REVISION		0x93	/* value not documented in datasheet */

#define REG_PECI_OFFSET_BASE	0x94

/* 8 SMBus devices */
#define REG_SMBUS_SLAVES_BASE	0x98	/* 1 address, 1 temperature ptr */
#define REG_SMBUS_VALUES_BASE	0xA8	/* 1 temperature value each */

#define REG_SMBUS_CONFIG1	0xB0	/* repeated start config */
#define REG_SMBUS_CONFIG2	0xB1	/* PEC byte support */
#define REG_SMBUS_CONFIG3	0xB2	/* temperature value format, dev 0-3 */
#define REG_SMBUS_CONFIG4	0xB3	/* temperature value format, dev 4-7 */
#define REG_SMBUS_CONFIG5	0xB5

#define REG_SMBUS_STATUS_BASE   0xB6
#define REG_SMBUS_STATUS_NACK	0xB6	/* Nack */
#define REG_SMBUS_STATUS_PEC	0xB7	/* PEC error */
#define REG_SMBUS_STATUS_TOE	0xB8	/* Timeout error */
#define REG_SMBUS_STATUS_RANGE	0xB9	/* value outside limit */
#define REG_SMBUS_STATUS_TIV	0xBA	/* temperature invalid */
#define REG_SMBUS_STATUS_THERM	0xBB	/* THERM limit exceeded */

#define REG_SMBUS_LIMIT_HI	0xC1	/* power-on default 0x7F */
#define REG_SMBUS_LIMIT_LO	0xC2	/* power-on default 0x81 */
#define REG_SMBUS_LIMIT_THERM	0xC3	/* power-on default 0x64 */
#define REG_SMBUS_TMIN		0xC6	/* power-on default 0x5A */
#define REG_SMBUS_RANGE_INT	0xC7

#define REG_PUSH_TMIN		0xCC

/* Config register bits */

#define CONFIG3_SMBALERT	0x01
#define CONFIG3_THERM		0x02

#define CONFIG4_PINFUNC		0x03
#define CONFIG4_ATTN_IN10	0x30
#define CONFIG4_ATTN_IN43	0xC0

#define CONFIG5_TWOSCOMP	0x01
#define CONFIG5_TEMPOFFSET	0x02

/* Settings */

#define NCT7491_VOLTAGE_COUNT	5	/* Not counting Vtt */
#define NCT7491_TEMP_COUNT	3
#define NCT7491_TACH_COUNT	4
#define NCT7491_PWM_COUNT	3

#define NCT7491_SMBUS_SLAVE_COUNT	8


#define TEMP_FORMAT_TWOSCOMP	0
#define TEMP_FORMAT_JEDEC_SPD	1
#define TEMP_FORMAT_UNSIGNED	2
#define TEMP_FORMAT_PCH_BLOCK	3	/* device0 only */

/* Macro to read the registers */

#if 1
#define nct7491_read(reg) i2c_smbus_read_byte_data(client, (reg))
#else
u8 _nct7491_read(const char *func, struct i2c_client *client, int reg)
{
	u8 val = i2c_smbus_read_byte_data(client, (reg));

	pr_debug("%s: 0x%02x = 0x%02x", func, reg, val);
	return val;
}
#define nct7491_read(reg) _nct7491_read(__func__, client, reg)
#endif

/* Macros to easily index the registers */

#define TACH_REG(idx) (REG_TACH_BASE + ((idx) * 2))
#define TACH_MIN_REG(idx) (REG_TACH_MIN_BASE + ((idx) * 2))

#define PWM_REG(idx) (REG_PWM_BASE + (idx))
#define PWM_MAX_REG(idx) (REG_PWM_MAX_BASE + (idx))
#define PWM_MIN_REG(idx) (REG_PWM_MIN_BASE + (idx))
#define PWM_CONFIG_REG(idx) (REG_PWM_CONFIG_BASE + (idx))
#define PWM_SRC_CTL_REG(idx) (REG_PWM_SOURCE_BASE + ((idx) * 3))

#define VOLTAGE_REG(idx) (REG_VOLTAGE_BASE + (idx))
#define VOLTAGE_MIN_REG(idx) (REG_VOLTAGE_MIN_BASE + ((idx) * 2))
#define VOLTAGE_MAX_REG(idx) (REG_VOLTAGE_MAX_BASE + ((idx) * 2))

#define TEMP_REG(idx) (REG_TEMP_BASE + (idx))
#define TEMP_MIN_REG(idx) (REG_TEMP_MIN_BASE + ((idx) * 2))
#define TEMP_MAX_REG(idx) (REG_TEMP_MAX_BASE + ((idx) * 2))
#define TEMP_TMIN_REG(idx) (REG_TEMP_TMIN_BASE + (idx))
#define TEMP_THERM_REG(idx) (REG_TEMP_THERM_BASE + (idx))
#define TEMP_OFFSET_REG(idx) (REG_TEMP_OFFSET_BASE + (idx))
#define TEMP_TRANGE_REG(idx) (REG_TEMP_TRANGE_BASE + (idx))

#define SMB_SLAVE_ADDR_REG(idx) (REG_SMBUS_SLAVES_BASE + ((idx) * 2))
#define SMB_SLAVE_TEMP_REG(idx) (REG_SMBUS_VALUES_BASE + (idx))

static const unsigned short normal_i2c[] = { 0x2c, 0x2d, 0x2e, I2C_CLIENT_END };

enum chips { nct7491 };

static const struct i2c_device_id nct7491_id[] = {
	{ "nct7491", nct7491 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct7491_id);

static const struct of_device_id nct7491_of_match[] = {
	{
		.compatible = "onnn,nct7491",
		.data = (void *)nct7491
	},
	{ },
};
MODULE_DEVICE_TABLE(of, nct7491_of_match);

struct smb_sensor {
	u8 bus_addr;
	u8 reg;
	u8 format;
	u8 offset;	/* driver only. nct7491 doesn't support this. */
	u16 value;
	const char *name;
};

struct nct7491_data {
	struct device *hwmon_dev;
	struct mutex lock;

	unsigned long measure_updated;
	unsigned long limits_updated;
	char valid;


	u32 alarms;
	u16 voltage[3][6];
	u16 temp[7][3];
	u16 tach[2][4];
	u8 pwm[4][3];
	u8 range[3];
	u8 pwmctl[3];
	int pwmsrc[3];
	u8 enh_acoustics[2];
	u8 vid;
	u8 vrm;

	struct smb_sensor smb_sensor[8];
	u8 smb_sensor_lo;	/* device stores 2's complement format */
	u8 smb_sensor_hi;
	u8 smb_therm;
	u8 smb_tmin;
	u8 has_pch:1;

	u8 bypass_attn:NCT7491_VOLTAGE_COUNT;	/* Bypass voltage attenuator */
	u8 has_2p5v:1;
	u8 has_pwm2:1;		/* !(REG_CONFIG3 & CONFIG3_SMBALERT) */
	u8 has_fan4:1;		/* REG_CONFIG4 & CONFIG4_PINFUNC */
	u8 twoscomp:1;		/* REG_CONFIG5 & CONFIG5_TWOSCOMP */
	u8 tempoffset:1;	/* REG_CONFIG5 & CONFIG5_TEMPOFFSET */

	u8 therm_sources:5;
};

/* This is the order of struct nct7491_data.therm_sources bitmap. */
static const char *acpi_therm_sources[] = {
	"therm-enable-remote1",
	"therm-enable-local",
	"therm-enable-remote2",
	"therm-enable-push",
	"therm-enable-smbus"
};


static struct i2c_driver nct7491_driver;
static struct nct7491_data *nct7491_read_sensors(struct device *dev);
static void nct7491_read_hystersis(struct i2c_client *client);
static void nct7491_read_pwm(struct i2c_client *client, int index);
static void read_firmware_pwm_sources(struct i2c_client *client,
				      struct nct7491_data *priv);

/* Given a temp value, convert it to register value */

static inline u16 temp2reg(struct nct7491_data *data, long val)
{
	u16 ret;

	if (!(data->twoscomp)) {
		val = clamp_val(val, -64000, 191000);
		ret = (val + 64500) / 1000;
	} else {
		val = clamp_val(val, -128000, 127000);
		if (val < -500)
			ret = (256500 + val) / 1000;
		else
			ret = (val + 500) / 1000;
	}

	return ret << 2;
}

/* Given a register value, convert it to a real temp value */

static inline int reg2temp(struct nct7491_data *data, u16 reg)
{
	if (data->twoscomp) {
		if (reg >= 512)
			return (reg - 1024) * 250;
		else
			return reg * 250;
	} else
		return (reg - 256) * 250;
}

static inline int tach2rpm(u16 tach)
{
	if (tach == 0 || tach == 0xFFFF)
		return 0;

	return (78000 * 60) / tach;
}

static inline u16 rpm2tach(unsigned long rpm)
{
	if (rpm == 0)
		return 0;

	return clamp_val((78000 * 60) / rpm, 1, 0xFFFF);
}

/* Scaling factors for voltage inputs, taken from the NCT7491 datasheet */
static const int nct7491_in_scaling[NCT7491_VOLTAGE_COUNT + 1][2] = {
	{ 4940, 7400 },	/* +2.5V */
	{ 4110, 8230 },	/* Vccp */
	{ 6670, 5580 },	/* Vcc */
	{ 8590, 3660 },	/* +5V */
	{ 2157, 3020 },	/* +12V */
	{ 1370, 1097 },	/* Vtt */
};

static inline int reg2volt(int channel, u16 reg, u8 bypass_attn)
{
	const int *r = nct7491_in_scaling[channel];

	if (bypass_attn & (1 << channel))
		return DIV_ROUND_CLOSEST(reg * 2000, 1024);
	return DIV_ROUND_CLOSEST(reg * (r[0] + r[1]) * 2000, r[1] * 1024);
}

static inline u16 volt2reg(int channel, long volt, u8 bypass_attn)
{
	const int *r = nct7491_in_scaling[channel];
	long reg;

	if (bypass_attn & (1 << channel))
		reg = (volt * 1024) / 2000;
	else
		reg = (volt * r[1] * 1024) / ((r[0] + r[1]) * 2000);
	return clamp_val(reg, 0, 1023) & (0xff << 2);
}

static u16 nct7491_read_word(struct i2c_client *client, int reg)
{
	u16 val;

	val = i2c_smbus_read_byte_data(client, reg);
	val |= (i2c_smbus_read_byte_data(client, reg + 1) << 8);

	return val;
}

static void nct7491_write_word(struct i2c_client *client, int reg, u16 val)
{
	i2c_smbus_write_byte_data(client, reg + 1, val >> 8);
	i2c_smbus_write_byte_data(client, reg, val & 0xFF);
}

static void chip_write_pwm_src_ctl(struct i2c_client *client,
				   struct nct7491_data *priv)
{
	int i;

	for (i = 0; i < 3; i++) {
		i2c_smbus_write_byte_data(client, PWM_SRC_CTL_REG(i) + 2,
					  (priv->pwmsrc[i] >> 16) & 0xFF);
		i2c_smbus_write_byte_data(client, PWM_SRC_CTL_REG(i) + 1,
					  (priv->pwmsrc[i] >> 8) & 0xFF);
		i2c_smbus_write_byte_data(client, PWM_SRC_CTL_REG(i),
					  priv->pwmsrc[i] & 0xFF);

		dev_dbg(&client->dev, "%s pwm%d src ctl: 0x%02x = 0x%06x",
			__func__, i + 1, PWM_SRC_CTL_REG(i), priv->pwmsrc[i]);

		/* write the pwm minimum duty cycle */
		i2c_smbus_write_byte_data(client, PWM_MIN_REG(i),
					  priv->temp[AUTOMIN][i]);
	}
}

static ssize_t show_voltage(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	unsigned short val;

	switch (sattr->nr) {
	case ALARM:
		return sprintf(buf, "%d\n",
				(data->alarms >> sattr->index) & 1);
	default:
		val = data->voltage[sattr->nr][sattr->index];
		return sprintf(buf, "%d\n",
			       reg2volt(sattr->index, val, data->bypass_attn));
	}
}

static ssize_t set_voltage(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{

	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	unsigned char reg;
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	data->voltage[sattr->nr][sattr->index] =
				volt2reg(sattr->index, val, data->bypass_attn);

	if (sattr->index < NCT7491_VOLTAGE_COUNT) {
		if (sattr->nr == MIN)
			reg = VOLTAGE_MIN_REG(sattr->index);
		else
			reg = VOLTAGE_MAX_REG(sattr->index);
	} else {
		if (sattr->nr == MIN)
			reg = REG_VTT_MIN;
		else
			reg = REG_VTT_MAX;
	}

	i2c_smbus_write_byte_data(client, reg,
				  data->voltage[sattr->nr][sattr->index] >> 2);
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_smb_temp(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out;

	mutex_lock(&data->lock);

	switch (data->smb_sensor[sattr->index].format) {
	case TEMP_FORMAT_TWOSCOMP:
		out = (s8)data->smb_sensor[sattr->index].value;
		out += data->smb_sensor[sattr->index].offset;
		out *= 1000;
		break;
	case TEMP_FORMAT_JEDEC_SPD:
		dev_dbg(dev, "sensor%d JEDEC SPD format TODO.", sattr->index);
		/* fall through */
	case TEMP_FORMAT_UNSIGNED:
	case TEMP_FORMAT_PCH_BLOCK:	/* device0 only */
		out = reg2temp(data, data->smb_sensor[sattr->index].value);
		out += data->smb_sensor[sattr->index].offset;
		out *= 1000;
		break;
	default:
		dev_info(dev, "sensor%d format:%d unknown.", sattr->index,
			 data->smb_sensor[sattr->index].format);
		out = -128000;
	}
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", out);

}

static ssize_t show_temp(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out;

	switch (sattr->nr) {
	case HYSTERSIS:
		mutex_lock(&data->lock);
		out = data->temp[sattr->nr][sattr->index];
		if (sattr->index != 1)
			out = (out >> 4) & 0xF;
		else
			out = (out & 0xF);
		/*
		 * Show the value as an absolute number tied to
		 * THERM
		 */
		out = reg2temp(data, data->temp[THERM][sattr->index]) -
			out * 1000;
		mutex_unlock(&data->lock);
		break;

	case OFFSET:
		/*
		 * Offset is always 2's complement, regardless of the
		 * setting in CONFIG5
		 */
		mutex_lock(&data->lock);
		out = (s8)data->temp[sattr->nr][sattr->index];
		if (data->tempoffset)
			out *= 1000;
		else
			out *= 500;
		mutex_unlock(&data->lock);
		break;

	case ALARM:
		out = (data->alarms >> (sattr->index + 4)) & 1;
		break;

	case FAULT:
		/* Note - only for remote1 and remote2 */
		out = !!(data->alarms & (sattr->index ? 0x8000 : 0x4000));
		break;

	default:
		/* All other temp values are in the configured format */
		out = reg2temp(data, data->temp[sattr->nr][sattr->index]);
	}

	return sprintf(buf, "%d\n", out);
}

static ssize_t set_temp(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	unsigned char reg = 0;
	u8 out;
	int temp;
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	switch (sattr->nr) {
	case OFFSET:
		if (data->tempoffset) {
			val = clamp_val(val, -63000, 127000);
			out = data->temp[OFFSET][sattr->index] = val / 1000;
		} else {
			val = clamp_val(val, -63000, 64000);
			out = data->temp[OFFSET][sattr->index] = val / 500;
		}
		break;

	case HYSTERSIS:
		/*
		 * The value will be given as an absolute value, turn it
		 * into an offset based on THERM
		 */

		/* Read fresh THERM and HYSTERSIS values from the chip */
		data->temp[THERM][sattr->index] =
			nct7491_read(TEMP_THERM_REG(sattr->index)) << 2;
		nct7491_read_hystersis(client);

		temp = reg2temp(data, data->temp[THERM][sattr->index]);
		val = clamp_val(val, temp - 15000, temp);
		val = (temp - val) / 1000;

		if (sattr->index != 1) {
			data->temp[HYSTERSIS][sattr->index] &= 0xF0;
			data->temp[HYSTERSIS][sattr->index] |= (val & 0xF) << 4;
		} else {
			data->temp[HYSTERSIS][sattr->index] &= 0x0F;
			data->temp[HYSTERSIS][sattr->index] |= (val & 0xF);
		}

		out = data->temp[HYSTERSIS][sattr->index];
		break;

	default:
		data->temp[sattr->nr][sattr->index] = temp2reg(data, val);

		/*
		 * We maintain an extra 2 digits of precision for simplicity
		 * - shift those back off before writing the value
		 */
		out = (u8) (data->temp[sattr->nr][sattr->index] >> 2);
	}

	switch (sattr->nr) {
	case MIN:
		reg = TEMP_MIN_REG(sattr->index);
		break;
	case MAX:
		reg = TEMP_MAX_REG(sattr->index);
		break;
	case OFFSET:
		reg = TEMP_OFFSET_REG(sattr->index);
		break;
	case AUTOMIN:
		reg = TEMP_TMIN_REG(sattr->index);
		break;
	case THERM:
		reg = TEMP_THERM_REG(sattr->index);
		break;
	case HYSTERSIS:
		if (sattr->index != 2)
			reg = REG_REMOTE1_HYSTERSIS;
		else
			reg = REG_REMOTE2_HYSTERSIS;

		break;
	}

	i2c_smbus_write_byte_data(client, reg, out);

	mutex_unlock(&data->lock);
	return count;
}

static const int nct7491_st_map[] = {
	37500, 18800, 12500, 7500, 4700, 3100, 1600, 800,
};

static ssize_t show_temp_st(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	long val;

	switch (sattr->index) {
	case 0:
		val = data->enh_acoustics[0] & 0xf;
		break;
	case 1:
		val = (data->enh_acoustics[1] >> 4) & 0xf;
		break;
	case 2:
	default:
		val = data->enh_acoustics[1] & 0xf;
		break;
	}

	if (val & 0x8)
		return sprintf(buf, "%d\n", nct7491_st_map[val & 0x7]);
	else
		return sprintf(buf, "0\n");
}

static ssize_t set_temp_st(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	unsigned char reg;
	int shift, idx;
	ulong val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (sattr->index) {
	case 0:
		reg = REG_ENHANCE_ACOUSTICS1;
		shift = 0;
		idx = 0;
		break;
	case 1:
		reg = REG_ENHANCE_ACOUSTICS2;
		shift = 0;
		idx = 1;
		break;
	case 2:
	default:
		reg = REG_ENHANCE_ACOUSTICS2;
		shift = 4;
		idx = 1;
		break;
	}

	if (val > 0) {
		val = find_closest_descending(val, nct7491_st_map,
					      ARRAY_SIZE(nct7491_st_map));
		val |= 0x8;
	}

	mutex_lock(&data->lock);

	data->enh_acoustics[idx] &= ~(0xf << shift);
	data->enh_acoustics[idx] |= (val << shift);

	i2c_smbus_write_byte_data(client, reg, data->enh_acoustics[idx]);

	mutex_unlock(&data->lock);

	return count;
}

/*
 * Table of autorange values - the user will write the value in millidegrees,
 * and we'll convert it
 */
static const int autorange_table[] = {
	2000, 2500, 3330, 4000, 5000, 6670, 8000,
	10000, 13330, 16000, 20000, 26670, 32000, 40000,
	53330, 80000
};

static ssize_t show_point2(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out, val;

	mutex_lock(&data->lock);
	out = (data->range[sattr->index] >> 4) & 0x0F;
	val = reg2temp(data, data->temp[AUTOMIN][sattr->index]);
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", val + autorange_table[out]);
}

static ssize_t set_point2(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int temp;
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	/* Get a fresh copy of the needed registers */
	data->temp[AUTOMIN][sattr->index] =
		nct7491_read(TEMP_TMIN_REG(sattr->index)) << 2;
	data->range[sattr->index] =
		nct7491_read(TEMP_TRANGE_REG(sattr->index));

	/*
	 * The user will write an absolute value, so subtract the start point
	 * to figure the range
	 */
	temp = reg2temp(data, data->temp[AUTOMIN][sattr->index]);
	val = clamp_val(val, temp + autorange_table[0],
		temp + autorange_table[ARRAY_SIZE(autorange_table) - 1]);
	val -= temp;

	/* Find the nearest table entry to what the user wrote */
	val = find_closest(val, autorange_table, ARRAY_SIZE(autorange_table));

	data->range[sattr->index] &= ~0xF0;
	data->range[sattr->index] |= val << 4;

	i2c_smbus_write_byte_data(client, TEMP_TRANGE_REG(sattr->index),
				  data->range[sattr->index]);

	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_tach(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int out;

	if (sattr->nr == ALARM)
		out = (data->alarms >> (sattr->index + 10)) & 1;
	else
		out = tach2rpm(data->tach[sattr->nr][sattr->index]);

	return sprintf(buf, "%d\n", out);
}

static ssize_t set_tach(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{

	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	data->tach[MIN][sattr->index] = rpm2tach(val);

	nct7491_write_word(client, TACH_MIN_REG(sattr->index),
			   data->tach[MIN][sattr->index]);

	mutex_unlock(&data->lock);
	return count;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	return sprintf(buf, "%d\n", data->pwm[sattr->nr][sattr->index]);
}

static ssize_t show_pwmchan(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	return sprintf(buf, "%d\n", data->pwmsrc[sattr->index]);
}

static ssize_t show_pwmctrl(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);

	nct7491_read_pwm(client, sattr->index);

	return sprintf(buf, "%d\n", data->pwmctl[sattr->index]);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{

	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	unsigned char reg = 0;
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	switch (sattr->nr) {
	case INPUT:
		nct7491_read_pwm(client, sattr->index);
		/* If we are in auto mode, the pwm speed can't be changed. */
		if (data->pwmctl[sattr->index] > 1) {
			mutex_unlock(&data->lock);
			return count;
		}

		reg = PWM_REG(sattr->index);
		break;

	case MIN:
		reg = PWM_MIN_REG(sattr->index);
		break;

	case MAX:
		reg = PWM_MAX_REG(sattr->index);
		break;
	}

	data->pwm[sattr->nr][sattr->index] = clamp_val(val, 0, 0xFF);
	i2c_smbus_write_byte_data(client, reg,
				  data->pwm[sattr->nr][sattr->index]);
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_stall_disable(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	u8 mask = BIT(5 + sattr->index);

	return sprintf(buf, "%d\n", !!(data->enh_acoustics[0] & mask));
}

static ssize_t set_stall_disable(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	long val;
	u8 mask = BIT(5 + sattr->index);

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);

	data->enh_acoustics[0] &= ~mask;
	if (val)
		data->enh_acoustics[0] |= mask;

	i2c_smbus_write_byte_data(client, REG_ENHANCE_ACOUSTICS1,
				  data->enh_acoustics[0]);

	mutex_unlock(&data->lock);

	return count;
}

/* Called by set_pwmctrl and set_pwmchan */

static int hw_set_pwm(struct i2c_client *client, int index,
		      unsigned int pwmctl, unsigned int pwmsrc)
{
	struct nct7491_data *data = i2c_get_clientdata(client);

	switch (pwmctl) {
	case 0:		/* Full speed */
		data->pwmsrc[index] = 0x00;	/* Disable sources */
		data->pwm[INPUT][index] = 0xFF;
		i2c_smbus_write_byte_data(client, PWM_REG(index),
					  data->pwm[INPUT][index]);
		break;
	case 1:		/* Manual mode */
		data->pwmsrc[index] = 0x00;
		break;
	case 2:		/* Auto control from firmware temp sources */
		read_firmware_pwm_sources(client, data);
		break;
	case 3:		/* Auto control from user-specified temp sources */
		data->pwmsrc[index] = pwmsrc;
		break;
	default:
		return -EINVAL;
	}

	data->pwmctl[index] = pwmctl;

	dev_dbg(&client->dev, "writing pwm%d sources: 0x%08x control:%u",
		index+1, data->pwmsrc[index], pwmctl);

	chip_write_pwm_src_ctl(client, data);

	return 0;
}

static ssize_t set_pwmchan(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	int r;
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);
	/* Read Modify Write PWM values */
	nct7491_read_pwm(client, sattr->index);
	r = hw_set_pwm(client, sattr->index, data->pwmctl[sattr->index], val);
	if (r)
		count = r;
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t set_pwmctrl(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	int r;
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&data->lock);
	/* Read Modify Write PWM values */
	nct7491_read_pwm(client, sattr->index);
	r = hw_set_pwm(client, sattr->index, val, data->pwmsrc[sattr->index]);
	if (r)
		count = r;
	mutex_unlock(&data->lock);

	return count;
}

/* List of frequencies for the PWM */
static const int pwmfreq_table[] = {
	11, 14, 22, 29, 35, 44, 58, 88, 22500
};

static ssize_t show_pwmfreq(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int i = clamp_val(data->range[sattr->index] & 0xf, 0,
			  ARRAY_SIZE(pwmfreq_table) - 1);

	return sprintf(buf, "%d\n", pwmfreq_table[i]);
}

static ssize_t set_pwmfreq(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	int out;
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	out = find_closest(val, pwmfreq_table, ARRAY_SIZE(pwmfreq_table));

	mutex_lock(&data->lock);

	data->range[sattr->index] =
		nct7491_read(TEMP_TRANGE_REG(sattr->index));
	data->range[sattr->index] &= ~0xf;
	data->range[sattr->index] |= out;

	i2c_smbus_write_byte_data(client, TEMP_TRANGE_REG(sattr->index),
				  data->range[sattr->index]);

	mutex_unlock(&data->lock);
	return count;
}

static ssize_t vrm_show(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct nct7491_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", (int)data->vrm);
}

static ssize_t vrm_store(struct device *dev, struct device_attribute *devattr,
			 const char *buf, size_t count)
{
	struct nct7491_data *data = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;
	if (val < 0 || val > 255)
		return -EINVAL;
	data->vrm = val;

	return count;
}

static ssize_t cpu0_vid_show(struct device *dev,
			     struct device_attribute *devattr, char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);

	return sprintf(buf, "%d\n", vid_from_reg(data->vid, data->vrm));
}

static ssize_t show_smb_crit_temp(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);

	return sprintf(buf, "%d\n", data->smb_therm * 1000);
}

static ssize_t set_smb_crit_temp(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = nct7491_read_sensors(dev);
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;
	val /= 1000;
	if (val < 0 || val > 255)
		return -EINVAL;

	mutex_lock(&data->lock);
	data->smb_therm = val;
	i2c_smbus_write_byte_data(client, REG_SMBUS_LIMIT_THERM, val);
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_smb_tmin(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);

	return sprintf(buf, "%d\n", data->smb_tmin * 1000);
}

static ssize_t set_smb_tmin(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = nct7491_read_sensors(dev);
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;
	val /= 1000;
	if (val < 0 || val > 175)
		return -EINVAL;

	mutex_lock(&data->lock);
	data->smb_tmin = val;
	i2c_smbus_write_byte_data(client, REG_SMBUS_TMIN, val);
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t show_therm_sources(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nct7491_data *data = nct7491_read_sensors(dev);

	return sprintf(buf, "%d\n", data->therm_sources);
}

static void write_therm_config(struct i2c_client *client, u8 therm_sources)
{
	u8 config5, therm_config1;

	config5 = nct7491_read(REG_CONFIG5);
	config5 &= 0x1F;	/* clear bits 5,6,7 */
	config5 |= (therm_sources & BIT(0)) ? BIT(5) : 0; /* remote 1 */
	config5 |= (therm_sources & BIT(1)) ? BIT(6) : 0; /* local */
	config5 |= (therm_sources & BIT(2)) ? BIT(7) : 0; /* remote 2 */
	i2c_smbus_write_byte_data(client, REG_CONFIG5, config5);

	therm_config1 = nct7491_read(REG_THERM_CONFIG1);
	therm_config1 &= 0x9F;	/* clear bits 5,6 */
	therm_config1 |= (therm_sources & BIT(3)) ? BIT(5) : 0; /* push */
	therm_config1 |= (therm_sources & BIT(4)) ? BIT(6) : 0; /* smbus */
	i2c_smbus_write_byte_data(client, REG_THERM_CONFIG1, therm_config1);
}

static ssize_t set_therm_sources(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = nct7491_read_sensors(dev);
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;
	if (val < 0 || val > 0x1f)
		return -EINVAL;

	mutex_lock(&data->lock);
	data->therm_sources = val & 0x1f; /* therm_sources is only 5 bits */
	write_therm_config(client, data->therm_sources);
	mutex_unlock(&data->lock);

	return count;
}

static SENSOR_DEVICE_ATTR_2(in0_input, 0444, show_voltage, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(in0_max, 0644, show_voltage, set_voltage, MAX, 0);
static SENSOR_DEVICE_ATTR_2(in0_min, 0644, show_voltage, set_voltage, MIN, 0);
static SENSOR_DEVICE_ATTR_2(in0_alarm, 0444, show_voltage, NULL, ALARM, 0);
static SENSOR_DEVICE_ATTR_2(in1_input, 0444, show_voltage, NULL, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(in1_max, 0644, show_voltage, set_voltage, MAX, 1);
static SENSOR_DEVICE_ATTR_2(in1_min, 0644, show_voltage, set_voltage, MIN, 1);
static SENSOR_DEVICE_ATTR_2(in1_alarm, 0444, show_voltage, NULL, ALARM, 1);
static SENSOR_DEVICE_ATTR_2(in2_input, 0444, show_voltage, NULL, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(in2_max, 0644, show_voltage, set_voltage, MAX, 2);
static SENSOR_DEVICE_ATTR_2(in2_min, 0644, show_voltage, set_voltage, MIN, 2);
static SENSOR_DEVICE_ATTR_2(in2_alarm, 0444, show_voltage, NULL, ALARM, 2);
static SENSOR_DEVICE_ATTR_2(in3_input, 0444, show_voltage, NULL, INPUT, 3);
static SENSOR_DEVICE_ATTR_2(in3_max, 0644, show_voltage, set_voltage, MAX, 3);
static SENSOR_DEVICE_ATTR_2(in3_min, 0644, show_voltage, set_voltage, MIN, 3);
static SENSOR_DEVICE_ATTR_2(in3_alarm, 0444, show_voltage, NULL, ALARM, 3);
static SENSOR_DEVICE_ATTR_2(in4_input, 0444, show_voltage, NULL, INPUT, 4);
static SENSOR_DEVICE_ATTR_2(in4_max, 0644, show_voltage, set_voltage, MAX, 4);
static SENSOR_DEVICE_ATTR_2(in4_min, 0644, show_voltage, set_voltage, MIN, 4);
static SENSOR_DEVICE_ATTR_2(in4_alarm, 0444, show_voltage, NULL, ALARM, 8);
static SENSOR_DEVICE_ATTR_2(in5_input, 0444, show_voltage, NULL, INPUT, 5);
static SENSOR_DEVICE_ATTR_2(in5_max, 0644, show_voltage, set_voltage, MAX, 5);
static SENSOR_DEVICE_ATTR_2(in5_min, 0644, show_voltage, set_voltage, MIN, 5);
static SENSOR_DEVICE_ATTR_2(in5_alarm, 0444, show_voltage, NULL, ALARM, 31);
static SENSOR_DEVICE_ATTR_2(temp1_input, 0444, show_temp, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(temp1_alarm, 0444, show_temp, NULL, ALARM, 0);
static SENSOR_DEVICE_ATTR_2(temp1_fault, 0444, show_temp, NULL, FAULT, 0);
static SENSOR_DEVICE_ATTR_2(temp1_max, 0644, show_temp, set_temp, MAX, 0);
static SENSOR_DEVICE_ATTR_2(temp1_min, 0644, show_temp, set_temp, MIN, 0);
static SENSOR_DEVICE_ATTR_2(temp1_offset, 0644, show_temp, set_temp, OFFSET, 0);
static SENSOR_DEVICE_ATTR_2(temp1_auto_point1_temp, 0644, show_temp, set_temp,
			    AUTOMIN, 0);
static SENSOR_DEVICE_ATTR_2(temp1_auto_point2_temp, 0644, show_point2,
			    set_point2, 0, 0);
static SENSOR_DEVICE_ATTR_2(temp1_crit, 0644, show_temp, set_temp, THERM, 0);
static SENSOR_DEVICE_ATTR_2(temp1_crit_hyst, 0644, show_temp,
			    set_temp, HYSTERSIS, 0);
static SENSOR_DEVICE_ATTR_2(temp1_smoothing, 0644, show_temp_st,
			    set_temp_st, 0, 0);
static SENSOR_DEVICE_ATTR_2(temp2_input, 0444, show_temp, NULL, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(temp2_alarm, 0444, show_temp, NULL, ALARM, 1);
static SENSOR_DEVICE_ATTR_2(temp2_max, 0644, show_temp, set_temp, MAX, 1);
static SENSOR_DEVICE_ATTR_2(temp2_min, 0644, show_temp, set_temp, MIN, 1);
static SENSOR_DEVICE_ATTR_2(temp2_offset, 0644, show_temp, set_temp, OFFSET, 1);
static SENSOR_DEVICE_ATTR_2(temp2_auto_point1_temp, 0644,
			    show_temp, set_temp, AUTOMIN, 1);
static SENSOR_DEVICE_ATTR_2(temp2_auto_point2_temp, 0644,
			    show_point2, set_point2, 0, 1);
static SENSOR_DEVICE_ATTR_2(temp2_crit, 0644, show_temp, set_temp, THERM, 1);
static SENSOR_DEVICE_ATTR_2(temp2_crit_hyst, 0644, show_temp,
			    set_temp, HYSTERSIS, 1);
static SENSOR_DEVICE_ATTR_2(temp2_smoothing, 0644, show_temp_st,
			    set_temp_st, 0, 1);
static SENSOR_DEVICE_ATTR_2(temp3_input, 0444, show_temp, NULL, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(temp3_alarm, 0444, show_temp, NULL, ALARM, 2);
static SENSOR_DEVICE_ATTR_2(temp3_fault, 0444, show_temp, NULL, FAULT, 2);
static SENSOR_DEVICE_ATTR_2(temp3_max, 0644, show_temp, set_temp, MAX, 2);
static SENSOR_DEVICE_ATTR_2(temp3_min, 0644, show_temp, set_temp, MIN, 2);
static SENSOR_DEVICE_ATTR_2(temp3_offset, 0644, show_temp, set_temp, OFFSET, 2);
static SENSOR_DEVICE_ATTR_2(temp3_auto_point1_temp, 0644,
			    show_temp, set_temp, AUTOMIN, 2);
static SENSOR_DEVICE_ATTR_2(temp3_auto_point2_temp, 0644,
			    show_point2, set_point2, 0, 2);
static SENSOR_DEVICE_ATTR_2(temp3_crit, 0644, show_temp, set_temp, THERM, 2);
static SENSOR_DEVICE_ATTR_2(temp3_crit_hyst, 0644, show_temp,
			    set_temp, HYSTERSIS, 2);
static SENSOR_DEVICE_ATTR_2(temp3_smoothing, 0644, show_temp_st,
			    set_temp_st, 0, 2);
static SENSOR_DEVICE_ATTR_2(fan1_input, 0444, show_tach, NULL, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(fan1_min, 0644, show_tach, set_tach, MIN, 0);
static SENSOR_DEVICE_ATTR_2(fan1_alarm, 0444, show_tach, NULL, ALARM, 0);
static SENSOR_DEVICE_ATTR_2(fan2_input, 0444, show_tach, NULL, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(fan2_min, 0644, show_tach, set_tach, MIN, 1);
static SENSOR_DEVICE_ATTR_2(fan2_alarm, 0444, show_tach, NULL, ALARM, 1);
static SENSOR_DEVICE_ATTR_2(fan3_input, 0444, show_tach, NULL, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(fan3_min, 0644, show_tach, set_tach, MIN, 2);
static SENSOR_DEVICE_ATTR_2(fan3_alarm, 0444, show_tach, NULL, ALARM, 2);
static SENSOR_DEVICE_ATTR_2(fan4_input, 0444, show_tach, NULL, INPUT, 3);
static SENSOR_DEVICE_ATTR_2(fan4_min, 0644, show_tach, set_tach, MIN, 3);
static SENSOR_DEVICE_ATTR_2(fan4_alarm, 0444, show_tach, NULL, ALARM, 3);
static SENSOR_DEVICE_ATTR_2(pwm1, 0644, show_pwm, set_pwm, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_freq, 0644, show_pwmfreq,
			    set_pwmfreq, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_enable, 0644, show_pwmctrl,
			    set_pwmctrl, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_auto_channels_temp, 0644,
			    show_pwmchan, set_pwmchan, INPUT, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_auto_point1_pwm, 0644, show_pwm,
			    set_pwm, MIN, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_auto_point2_pwm, 0644, show_pwm,
			    set_pwm, MAX, 0);
static SENSOR_DEVICE_ATTR_2(pwm1_stall_disable, 0644,
			    show_stall_disable, set_stall_disable, 0, 0);
static SENSOR_DEVICE_ATTR_2(pwm2, 0644, show_pwm, set_pwm, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_freq, 0644, show_pwmfreq,
			    set_pwmfreq, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_enable, 0644, show_pwmctrl,
			    set_pwmctrl, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_auto_channels_temp, 0644,
			    show_pwmchan, set_pwmchan, INPUT, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_auto_point1_pwm, 0644, show_pwm,
			    set_pwm, MIN, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_auto_point2_pwm, 0644, show_pwm,
			    set_pwm, MAX, 1);
static SENSOR_DEVICE_ATTR_2(pwm2_stall_disable, 0644,
			    show_stall_disable, set_stall_disable, 0, 1);
static SENSOR_DEVICE_ATTR_2(pwm3, 0644, show_pwm, set_pwm, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_freq, 0644, show_pwmfreq,
			    set_pwmfreq, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_enable, 0644, show_pwmctrl,
			    set_pwmctrl, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_auto_channels_temp, 0644,
			    show_pwmchan, set_pwmchan, INPUT, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_auto_point1_pwm, 0644, show_pwm,
			    set_pwm, MIN, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_auto_point2_pwm, 0644, show_pwm,
			    set_pwm, MAX, 2);
static SENSOR_DEVICE_ATTR_2(pwm3_stall_disable, 0644,
			    show_stall_disable, set_stall_disable, 0, 2);

static SENSOR_DEVICE_ATTR(temp4_input, 0444, show_smb_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp5_input, 0444, show_smb_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp6_input, 0444, show_smb_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(temp7_input, 0444, show_smb_temp, NULL, 3);
static SENSOR_DEVICE_ATTR(temp8_input, 0444, show_smb_temp, NULL, 4);
static SENSOR_DEVICE_ATTR(temp9_input, 0444, show_smb_temp, NULL, 5);
static SENSOR_DEVICE_ATTR(temp10_input, 0444, show_smb_temp, NULL, 6);
static SENSOR_DEVICE_ATTR(temp11_input, 0444, show_smb_temp, NULL, 7);

static SENSOR_DEVICE_ATTR(temp_smb_crit, 0644, show_smb_crit_temp,
			  set_smb_crit_temp, 0);
static SENSOR_DEVICE_ATTR(temp_smb_tmin, 0644, show_smb_tmin,
			  set_smb_tmin, 0);

static SENSOR_DEVICE_ATTR(therm_sources, 0644, show_therm_sources,
			  set_therm_sources, 0);

static DEVICE_ATTR_RW(vrm);
static DEVICE_ATTR_RO(cpu0_vid);

static struct attribute *nct7491_attrs[] = {
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in1_alarm.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_max.dev_attr.attr,
	&sensor_dev_attr_in2_min.dev_attr.attr,
	&sensor_dev_attr_in2_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_fault.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_offset.dev_attr.attr,
	&sensor_dev_attr_temp1_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_temp1_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_temp1_crit.dev_attr.attr,
	&sensor_dev_attr_temp1_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_temp1_smoothing.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp2_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp2_min.dev_attr.attr,
	&sensor_dev_attr_temp2_offset.dev_attr.attr,
	&sensor_dev_attr_temp2_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_temp2_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_temp2_crit.dev_attr.attr,
	&sensor_dev_attr_temp2_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_temp2_smoothing.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp3_fault.dev_attr.attr,
	&sensor_dev_attr_temp3_alarm.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_temp3_min.dev_attr.attr,
	&sensor_dev_attr_temp3_offset.dev_attr.attr,
	&sensor_dev_attr_temp3_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_temp3_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_temp3_crit.dev_attr.attr,
	&sensor_dev_attr_temp3_crit_hyst.dev_attr.attr,
	&sensor_dev_attr_temp3_smoothing.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_min.dev_attr.attr,
	&sensor_dev_attr_fan1_alarm.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan2_min.dev_attr.attr,
	&sensor_dev_attr_fan2_alarm.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan3_min.dev_attr.attr,
	&sensor_dev_attr_fan3_alarm.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm1_freq.dev_attr.attr,
	&sensor_dev_attr_pwm1_enable.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_channels_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point2_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_stall_disable.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm3_freq.dev_attr.attr,
	&sensor_dev_attr_pwm3_enable.dev_attr.attr,
	&sensor_dev_attr_pwm3_auto_channels_temp.dev_attr.attr,
	&sensor_dev_attr_pwm3_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm3_auto_point2_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm3_stall_disable.dev_attr.attr,
	&sensor_dev_attr_temp4_input.dev_attr.attr,
	&sensor_dev_attr_temp5_input.dev_attr.attr,
	&sensor_dev_attr_temp6_input.dev_attr.attr,
	&sensor_dev_attr_temp7_input.dev_attr.attr,
	&sensor_dev_attr_temp8_input.dev_attr.attr,
	&sensor_dev_attr_temp9_input.dev_attr.attr,
	&sensor_dev_attr_temp10_input.dev_attr.attr,
	&sensor_dev_attr_temp11_input.dev_attr.attr,
	&sensor_dev_attr_temp_smb_crit.dev_attr.attr,
	&sensor_dev_attr_temp_smb_tmin.dev_attr.attr,
	&sensor_dev_attr_therm_sources.dev_attr.attr,

	NULL,
};

static struct attribute *fan4_attrs[] = {
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan4_min.dev_attr.attr,
	&sensor_dev_attr_fan4_alarm.dev_attr.attr,
	NULL
};

static struct attribute *pwm2_attrs[] = {
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm2_freq.dev_attr.attr,
	&sensor_dev_attr_pwm2_enable.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_channels_temp.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_point2_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm2_stall_disable.dev_attr.attr,
	NULL
};

static struct attribute *in0_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_max.dev_attr.attr,
	&sensor_dev_attr_in0_min.dev_attr.attr,
	&sensor_dev_attr_in0_alarm.dev_attr.attr,
	NULL
};

static struct attribute *in_attrs[] = {
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in3_max.dev_attr.attr,
	&sensor_dev_attr_in3_min.dev_attr.attr,
	&sensor_dev_attr_in3_alarm.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_max.dev_attr.attr,
	&sensor_dev_attr_in4_min.dev_attr.attr,
	&sensor_dev_attr_in4_alarm.dev_attr.attr,
	NULL
};

static struct attribute *in_vtt_attrs[] = {
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in5_max.dev_attr.attr,
	&sensor_dev_attr_in5_min.dev_attr.attr,
	&sensor_dev_attr_in5_alarm.dev_attr.attr,
	NULL
};

static struct attribute *vid_attrs[] = {
	&dev_attr_cpu0_vid.attr,
	&dev_attr_vrm.attr,
	NULL
};

static const struct attribute_group nct7491_attr_group = {
	.attrs = nct7491_attrs
};
static const struct attribute_group fan4_attr_group = { .attrs = fan4_attrs };
static const struct attribute_group pwm2_attr_group = { .attrs = pwm2_attrs };
static const struct attribute_group in_2p5v_attr_group = { .attrs = in0_attrs };
static const struct attribute_group in_vtt_attr_group = {
	.attrs = in_vtt_attrs
};
static const struct attribute_group in_attr_group = { .attrs = in_attrs };
static const struct attribute_group vid_attr_group = { .attrs = vid_attrs };

static int nct7491_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int vendid, devid;
	const char *name;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	vendid = nct7491_read(REG_VENDID);
	devid = nct7491_read(REG_DEVID);
	if (vendid != ONSEMI_VENDID || devid != NCT7491_DEVID)
		return -ENODEV;

	name = "nct7491";
	strlcpy(info->type, name, I2C_NAME_SIZE);

	dev_dbg(&adapter->dev,
		"Detected an NCT7491 part at 0x%02x\n",
		 (unsigned int)client->addr);

	return 0;
}

static void nct7491_remove_files(struct i2c_client *client,
				 struct nct7491_data *data)
{
	sysfs_remove_group(&client->dev.kobj, &nct7491_attr_group);
	sysfs_remove_group(&client->dev.kobj, &in_attr_group);
	if (data->has_fan4)
		sysfs_remove_group(&client->dev.kobj, &fan4_attr_group);
	if (data->has_pwm2)
		sysfs_remove_group(&client->dev.kobj, &pwm2_attr_group);
	if (data->has_2p5v)
		sysfs_remove_group(&client->dev.kobj, &in_2p5v_attr_group);
	if (data->has_pch)
		sysfs_remove_group(&client->dev.kobj, &in_vtt_attr_group);


}

static int nct7491_create_files(struct i2c_client *client,
				struct nct7491_data *data)
{
	int ret;

	ret = sysfs_create_group(&client->dev.kobj, &nct7491_attr_group);
	if (ret)
		return ret;

	/* Features that can be disabled individually */
	if (data->has_fan4) {
		ret = sysfs_create_group(&client->dev.kobj, &fan4_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_pwm2) {
		ret = sysfs_create_group(&client->dev.kobj, &pwm2_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_2p5v) {
		ret = sysfs_create_group(&client->dev.kobj,
					 &in_2p5v_attr_group);
		if (ret)
			goto eremove;
	}
	if (data->has_pch) {
		ret = sysfs_create_group(&client->dev.kobj, &in_vtt_attr_group);
		if (ret)
			goto eremove;
	}
	ret = sysfs_create_group(&client->dev.kobj, &in_attr_group);
	if (ret)
		goto eremove;


	data->hwmon_dev = hwmon_device_register_with_info(&client->dev,
							  client->name,
							  data, NULL, NULL);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		goto eremove;
	}

	return 0;

eremove:
	nct7491_remove_files(client, data);
	return ret;
}

static char *status_reg_name(int reg)
{
	switch (reg) {
	case REG_SMBUS_STATUS_NACK:
		return "NACK";
	case REG_SMBUS_STATUS_PEC:
		return "PEC";
	case REG_SMBUS_STATUS_TOE:
		return "Timeout";
	case REG_SMBUS_STATUS_RANGE:
		return "Range";
	case REG_SMBUS_STATUS_TIV:
		return "Temperature Invalid";
	case REG_SMBUS_STATUS_THERM:
		return "THERM";
	default:
		WARN_ON(1);
		return "";
	};
}

static void decode_smb_dev_error_status(struct i2c_client *client,
					u8 status_reg, u8 status)
{
	int j;
	struct nct7491_data *data = i2c_get_clientdata(client);

	for (j = 0; j < 8; j++) {
		if (status & (1 << j)) {
			dev_warn(&client->dev,
				"SMBus sensor error. device%d  (bus address: 0x%02x, reg: 0x%02x) Error: %s",
				j,
				data->smb_sensor[j].bus_addr,
				data->smb_sensor[j].reg,
				status_reg_name(status_reg));
		}
	}
}

static int update_smb_sensors(struct i2c_client *client)
{
	int i;
	u8 status_reg, status;
	struct nct7491_data *data = i2c_get_clientdata(client);

	/* read all the status registers */
	for (i = 0; i < 6; i++) {
		status_reg = REG_SMBUS_STATUS_BASE + i;
		status = nct7491_read(status_reg);
		if (!status)
			continue; /* all clear */

		/* status register indicated error */
		decode_smb_dev_error_status(client, status_reg, status);

		if (status_reg == REG_SMBUS_STATUS_TOE ||
		    status_reg == REG_SMBUS_STATUS_PEC) {
			return -1;
		}
	}

	for (i = 0; i < NCT7491_SMBUS_SLAVE_COUNT; i++) {
		if (data->smb_sensor[i].bus_addr)
			data->smb_sensor[i].value =
				nct7491_read(SMB_SLAVE_TEMP_REG(i));
	}

	data->smb_therm = nct7491_read(REG_SMBUS_LIMIT_THERM);
	data->smb_tmin = nct7491_read(REG_SMBUS_TMIN);

	return 0;
}

/* SMBALERT / PWM2 pin mux config. Defaults to PWM2 output. */
static void config_mux_pwm2_alert(struct i2c_client *client,
				  struct nct7491_data *priv)
{
	int ret;
	u8 reg;
	const char *fw_val, *fw_prop = "mux-alert-pwm2";
	const char *const mux_funcs[] = {"pwm2", "smbalert"};

	ret = device_property_read_string(&client->dev, fw_prop, &fw_val);
	if (ret)
		fw_val = mux_funcs[0];
	else {
		ret = match_string(mux_funcs, ARRAY_SIZE(mux_funcs), fw_val);
		if (ret < 0) {
			dev_info(&client->dev,
				 "fw has unknown value [%s] for property %s",
				 fw_val, fw_prop);
			fw_val = mux_funcs[0];
		}
	}

	/* REG_CONFIG3 <0> : PWM2 = 0; ALERT = 1; */
	reg = nct7491_read(REG_CONFIG3);
	reg &= 0xFE;

	if (!strcmp(fw_val, "smbalert"))
		reg |= 0x01;

	i2c_smbus_write_byte_data(client, REG_CONFIG3, reg);
}

/* 2.5V / THERM pin mux config. Defaults to 2.5V sensing */
static void config_mux_therm_2p5v(struct i2c_client *client,
				  struct nct7491_data *priv)
{
	int ret;
	u8 reg;
	const char *fw_val, *fw_prop = "mux-therm-2p5v";
	const char *const mux_funcs[] = {"2p5v", "therm"};

	ret = device_property_read_string(&client->dev, fw_prop, &fw_val);

	if (ret)
		fw_val = mux_funcs[0];
	else {
		ret = match_string(mux_funcs, ARRAY_SIZE(mux_funcs), fw_val);
		if (ret < 0) {
			dev_info(&client->dev,
				 "fw has unknown value [%s] for property %s",
				 fw_val, fw_prop);
			fw_val = mux_funcs[0];
		}
	}

	/* REG_CONFIG3 <1> : 2.5V = 0; THERM = 1; */
	reg = nct7491_read(REG_CONFIG3);
	reg &= 0xFD;

	if (!strcmp(fw_val, mux_funcs[1])) {
		priv->has_2p5v = 0;
		reg |= 0x2;	/* THERM */
	} else {
		priv->has_2p5v = 1;
	}

	i2c_smbus_write_byte_data(client, REG_CONFIG3, reg);

	dev_dbg(&client->dev, "config3: 0x%02x. mux pin 22/19 function: %s",
		reg, (reg & 0x02) ? "THERM" : "2.5V");
}

#if defined(DEBUG)
static const char *decode_pin14func_mux(u8 val)
{
	val &= 0x03;
	switch (val) {
	case 0: return "TACH4";
	case 1:	return "THERM";
	case 2:	return "SMBALERT";
	case 3: return "RESERVED!";	/* we should never see this!? */
	}
	return "";
}
#else
static const char *decode_pin14func_mux(u8 val)
{
	return "";
}
#endif

/* TACH4 / THERM / SMBALERT mux config for pin 14 QSOP. Defaults to TACH4 */
static void config_mux_pin14func(struct i2c_client *client,
				 struct nct7491_data *priv)
{
	int ret;
	u8 reg;
	const char *fw_val, *fw_prop = "mux-pin14func";
	const char *const mux_funcs[] = {"tach4", "therm", "smbalert"};

	ret = device_property_read_string(&client->dev, fw_prop, &fw_val);
	if (ret)
		fw_val = mux_funcs[0];
	else {
		ret = match_string(mux_funcs, ARRAY_SIZE(mux_funcs), fw_val);
		if (ret < 0) {
			dev_info(&client->dev,
				 "fw has unknown value [%s] for property %s",
				 fw_val, fw_prop);
			fw_val = mux_funcs[0];
		}
	}

	reg = nct7491_read(REG_CONFIG4);
	reg &= 0xFC;

	if (!strcmp(fw_val, mux_funcs[1]))
		reg |= 0x1;	/* THERM */
	else if (!strcmp(fw_val, mux_funcs[2]))
		reg |= 0x2;	/* SMBALERT */
	else
		priv->has_fan4 = 1;

	i2c_smbus_write_byte_data(client, REG_CONFIG4, reg);

	dev_dbg(&client->dev, "config4: 0x%02x. muxed pin 14/11 function: %s",
		reg, decode_pin14func_mux(reg));
}

#if defined(DEBUG)
static const char *decode_pin19func_mux(u8 val)
{
	val &= 0x0C;
	val = val >> 2;
	switch (val) {
	case 0: return "SMBALERT";
	case 1:	return "THERM";
	case 2: return "GPIO3";
	case 3: return "RESERVED!";	/* we should never see this?! */
	}

	return "";
}
#else
static const char *decode_pin19func_mux(u8 val)
{
	return "";
}
#endif

/* SMBALERT / THERM / GPIO3 mux config for pin 19 QSOP. Defaults to THERM */
static void config_mux_pin19func(struct i2c_client *client)
{
	int ret;
	u8 reg;
	const char *fw_val, *fw_prop = "mux-pin19func";
	const char *const mux_funcs[] = {"smbalert", "therm", "gpio"};

	ret = device_property_read_string(&client->dev, fw_prop, &fw_val);
	if (ret)
		fw_val = mux_funcs[1];
	else {
		ret = match_string(mux_funcs, ARRAY_SIZE(mux_funcs), fw_val);
		if (ret < 0) {
			dev_info(&client->dev,
				 "fw has unknown value [%s] for property %s",
				 fw_val, fw_prop);
			fw_val = mux_funcs[1];
		}
	}

	reg = nct7491_read(REG_CONFIG5);
	reg &= 0xF3;

	if (!strcmp(fw_val, mux_funcs[1]))
		reg |= 0x4;	/* THERM */
	else if (!strcmp(fw_val, mux_funcs[2]))
		reg |= 0x8;	/* GPIO */

	i2c_smbus_write_byte_data(client, REG_CONFIG5, reg);

	dev_dbg(&client->dev,
		"config5 after: 0x%02x. muxed pin 19/16 function: %s",
		reg, decode_pin19func_mux(reg));
}


static int nct7491_smbus_sensor_init(struct i2c_client *client,
				     struct smb_sensor *sensor,
				     struct fwnode_handle *fw_node)
{
	int ret;
	const char *format;

	/* required properties */
	if (!fwnode_property_present(fw_node, "i2c-address")) {
		dev_warn(&client->dev,
			 "firmware must define i2c-address. skipping sensor.");
		goto eclear;
	}
	ret = fwnode_property_read_u8_array(fw_node, "i2c-address",
					    &sensor->bus_addr, 1);
	if (ret) {
		dev_warn(&client->dev,
			 "error reading i2c-address property: %d", ret);
		goto eclear;
	}

	if (!fwnode_property_present(fw_node, "value-register")) {
		dev_warn(&client->dev,
			 "firmware must define value-register. skipping sensor.");
		goto eclear;
	}
	ret = fwnode_property_read_u8_array(fw_node, "value-register",
					    &sensor->reg, 1);
	if (ret) {
		dev_warn(&client->dev,
			 "error reading value-register property: %d", ret);
		goto eclear;
	}

	/* optional properties */
	fwnode_property_read_string(fw_node, "label", &sensor->name);
	fwnode_property_read_u8_array(fw_node, "value-offset",
				      &sensor->offset, 1);
	fwnode_property_read_string(fw_node, "value-format", &format);

	if (!strcmp(format, "pch-block"))
		sensor->format = TEMP_FORMAT_PCH_BLOCK;
	else if (!strcmp(format, "twoscomp"))
		sensor->format = TEMP_FORMAT_TWOSCOMP;
	else if (!strcmp(format, "jedec-spd"))
		sensor->format = TEMP_FORMAT_JEDEC_SPD;
	else if (!strcmp(format, "unsigned-binary"))
		sensor->format = TEMP_FORMAT_UNSIGNED;
	else
		dev_warn(&client->dev,
			 "firmware specified unknown sensor format: %s",
			 format);

	dev_dbg(&client->dev,
		"sensor i2c-address: 0x%02x, value-register : 0x%02x, format: %s, value-offset: %d, label: %s",
		sensor->bus_addr, sensor->reg, format, sensor->offset,
		sensor->name);
	return 0;

eclear:
	sensor->bus_addr = 0;
	sensor->reg = 0;
	sensor->name = "";
	sensor->offset = 0;
	sensor->format = TEMP_FORMAT_TWOSCOMP;
	return -1;
}

static void read_firmware_pwm_sources(struct i2c_client *client,
				      struct nct7491_data *priv)
{
	int i, ret;
	u8 pwm_sources[9];

	/* Configure PWM control sources */
	ret = device_property_read_u8_array(&client->dev, "pwm-control-sources",
					    pwm_sources, sizeof(pwm_sources));
	if (ret) {
		memset(pwm_sources, 0, sizeof(pwm_sources));
		pwm_sources[0] = 0x08; /* power-on default */
	}

	for (i = 0; i < 3; i++) {
		priv->pwmsrc[i] =
			pwm_sources[i * 3] |
			(pwm_sources[i * 3 + 1] << 8) |
			(pwm_sources[i * 3 + 2] << 16);
	}
}

/* FIXME: refactor this. maybe change acpi also. */
static void config_tmin(struct i2c_client *client,
			struct nct7491_data *priv)
{
	int ret;
	u8 tmin;

	/* the power-on default is 90 degrees Celsius,
	 * which seems too late to start fans at minimum speed,
	 * so we use a lower default value of 32 degrees Celsius.
	 */

	ret = device_property_read_u8_array(&client->dev, "tmin-peci",
					    &tmin, 1);
	if (ret)
		tmin = 32;
	i2c_smbus_write_byte_data(client, REG_PECI_TMIN, tmin);

	ret = device_property_read_u8_array(&client->dev, "tmin-remote1",
					    &tmin, 1);
	if (ret)
		tmin = 32;
	i2c_smbus_write_byte_data(client, TEMP_TMIN_REG(0), tmin);

	ret = device_property_read_u8_array(&client->dev, "tmin-local",
					    &tmin, 1);
	if (ret)
		tmin = 32;
	i2c_smbus_write_byte_data(client, TEMP_TMIN_REG(1), tmin);

	ret = device_property_read_u8_array(&client->dev, "tmin-remote2",
					    &tmin, 1);
	if (ret)
		tmin = 32;
	i2c_smbus_write_byte_data(client, TEMP_TMIN_REG(2), tmin);

	/* SMBus sensors share the same Tmin */
	ret = device_property_read_u8_array(&client->dev, "tmin-smbus",
					    &tmin, 1);
	if (ret)
		tmin = 32;
	i2c_smbus_write_byte_data(client, REG_SMBUS_TMIN, tmin);

	ret = device_property_read_u8_array(&client->dev, "tmin-push",
					    &tmin, 1);
	if (ret)
		tmin = 32;
	i2c_smbus_write_byte_data(client, REG_PUSH_TMIN, tmin);
}

static void config_therm_sources(struct i2c_client *client,
				 struct nct7491_data *priv)
{
	int i, ret;
	u8 therm_enabled;

	priv->therm_sources = 0;

	for (i = 0; i < ARRAY_SIZE(acpi_therm_sources); i++) {
		ret = device_property_read_u8_array(&client->dev,
						    acpi_therm_sources[i],
						    &therm_enabled, 1);
		if (ret)
			therm_enabled = 0;

		priv->therm_sources |= (therm_enabled) ? BIT(i) : 0;
	}

	write_therm_config(client, priv->therm_sources);
}

enum acpi_therm_limit {
	THERM_LIMIT_REMOTE1 = 0,
	THERM_LIMIT_LOCAL,
	THERM_LIMIT_REMOTE2,
	THERM_LIMIT_PUSH,
	THERM_LIMIT_SMBUS
};

static const char *acpi_therm_limits[] = {
	[THERM_LIMIT_REMOTE1] = "therm-limit-remote1",
	[THERM_LIMIT_LOCAL] = "therm-limit-local",
	[THERM_LIMIT_REMOTE2] = "therm-limit-remote2",
	[THERM_LIMIT_PUSH] = "therm-limit-push",
	[THERM_LIMIT_SMBUS] = "therm-limit-smbus"
};

static void config_therm_limits(struct i2c_client *client,
				struct nct7491_data *priv)
{
	int i, ret;
	u8 therm_limit;

	for (i = THERM_LIMIT_REMOTE1; i <= THERM_LIMIT_REMOTE2; i++) {
		ret = device_property_read_u8_array(&client->dev,
						    acpi_therm_limits[i],
						    &therm_limit, 1);
		if (ret)
			therm_limit = 100; /* power on default is 100 */

		priv->temp[THERM][i] = therm_limit;
		i2c_smbus_write_byte_data(client, TEMP_THERM_REG(i),
					  therm_limit);
	}

	ret = device_property_read_u8_array(&client->dev,
					    acpi_therm_limits[THERM_LIMIT_SMBUS],
					    &therm_limit, 1);
	if (ret)
		therm_limit = 100; /* power on default is 100 */

	priv->smb_therm = therm_limit;
	i2c_smbus_write_byte_data(client, REG_SMBUS_LIMIT_THERM, therm_limit);
}

/*
 * SMBUS_CONFIG5:7 switches between Intel PCH mode (default) and
 * non-Intel generic smbus mode.
 * In Intel mode, the chip uses block read mode to get PCH temperature
 * into register 0xA8 (device0) and overwrites registers 0xA9 to 0xAC
 * (device1 - device4) with DIMM temperatures.
 */
static void config_smbus_pch_mode(struct i2c_client *client,
				  struct nct7491_data *priv)
{
	int ret;
	u8 smb_config5, pch_mode;

	/* has_pch: Intel PCH mode, or Generic mode; Default to 'generic' */
	ret = device_property_read_u8_array(&client->dev, "pch-mode",
					    &pch_mode, 1);
	if (ret)
		pch_mode = 0;

	priv->has_pch = pch_mode;

	smb_config5 = nct7491_read(REG_SMBUS_CONFIG5);
	if (pch_mode)
		smb_config5 |= 0x80;
	else
		smb_config5 &= 0x7F;

	i2c_smbus_write_byte_data(client, REG_SMBUS_CONFIG5, smb_config5);
}

/* configure 2s Complement range and Temperature Offset in CONFIG5 0x7C */
static void config_format_offset_range(struct i2c_client *client,
				       struct nct7491_data *priv)
{
	int ret;
	u8 twoscomp, tempoffset, config5;

	/* Two's complement. Defaults to 'on' */
	ret = device_property_read_u8_array(&client->dev, "twoscomp-range",
					    &twoscomp, 1);
	if (ret)
		twoscomp = 1;

	priv->twoscomp = twoscomp;

	/* Temp offset; Defaults to -63°C to +64°C with 0.5°C resolution */
	ret = device_property_read_u8_array(&client->dev, "offset-range",
					    &tempoffset, 1);
	if (ret)
		tempoffset = 0;

	priv->tempoffset = tempoffset;

	config5 = nct7491_read(REG_CONFIG5);

	config5 &= 0xFE;
	if (priv->twoscomp)
		config5 |= 0x01;

	config5 &= 0xFD;
	if (priv->tempoffset)
		config5 |= 0x2;

	i2c_smbus_write_byte_data(client, REG_CONFIG5, config5);

}

static void config_smbus_sensors(struct i2c_client *client,
				 struct nct7491_data *priv)
{
	struct smb_sensor *sensor;
	struct fwnode_handle *child;
	int i;
	u8 smb_config3, smb_config4, smb_config5;

	/* initialise SMBus sensors */
	i = 0;
	device_for_each_child_node(&client->dev, child) {

		if (i == NCT7491_SMBUS_SLAVE_COUNT) {
			dev_info(&client->dev,
				 "firmware defines more sensors than device supports (%d).",
				 NCT7491_SMBUS_SLAVE_COUNT);
			break;
		}

		sensor = &priv->smb_sensor[i];

		if (nct7491_smbus_sensor_init(client, sensor, child) == 0)
			i++;
	}

	/*
	 * firmware may define fewer nodes than NCT7491_SMBUS_SLAVE_COUNT,
	 * but we have to initialise all the chip registers.
	 */

	for (i = 0; i < NCT7491_SMBUS_SLAVE_COUNT; i++) {
		i2c_smbus_write_byte_data(client, SMB_SLAVE_ADDR_REG(i),
			priv->smb_sensor[i].bus_addr);
		i2c_smbus_write_byte_data(client, SMB_SLAVE_ADDR_REG(i) + 1,
			priv->smb_sensor[i].reg);
	}

	for (i = 0; i < 4; i++)
		smb_config3 |= priv->smb_sensor[i].format;

	i2c_smbus_write_byte_data(client, REG_SMBUS_CONFIG3, smb_config3);

	for (i = 4; i < 8; i++)
		smb_config4 |= priv->smb_sensor[i].format;

	i2c_smbus_write_byte_data(client, REG_SMBUS_CONFIG4, smb_config4);

	smb_config5 = nct7491_read(REG_SMBUS_CONFIG5);
	smb_config5 |= 0x01;	/* SMBus Master enable */
	i2c_smbus_write_byte_data(client, REG_SMBUS_CONFIG5, smb_config5);
	dev_dbg(&client->dev, "SMBus Master enabled.");
}

static void config_pwm_sources(struct i2c_client *client,
			       struct nct7491_data *priv)
{
	int ret, i;
	u8 pwm_min[3];

	read_firmware_pwm_sources(client, priv);

	/*
	 * We want the minimum wpm fan speed to be independent from the pwm
	 * control source.
	 * FIXME: should pwm_min set data->pwm[MIN][i]
	 * instead of priv->temp[AUTOMIN][i] ??
	 */
	ret = device_property_read_u8_array(&client->dev, "pwm-min", pwm_min,
					    sizeof(pwm_min));
	if (ret) {
		/* power-on default is 0x80 (50% duty cycle) */
		memset(pwm_min, 0x80, sizeof(pwm_min));
		dev_dbg(&client->dev, "pwm-min not found in acpi");
	}

	for (i = 0; i < 3; i++)
		priv->temp[AUTOMIN][i] = pwm_min[i];

	/* set the chip pwm config to match firmware config */
	chip_write_pwm_src_ctl(client, priv);
}

static void do_bringup_hacks(struct i2c_client *client,
			     struct nct7491_data *priv)
{

#if 0	/* THIS MAKES THE SMBUS MASTER PROBLEM TAKE LONGER TO TRIGGER */
	/* set the SMBus Master Retry time to 8ms. */
	config6 = nct7491_read(REG_CONFIG6);
	dev_dbg(&client->dev, "config6 before: 0x%02x", config6);
	config6 &= ~(3 << 3);
	config6 |= (3 << 3); /* set the SMBus Master Retry time to 8ms. */
	dev_dbg(&client->dev, "config6 after: 0x%02x", config6);
	i2c_smbus_write_byte_data(client, REG_CONFIG6, config6);
#endif

#if 1	/* THIS MAKES THE SMBUS MASTER PROBLEM TAKE LONGER TO TRIGGER */
	/* SMBus master polling interval */
	u8 smb_master_int = nct7491_read(REG_SMBUS_RANGE_INT);

	dev_dbg(&client->dev, "master interval before: 0x%02x", smb_master_int);
	smb_master_int = 0xCC;
	dev_dbg(&client->dev, "master interval after: 0x%02x", smb_master_int);
	i2c_smbus_write_byte_data(client, REG_SMBUS_RANGE_INT, smb_master_int);
#endif

}

static int nct7491_chip_init(struct i2c_client *client,
			     struct nct7491_data *priv)
{
	/* using NCT7491 without ACPI is a valid use-case. */
	if (!has_acpi_companion(&client->dev))
		dev_dbg(&client->dev,
			"no ACPI description found. Using chip defaults.");

	config_mux_pwm2_alert(client, priv);

	/*
	 * THERM pins (14, 19, 22 QSOP) can be independently configured.
	 * pin 14 QSOP / 11 QFN -> mux-pin14func -> Config Reg 4 0x7D <1:0>
	 * pin 19 QSOP / 16 QFN -> mux-pin19func -> Config Reg 5 0x7C <3:2>
	 * pin 22 QSOP / 19 QFN -> mux-therm-2p5v -> Config Reg 3 0x78 <1>
	 */
	config_mux_therm_2p5v(client, priv);
	config_mux_pin14func(client, priv);
	config_mux_pin19func(client);

	/* TODO: voltage sensing and attenuation */
	priv->bypass_attn = 0;

	config_format_offset_range(client, priv);
	config_tmin(client, priv);

	config_therm_limits(client, priv);
	config_therm_sources(client, priv);

	config_smbus_pch_mode(client, priv);
	config_smbus_sensors(client, priv);

	config_pwm_sources(client, priv);

	/* FIXME: remove this when bring-up testing is complete. */
	do_bringup_hacks(client, priv);

	/* Start monitoring */
	i2c_smbus_write_byte_data(client, REG_CONFIG1,
				  nct7491_read(REG_CONFIG1) | 0x01);

	return 0;
}

static int nct7491_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	enum chips chip;
	static const char * const names[] = {
		[nct7491] = "NCT7491",
	};

	struct nct7491_data *data;
	int ret = 0, version, revision;
	u8 vendid, devid;

	dev_dbg(&client->dev, "%s", __func__);

	vendid = nct7491_read(REG_VENDID);
	devid = nct7491_read(REG_DEVID);
	if (vendid != ONSEMI_VENDID || devid != NCT7491_DEVID)
		return -ENODEV;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);

	if (client->dev.of_node)
		chip = (enum chips)of_device_get_match_data(&client->dev);
	else
		chip = id->driver_data;

	/* Initialize device-specific values */
	switch (chip) {
	case nct7491:
		revision = nct7491_read(REG_VERSION);
		version = (revision >> 4) & 0xf;
		revision &= 0x03; /* revision is bits <1:0> */
		break;
	default:
		revision = 0xff; /* unknown */
		version = 0xff;
	}

	nct7491_chip_init(client, data);
	nct7491_read_sensors(&client->dev);

	ret = nct7491_create_files(client, data);
	if (ret)
		return ret;


	dev_info(&client->dev, "%s device, revision %d, version %d\n",
		 names[id->driver_data], revision, version);
	dev_info(&client->dev, "%s SMBus Master mode: %s", __func__,
		data->has_pch ? "Intel PCH SMLink1" : "Generic SMBus");
	if ((data->has_2p5v) || data->has_fan4 || data->has_pwm2)
		dev_info(&client->dev, "Optional features:%s%s%s%s\n",
			 data->has_2p5v ? " in0" : "",
			 data->has_pch  ? " in5" : "",
			 data->has_fan4 ? " fan4" : "",
			 data->has_pwm2 ? " pwm2" : "");
	if (data->bypass_attn)
		dev_info(&client->dev, "Bypassing attenuators on:%s%s%s%s\n",
			 (data->bypass_attn & (1 << 0)) ? " in0" : "",
			 (data->bypass_attn & (1 << 1)) ? " in1" : "",
			 (data->bypass_attn & (1 << 3)) ? " in3" : "",
			 (data->bypass_attn & (1 << 4)) ? " in4" : "");
	return 0;
}


static int nct7491_remove(struct i2c_client *client)
{
	struct nct7491_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	nct7491_remove_files(client, data);

	return 0;
}

static struct i2c_driver nct7491_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "nct7491",
		.of_match_table = of_match_ptr(nct7491_of_match),
	},
	.probe		= nct7491_probe,
	.remove		= nct7491_remove,
	.id_table	= nct7491_id,
	.detect		= nct7491_detect,
	.address_list	= normal_i2c,
};

static void nct7491_read_hystersis(struct i2c_client *client)
{
	struct nct7491_data *data = i2c_get_clientdata(client);

	data->temp[HYSTERSIS][0] = (u16) nct7491_read(REG_REMOTE1_HYSTERSIS);
	data->temp[HYSTERSIS][1] = data->temp[HYSTERSIS][0];
	data->temp[HYSTERSIS][2] = (u16) nct7491_read(REG_REMOTE2_HYSTERSIS);
}

static void nct7491_read_pwm(struct i2c_client *client, int index)
{
	struct nct7491_data *data = i2c_get_clientdata(client);
	int i, sources = 0;

	data->pwm[INPUT][index] = nct7491_read(PWM_REG(index));

	for (i = 0; i < 3; i++) {
		int s = nct7491_read(PWM_SRC_CTL_REG(index) + i);

		sources |= (s << (i * 8));
	}
	data->pwmsrc[index] = sources;

	if (!sources && data->pwm[INPUT][index] == 0xff)
		data->pwmctl[index] = 0;
	else if (!sources)
		data->pwmctl[index] = 1;
	else if (sources && data->pwmctl[index] != 2)
		data->pwmctl[index] = 3;
	else
		data->pwmctl[index] = 2;

	dev_dbg(&client->dev, "pwm%d duty:%d, sources: 0x%06x, control:%d",
		index+1, data->pwm[INPUT][index], sources, data->pwmctl[index]);
}

static struct nct7491_data *nct7491_read_sensors(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7491_data *data = i2c_get_clientdata(client);
	u16 ext;
	int i;
	u8 config4, config5;

	mutex_lock(&data->lock);

	update_smb_sensors(client);

	/* Measurement values update every 2 seconds */
	if (time_after(jiffies, data->measure_updated + HZ * 2) ||
	    !data->valid) {
		data->alarms = nct7491_read(REG_STATUS1);
		data->alarms |= nct7491_read(REG_STATUS2) << 8;
		data->alarms |= nct7491_read(REG_STATUS3) << 16;

		ext = (nct7491_read(REG_EXTEND2) << 8) |
			nct7491_read(REG_EXTEND1);

		for (i = 0; i < NCT7491_TEMP_COUNT; i++) {
			data->temp[INPUT][i] =
				(nct7491_read(TEMP_REG(i)) << 2) |
				((ext >> ((i + 5) * 2)) & 3);
		}

		if (data->has_2p5v)
			data->voltage[INPUT][0] =
				(nct7491_read(VOLTAGE_REG(i)) << 2) | (ext & 3);
		for (i = 1; i < NCT7491_VOLTAGE_COUNT; i++) {
			data->voltage[INPUT][i] =
				(nct7491_read(VOLTAGE_REG(i)) << 2) |
				((ext >> (i * 2)) & 3);
		}

		if (data->has_pch) {
			data->alarms |= nct7491_read(REG_STATUS4) << 24;
			ext = nct7491_read(REG_EXTEND3);
			data->voltage[INPUT][5] = nct7491_read(REG_VTT) << 2 |
				((ext >> 4) & 3);
		}

		for (i = 0; i < NCT7491_TACH_COUNT; i++) {
			if (i == 3 && !data->has_fan4)
				continue;
			data->tach[INPUT][i] =
				nct7491_read_word(client, TACH_REG(i));
		}

		/* Updated by hw when in auto mode */
		for (i = 0; i < NCT7491_PWM_COUNT; i++) {
			if (i == 1 && !data->has_pwm2)
				continue;
			nct7491_read_pwm(client, i);
		}

		data->measure_updated = jiffies;
	}

	/* Limits and settings, should never change update every 60 seconds */
	if (time_after(jiffies, data->limits_updated + HZ * 60) ||
	    !data->valid) {
		config4 = nct7491_read(REG_CONFIG4);
		data->has_fan4 = ((config4 & CONFIG4_PINFUNC) == 0x0);

		config5 = nct7491_read(REG_CONFIG5);
		data->twoscomp = (config5 & CONFIG5_TWOSCOMP);
		data->tempoffset = (config5 & CONFIG5_TEMPOFFSET);

		for (i = 1; i < NCT7491_VOLTAGE_COUNT; i++) {
			/* Adjust values so they match the input precision */
			data->voltage[MIN][i] =
				nct7491_read(VOLTAGE_MIN_REG(i)) << 2;
			data->voltage[MAX][i] =
				nct7491_read(VOLTAGE_MAX_REG(i)) << 2;
		}

		if (data->has_pch) {
			data->voltage[MIN][5] = nct7491_read(REG_VTT_MIN) << 2;
			data->voltage[MAX][5] = nct7491_read(REG_VTT_MAX) << 2;
		}

		for (i = 0; i < NCT7491_TEMP_COUNT; i++) {
			/* Adjust values so they match the input precision */
			data->temp[MIN][i] =
				nct7491_read(TEMP_MIN_REG(i)) << 2;
			data->temp[MAX][i] =
				nct7491_read(TEMP_MAX_REG(i)) << 2;
			data->temp[AUTOMIN][i] =
				nct7491_read(TEMP_TMIN_REG(i)) << 2;
			data->temp[THERM][i] =
				nct7491_read(TEMP_THERM_REG(i)) << 2;
			data->temp[OFFSET][i] =
				nct7491_read(TEMP_OFFSET_REG(i));
		}
		nct7491_read_hystersis(client);

		for (i = 0; i < NCT7491_TACH_COUNT; i++) {
			if (i == 3 && !data->has_fan4)
				continue;
			data->tach[MIN][i] =
				nct7491_read_word(client, TACH_MIN_REG(i));
		}

		for (i = 0; i < NCT7491_PWM_COUNT; i++) {
			if (i == 1 && !data->has_pwm2)
				continue;
			data->pwm[MAX][i] = nct7491_read(PWM_MAX_REG(i));
			data->pwm[MIN][i] = nct7491_read(PWM_MIN_REG(i));
			/* Set the channel and control information */
			nct7491_read_pwm(client, i);
		}

		data->range[0] = nct7491_read(TEMP_TRANGE_REG(0));
		data->range[1] = nct7491_read(TEMP_TRANGE_REG(1));
		data->range[2] = nct7491_read(TEMP_TRANGE_REG(2));

		data->limits_updated = jiffies;
		data->valid = 1;
	}



	mutex_unlock(&data->lock);

	return data;
}

module_i2c_driver(nct7491_driver);

MODULE_DESCRIPTION("nct7491 driver");
MODULE_LICENSE("GPL");
