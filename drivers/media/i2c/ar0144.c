// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the AR0144 Image Sensor Processor
 *
 * Copyright (C) 2018 eyeSight Technologies Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define AR0144_I2C_ADDR      0x10
#define AR0144_ID_REG        0x3000
#define AR0144_ID_VAL        0x1356

struct ar0144_reg_value {
	u16 reg;
	u16 val;
};

struct ar0144 {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;

	struct gpio_desc *rst_gpio;
	struct mutex lock;

	bool streaming;
};

static inline struct ar0144 *to_ar0144(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0144, sd);
}

static const struct ar0144_reg_value ar0144at_rev4_recommended_setting[] = {
	{0x3ED6, 0x3CB5},
	{0x3ED8, 0x8765},
	{0x3EDA, 0x8888},
	{0x3EDC, 0x97FF},
	{0x3EF8, 0x6522},
	{0x3EFA, 0x2222},
	{0x3EFC, 0x6666},
	{0x3F00, 0xAA05},
	{0x3EE2, 0x180E},
	{0x3EE4, 0x0808},
	{0X3EEA, 0x2A09},
	{0x3060, 0x000D},
	{0x3092, 0x00CF},
	{0X3268, 0x0030},
	{0X3786, 0x0060},
	{0x3F4A, 0x0F70},
	{0x306E, 0x4810},
	{0x3064, 0x1802},
	{0x3EF6, 0x804D},
	{0x3180, 0xC08F},
	{0x30BA, 0x7623},
	{0x3176, 0x0480},
	{0x3178, 0x0480},
	{0x317A, 0x0480},
	{0x317C, 0x0480},
};

static const struct ar0144_reg_value ar0144at_pll_27mhz[] = {
	{0x302A, 0x0006},
	{0x302C, 0x0001},
	{0x302E, 0x0004},
	{0x3030, 0x0042},
	{0x3036, 0x000C},
	{0x3038, 0x0001},
};

static const struct ar0144_reg_value ar0144at_mipi_2lane_12bit[] = {
	{0x31AE, 0x0202},
	{0x31AC, 0x0C0C},
	{0x31B0, 0x0042},
	{0x31B2, 0x002E},
	{0x31B4, 0x1665},
	{0x31B6, 0x110E},
	{0x31B8, 0x2047},
	{0x31BA, 0x0105},
	{0x31BC, 0x0004},
};

static const struct ar0144_reg_value ar0144at_1280x800_60fps[] = {
	{0x3002, 0x0000},
	{0x3004, 0x0004},
	{0x3006, 0x031F},
	{0x3008, 0x0503},
	{0x300A, 0x0339},
	{0x300C, 0x05D0},
	{0x3012, 0x0064},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
};

static const struct ar0144_reg_value ar0144at_context_b_2x2_binning[] = {
	{0x3040, 0x1000},
	{0x30A8, 0x0003},
	{0x3040, 0x3000},
	{0x30AE, 0x0003},
};

static const struct ar0144_reg_value ar0144at_embedded_data_stats[] = {
	{0x3064, 0x1982},
};

static const struct ar0144_reg_value ar0144at_start_stream[] = {
	{0x3028, 0x0010},
	{0x301A, 0x005C},
};

static const struct ar0144_reg_value ar0144at_stop_stream[] = {
	{0x301A, 0x0058},
};

static int ar0144_write_reg(struct ar0144 *ar0144, u16 reg, u16 val)
{
	u8 regbuf[4];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val >> 8;
	regbuf[3] = val & 0xff;

	ret = i2c_master_send(ar0144->i2c_client, regbuf, 4);
	if (ret < 0)
		dev_err(&ar0144->i2c_client->dev,
			"%s: write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, val);

	return ret;
}

static int ar0144_read_reg(struct ar0144 *ar0144, u16 reg, u16 *val)
{
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	ret = i2c_master_send(ar0144->i2c_client, buf, 2);
	if (ret < 0) {
		dev_err(&ar0144->i2c_client->dev,
			"%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ar0144->i2c_client, buf, 2);
	if (ret < 0) {
		dev_err(&ar0144->i2c_client->dev,
			"%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}
	*val = buf[0] << 8;
	*val |= (buf[1] & 0xff);

	return 0;
}

static int ar0144_set_register_array(struct ar0144 *ar0144,
				     const struct ar0144_reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = ar0144_write_reg(ar0144, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ar0144_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	u16 reg_val;
	int ret = 0;

	mutex_lock(&ar0144->lock);

	gpiod_direction_output(ar0144->rst_gpio, 1);
	if (!on)
		goto out;
	msleep(2); /* more than 1ms */
	gpiod_set_value_cansleep(ar0144->rst_gpio, 0);
	msleep(10); /* more than 160000 clocks at 24MHz; FIXME: use clk rate */

	ret = ar0144_read_reg(ar0144, AR0144_ID_REG, &reg_val);
	if (ret < 0)
		goto out;
	if (reg_val != AR0144_ID_VAL) {
		dev_err(&ar0144->i2c_client->dev,
			"wrong chip id (%u), expected %u\n", reg_val,
			AR0144_ID_VAL);
		ret = -ENODEV;
		goto out;
	}

	ret = ar0144_set_register_array(
		ar0144, ar0144at_rev4_recommended_setting,
		ARRAY_SIZE(ar0144at_rev4_recommended_setting));

out:
	mutex_unlock(&ar0144->lock);
	return ret;
}

static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB12_1X12;

	return 0;
}

static int ar0144_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SRGGB12_1X12)
		return -EINVAL;

	if (fse->index >= 1)
		return -EINVAL;

	fse->min_width = 1280;
	fse->max_width = 1280;
	fse->min_height = 800;
	fse->max_height = 800;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ar0144_get_pad_format(struct ar0144 *ar0144,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0144->fmt;
	default:
		return NULL;
	}
}

static int ar0144_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ar0144 *ar0144 = to_ar0144(sd);

	mutex_lock(&ar0144->lock);
	format->format = *__ar0144_get_pad_format(ar0144, cfg, format->pad,
						  format->which);
	mutex_unlock(&ar0144->lock);
	return 0;
}

static struct v4l2_rect *
__ar0144_get_pad_crop(struct ar0144 *ar0144, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0144->crop;
	default:
		return NULL;
	}
}

static int ar0144_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	struct v4l2_mbus_framefmt *__format;

	mutex_lock(&ar0144->lock);

	__format = __ar0144_get_pad_format(ar0144, cfg, format->pad,
			format->which);
	__format->width = 1280;
	__format->height = 800;
	__format->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	mutex_unlock(&ar0144->lock);
	return 0;
}

static int ar0144_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;

	ar0144_set_format(subdev, cfg, &fmt);

	return 0;
}

static int ar0144_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	struct ar0144 *ar0144 = to_ar0144(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ar0144_get_pad_crop(ar0144, cfg, sel->pad,
					sel->which);
	return 0;
}

static int ar0144_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ar0144 *ar0144 = to_ar0144(subdev);
	int ret;

	mutex_lock(&ar0144->lock);

	if (enable == 0) {
		ret = ar0144_set_register_array(
			ar0144, ar0144at_stop_stream,
			ARRAY_SIZE(ar0144at_stop_stream));
		goto out;
	}

	ret = ar0144_set_register_array(ar0144, ar0144at_pll_27mhz,
					ARRAY_SIZE(ar0144at_pll_27mhz));
	if (ret < 0)
		goto out;
	msleep(100);

	ret = ar0144_set_register_array(ar0144, ar0144at_mipi_2lane_12bit,
					ARRAY_SIZE(ar0144at_mipi_2lane_12bit));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(ar0144, ar0144at_1280x800_60fps,
					ARRAY_SIZE(ar0144at_1280x800_60fps));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(
		ar0144, ar0144at_context_b_2x2_binning,
		ARRAY_SIZE(ar0144at_context_b_2x2_binning));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(
		ar0144, ar0144at_embedded_data_stats,
		ARRAY_SIZE(ar0144at_embedded_data_stats));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(ar0144, ar0144at_start_stream,
					ARRAY_SIZE(ar0144at_start_stream));

out:
	mutex_unlock(&ar0144->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops ar0144_core_ops = {
	.s_power = ar0144_s_power,
};

static const struct v4l2_subdev_video_ops ar0144_video_ops = {
	.s_stream = ar0144_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0144_subdev_pad_ops = {
	.init_cfg = ar0144_entity_init_cfg,
	.enum_mbus_code = ar0144_enum_mbus_code,
	.enum_frame_size = ar0144_enum_frame_size,
	.get_fmt = ar0144_get_format,
	.set_fmt = ar0144_set_format,
	.get_selection = ar0144_get_selection,
};

static const struct v4l2_subdev_ops ar0144_subdev_ops = {
	.core = &ar0144_core_ops,
	.video = &ar0144_video_ops,
	.pad = &ar0144_subdev_pad_ops,
};

static int ar0144_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct ar0144 *ar0144;
	int ret;

	ar0144 = devm_kzalloc(dev, sizeof(struct ar0144), GFP_KERNEL);
	if (!ar0144)
		return -ENOMEM;

	ar0144->i2c_client = client;
	ar0144->dev = dev;
	mutex_init(&ar0144->lock);

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ar0144->ep);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	of_node_put(endpoint);

	if (ar0144->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be parallel\n");
		return -EINVAL;
	}

	ar0144->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ar0144->rst_gpio)) {
		if (PTR_ERR(ar0144->rst_gpio) != -EPROBE_DEFER)
			dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(ar0144->rst_gpio);
	}

	v4l2_i2c_subdev_init(&ar0144->sd, client, &ar0144_subdev_ops);
	ar0144->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar0144->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ar0144->pad.flags = MEDIA_PAD_FL_SOURCE;
	ar0144->sd.dev = &client->dev;

	ret = media_entity_pads_init(&ar0144->sd.entity, 1, &ar0144->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		return ret;
	}

	ret = ar0144_s_power(&ar0144->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up AR0144\n");
		goto free_entity;
	}

	dev_info(dev, "AR0144 detected at address 0x%02x\n", client->addr);

	ret = v4l2_async_register_subdev(&ar0144->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ar0144_entity_init_cfg(&ar0144->sd, NULL);

	return 0;

free_entity:
	media_entity_cleanup(&ar0144->sd.entity);

	return ret;
}

static int ar0144_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ar0144_id[] = {
	{ "ar0144", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ar0144_id);

static const struct of_device_id ar0144_of_match[] = {
	{ .compatible = "onnn,ar0144" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0144_of_match);

static struct i2c_driver ar0144_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ar0144_of_match),
		.name  = "ar0144",
	},
	.probe  = ar0144_probe,
	.remove = ar0144_remove,
	.id_table = ar0144_id,
};

module_i2c_driver(ar0144_i2c_driver);

MODULE_DESCRIPTION("ON Semiconductor AR0144 Camera Sensor");
MODULE_LICENSE("GPL v2");
