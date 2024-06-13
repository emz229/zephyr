/*
 * Copyright (c) 2024 Novatech LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp48xx_dac

#include <zephyr/kernel.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dac_mcp48xx, CONFIG_DAC_LOG_LEVEL);

/* Place Defines here */

struct mcp48xx_config {
	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_latch;
	uint8_t resolution;
	const uint8_t *channel_addresses;
	size_t channel_count;
	uint8_t gain;
	uint8_t power_down;
};

static int mcp48xx_channel_setup(const struct device *dev, const struct dac_channel_cfg *channel_cfg)
{
	const struct mcp48xx_config *config = dev->config;

	if (channel_cfg->channel_id >= config->channel_count) {
		LOG_ERR("invalid channel %i", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->resolution != config->resolution) {
		LOG_ERR("invalid resolution %i", channel_cfg->resolution);
		return -EINVAL;
	}

	return 0;
}

static int mcp48xx_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	const struct mcp48xx_config *config = (struct mcp48xx_config *)dev->config;
	uint8_t tx_data[3];
	int ret;
	uint8_t rx_data[ARRAY_SIZE(tx_data)];

	const struct spi_buf tx_buf[] = {{
		.buf = tx_data,
		.len = ARRAY_SIZE(tx_data),
	}};
	const struct spi_buf rx_buf[] = {{
		.buf = rx_data,
		.len = ARRAY_SIZE(rx_data),
	}};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	if (value > BIT(config->resolution) - 1) {
		LOG_ERR("invalid resolution value %i", value);
		return -EINVAL;
	}

	if (channel > config->channel_count) {
		LOG_ERR("invalid channel %i", channel);
		return -EINVAL;
	}

	LOG_ERR("Value passed in: %x", value);

	channel = channel - 1;
	tx_data[0] = 0;
	tx_data[0] = channel << 7; // Bit 15 is A / B channel
	tx_data[0] = tx_data[0] | (config->gain << 5); // Bit 13 is Gain
	tx_data[0] = tx_data[0] | (config->power_down << 4); // Bit 12 is SHDN
	tx_data[0] = tx_data[0] | (value >> 28); // Bit 11-8 is upper nibble
	tx_data[1] = 0;
	tx_data[1] = value << 4; // Bit 7-0 is lower byte
	
	LOG_ERR("tx[0] = %x/ntx[1] = %x", tx_data[0], tx_data[1]);
	
	ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret != 0) {
		LOG_ERR("spi_transceive failed with error %i", ret);
		return ret;
	}

	return 0;
}

static int dac_mcp48xx_init(const struct device *dev)
{
	const struct mcp48xx_config *config = dev->config;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}
	return 0;
}

static const struct dac_driver_api mcp48xx_driver_api = {
	.channel_setup = mcp48xx_channel_setup,
	.write_value = mcp48xx_write_value,
};

#define INST_DT_MCP48XX(inst, t) DT_INST(inst, microchip_mcp##t)

#define MCP40XX_DEVICE(t, n, res, nchan) \
	static const struct mcp48xx_config mcp##t##_config_##n = { \
	.bus = SPI_DT_SPEC_INST_GET(INST_DT_MCP48XX(n, t), \
		SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8), 0), \
	.resolution = res, \
	.channel_count = nchan, \
	}; \
	DEVICE_DT_DEFINE(INST_DT_MCP48XX(n, t), \
			&dac_mcp48xx_init, NULL, \
			NULL, \
			&mcp##t##_config_##n, POST_KERNEL, \
			CONFIG_DAC_MCP48XX_INIT_PRIORITY, \
			&mcp48xx_driver_api)

/*
 * MCP4802: 8-bit
 */
#define MCP4802_DEVICE(n) MCP48XX_DEVICE(4802, n, 8, 2)

/*
 * MCP4812: 10-bit
 */
#define MCP4812_DEVICE(n) MCP48XX_DEVICE(4812, n, 10, 2);

/*
 * MCP4822: 12-bit
 */
#define MCP4822_DEVICE(n) MCP48XX_DEVICE(4822, n, 12, 2);

#define CALL_WITH_ARG(arg, expr) expr(arg)

#define INST_DT_MCP48XX_FOREACH(t, inst_expr) \
	LISTIFY(DT_NUM_INST_STATUS_OKAY(microchip_mcp##t), \
			CALL_WITH_ARG, (), inst_expr)

INST_DT_MCP48XX_FOREACH(4802, MCP4802_DEVICE);
INST_DT_MCP48XX_FOREACH(4812, MCP4812_DEVICE);
INST_DT_MCP48XX_FOREACH(4822, MCP4822_DEVICE);

#if 0	
	.gpio_latch = GPIO_DT_SPEC_INST_GET_OR(0, latch_gpios, {0}),
	.gain = DT_INST_PROP_OR(0, gain, 0),
	.power_down = DT_INST_PROP(0, power_down_mode),
	};

DEVICE_DT_INST_DEFINE(0, &dac_mcp48xx_init, NULL, NULL, &mcp48xx_config_0,
		POST_KERNEL, CONFIG_DAC_MCP48XX_INIT_PRIORITY, &mcp48xx_driver_api);

#define DT_DRV_COMPAT microchip_mcp4802
DT_INST_FOREACH_STATUS_OKAY(DAC_MCP48XX_INST_DEFINE);
/*
 * .channel_addresses = channels,                                          \
   .channel_count = channels_count,                                        \
*/

#define DT_DRV_COMPAT microchip_mcp4802
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
/* DT_INST_FOREACH_STATUS_OKAY_VARGS(DAC_MCP48XX_INST_DEFINE) */
DT_INST_FOREACH_STATUS_OKAY(DAC_MCP48XX_INST_DEFINE)
#endif
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT microchip_mcp4812
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
/* DT_INST_FOREACH_STATUS_OKAY_VARGS(DAC_MCP48XX_INST_DEFINE) */
DT_INST_FOREACH_STATUS_OKAY(DAC_MCP48XX_INST_DEFINE)
#endif
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT microchip_mcp4822
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
/* DT_INST_FOREACH_STATUS_OK_VARGS(DAC_MCP48XX_INST_DEFINE) */
DT_INST_FOREACH_STATUS_OKAY(DAC_MCP48XX_INST_DEFINE)
#endif
#undef DT_DRV_COMPAT

#endif
