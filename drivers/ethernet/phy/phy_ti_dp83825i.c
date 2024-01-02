/*
 * Copyright 2023 Novatech Automation
 *
 * Inspiration from phy_mii.c, which is:
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_dp83825i

#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
// Test for build error resolution:
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>

#define LOG_MODULE_NAME phy_ti_dp83825i
#define LOG_LEVEL CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

struct ti_dp83825i_config {
    uint8_t phy_addr;
    const struct device *mdio_dev;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_reset_gpio)
	const struct gpio_dt_spec reset_gpio;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_interrupt_gpio)
	const struct gpio_dt_spec interrupt_gpio;
#endif
};

struct ti_dp83825i_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_mutex mutex;
	struct k_work_delayable phy_monitor_work;
};

static int phy_ti_dp83825i_read(const struct device *dev,
				uint16_t reg_addr, uint32_t *data)
{
    const struct ti_dp83825i_config *config = dev->config;
    int ret;

    ret = mdio_read(config->mdio_dev, config->phy_addr, reg_addr, (uint16_t *)data); //Why drop upper 16 bits?
    if (ret)
        return ret;

    return 0;
}

static int phy_ti_dp83825i_write(const struct device *dev,
				uint16_t reg_addr, uint32_t data)
{
    const struct ti_dp83825i_config *config = dev->config;
    int ret;

    ret = mdio_write(config->mdio_dev, config->phy_addr, reg_addr, (uint16_t)data); // Dropps upper 16 bits!
    if (ret)
        return ret;

    return 0;
}

// =========================================================
static int phy_ti_dp83825i_get_link(const struct device *dev,
					struct phy_link_state *state)
{
    return 0;
}

static int phy_ti_dp83825i_cfg_link(const struct device *dev,
					enum phy_link_speed speeds)
{
    return 0;
}
// =========================================================

static int phy_ti_dp83825i_link_cb_set(const struct device *dev,
					phy_callback_t cb, void *user_data)
{
    struct ti_dp83825i_data *data = dev->data;

    data->cb = cb;
    data->cb_data = user_data;

    // TODO: Uncomment after implemented
    // phy_ti_dp83825i_get_link(dev, &data->state);

    data->cb(dev, &data->state, data->cb_data);

    return 0;
}

// =========================================================
static int phy_ti_dp83825i_init(const struct device *dev)
{
    const struct ti_dp83825i_config *config = dev->config;
    struct ti_dp83825i_data *data = dev->data;
    int ret;

    LOG_INF("PHY ti_dp83825i init called\n");

    data->dev = dev;

    ret = k_mutex_init(&data->mutex);
    if (ret) {
        return ret;
    }

    mdio_bus_enable(config->mdio_dev);


    return 0;
}
// =========================================================

static const struct ethphy_driver_api ti_dp83825i_phy_api = {
	.get_link = phy_ti_dp83825i_get_link,
	.cfg_link = phy_ti_dp83825i_cfg_link,
	.link_cb_set = phy_ti_dp83825i_link_cb_set,
	.read = phy_ti_dp83825i_read,
	.write = phy_ti_dp83825i_write,
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_reset_gpio)
#define RESET_GPIO(n) \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, ti_reset_gpio, {0}),
#else
#define RESET_GPIO(n)
#endif /* reset gpio */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(ti_interrupt_gpio)
#define INTERRUPT_GPIO(n) \
		.interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, ti_interrupt_gpio, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif /* interrupt gpio */

// Removed:
// .phy_iface = DT_INST_ENUM_IDX(n, ti_interface_type),		\

#define TI_DP83825I_INIT(n)						\
	static const struct ti_dp83825i_config ti_dp83825i_##n##_config = {	\
		.phy_addr = DT_INST_REG_ADDR(n),					\
		.mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),			\
		RESET_GPIO(n)							\
		INTERRUPT_GPIO(n)						\
	};									\
										\
	static struct ti_dp83825i_data ti_dp83825i_##n##_data;			\
										\
	DEVICE_DT_INST_DEFINE(n, &phy_ti_dp83825i_init, NULL,			\
			&ti_dp83825i_##n##_data, &ti_dp83825i_##n##_config,	\
			POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,			\
			&ti_dp83825i_phy_api);

DT_INST_FOREACH_STATUS_OKAY(TI_DP83825I_INIT)