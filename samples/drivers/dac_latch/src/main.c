/*
 * Copyright (c) 2024 Novatech LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/dac.h>

#define SLEEP_TIME_MS	100

/*
 * Get dac configuration from the devicetree dac alias. This is mandatory.
 */
#define DAC_NODE	DT_ALIAS(dac)

#if !DT_NODE_HAS_STATUS(DAC_NODE, okay)
#error "Unsupported board; dac devicetree alias it not defined"
#endif

static const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);

static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id = 0, // Values hard coded here instead of in overlay
	.resolution = 8,
	.buffered = false,
};

int main(void)
{
	int ret = 0;

	if (!device_is_ready(dac_dev)) {
		printk("DAC device %s is not ready\n", dac_dev->name);
		return 0;
	}
	
	ret = dac_channel_setup(dac_dev, &dac_ch_cfg);

	if (ret != 0) {
		printk("Setting up of DAC channel failed with code %d\n", ret);
		return 0;
	}

	int i = 0;

	while (1) {
		/* Update CH0 buffer */
        	ret = dac_update_value(dac_dev, 0, i);
        	if (ret != 0) {
                	printk("dac_update_value() failed with code %d\n", ret);
        	}
		/* Update CH1 buffer */
		ret = dac_update_value(dac_dev, 1, i);
		if (ret != 0) {
			printk("dac_update_value() failed with code %d\n", ret);
		}
                /* Apply buffer to hardware */
                ret = dac_write_value(dac_dev, DAC_WRITE_ALL_CH, NULL);
		if (ret != 0) {
			printk("dac_write_value() failed with code %d\n", ret);
		}
		if (ret == -EINVAL) {
			i = 0;
		}
		else {
			i = i + 1;
		}
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
