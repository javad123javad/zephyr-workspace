#include <zephyr/kernel.h>
#include "../drivers/llcc68.h"

void main(void) {
    const struct device *llcc68_dev;
    struct llcc68_modem_config config;

    /* Get LLCC68 device */
    llcc68_dev = llcc68_get_device();

    /* Configure LoRa parameters */
    config.frequency = 868000000;
    config.bandwidth = LLCC68_LORA_BW_125_KHZ;
    config.datarate = LLCC68_LORA_SF_7;
    config.coding_rate = LLCC68_LORA_CR_4_5;
    config.preamble_len = 8;
    config.crc_enable = true;

    llcc68_set_lora_config(llcc68_dev, &config);
}