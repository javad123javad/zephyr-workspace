/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>

#include "bmp280.h"
//bmp280







////////////////////////
#define SERVICE_DATA_LEN        10		   /* Length of service data */
#define SERVICE_UUID            0xfcd2      /* BTHome service UUID */
#define IDX_TEMPL               4           /* Index of lo byte of temp in service data*/
#define IDX_TEMPH               5           /* Index of hi byte of temp in service data*/
#define IDX_PRESSL              7           /* Index of lo byte of humidity in service data*/
#define IDX_PRESSM              8           /* Index of hi byte of humidity in service data*/
#define IDX_PRESSH 				9


#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, \
				  BT_GAP_ADV_SLOW_INT_MIN, \
				  BT_GAP_ADV_SLOW_INT_MAX, NULL)


static uint8_t service_data[SERVICE_DATA_LEN] = {
    BT_UUID_16_ENCODE(SERVICE_UUID),
    0x40,
    0x02,	/* Temperature */
    0xc4,	/* Low byte */
    0x00,   /* High byte */
    0x04,	/* Pressure */
    0x13,	/* 50.55%  low byte*/
    0x8A,   /* 50.55%  high byte*/
    0x02,
};

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data))
};

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    /* Start advertising */
    err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
}

int main(void)
{
    int err;
    int temp = 0;
    const struct device *dev = check_bme280_device();
    bmp280_data_t bmp280_data = {0};
    if (dev == NULL) {
        printk("Could not find BME280 device\n");
        return 0;
    }
    printk("Found BME280 device\n");
    printk("Starting BTHome sensor template\n");

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    for (;;) {
        read_sensor_data(&bmp280_data);
        /* Simulate temperature from 0C to 25C */
        service_data[IDX_TEMPH] = (bmp280_data.temperature) >> 8;
        service_data[IDX_TEMPL] = (bmp280_data.temperature) & 0xff;
        service_data[IDX_PRESSH] = (bmp280_data.pressure) >> 16;
        service_data[IDX_PRESSM] = ((bmp280_data.pressure) >> 8)& 0x0ff;
        service_data[IDX_PRESSL] = (bmp280_data.pressure) & 0xff;
        printk("Temperature H: %d, L: %d\n", service_data[IDX_TEMPH], service_data[IDX_TEMPL]);
        err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            printk("Failed to update advertising data (err %d)\n", err);
        }
        else {
            printk("Data updated with temp: %d\n", bmp280_data.temperature);
        }
        k_sleep(K_MSEC(BT_GAP_ADV_SLOW_INT_MIN));
    }
    return 0;
}
