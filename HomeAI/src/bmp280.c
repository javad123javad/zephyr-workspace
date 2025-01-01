#include "bmp280.h"
// 


/*
   * Get a device structure from a devicetree node with compatible
   * "bosch,bme280". (If there are multiple, just pick one.)
   */
const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme280);
SENSOR_DT_READ_IODEV(iodev, DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bme280),
                     [0]= {SENSOR_CHAN_AMBIENT_TEMP, 0},
                     [1]= {SENSOR_CHAN_HUMIDITY, 0},
                     [2]= {SENSOR_CHAN_PRESS, 0});

RTIO_DEFINE(ctx, 1, 1);

const struct device *check_bme280_device(void)
{
    if (dev == NULL) {
        /* No such node, or the node does not have status "okay". */
        printk("\nError: no device found.\n");
        return NULL;
    }

    if (!device_is_ready(dev)) {
        printk("\nError: Device \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }

    printk("Found device \"%s\", getting sensor data\n", dev->name);
    return dev;
}

int read_sensor_data(bmp280_data_t *bmp280_data)
{
    uint8_t buf[128];

    int rc = sensor_read(&iodev, &ctx, buf, 128);

    if (rc != 0) {
        printk("%s: sensor_read() failed: %d\n", dev->name, rc);
        return rc;
    }

    const struct sensor_decoder_api *decoder;

    rc = sensor_get_decoder(dev, &decoder);

    if (rc != 0) {
        printk("%s: sensor_get_decode() failed: %d\n", dev->name, rc);
        return rc;
    }

    uint32_t temp_fit = 0;
    struct sensor_q31_data temp_data = {0};

    decoder->decode(buf,
    (struct sensor_chan_spec) {
        SENSOR_CHAN_AMBIENT_TEMP, 0
    },
    &temp_fit, 1, &temp_data);

    uint32_t press_fit = 0;
    struct sensor_q31_data press_data = {0};

    decoder->decode(buf,
    (struct sensor_chan_spec) {
        SENSOR_CHAN_PRESS, 0
    },
    &press_fit, 1, &press_data);

    printk("temp: %s%d.%d; press: %s%d.%d\n",
           PRIq_arg(temp_data.readings[0].temperature, 6, temp_data.shift),
           PRIq_arg(press_data.readings[0].pressure, 6, press_data.shift));
    char temp[10] = {0};
    sprintf(temp, "%s%d.%d", PRIq_arg(temp_data.readings[0].temperature, 2, temp_data.shift));
    bmp280_data->temperature = atof(temp)*100;
    char press[10] = {0};
    sprintf(press, "%s%d.%d", PRIq_arg(press_data.readings[0].pressure, 3, press_data.shift));
    bmp280_data->pressure = atof(press)*1000;
    //k_sleep(K_MSEC(1000));

	return 0;

}