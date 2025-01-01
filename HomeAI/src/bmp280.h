#ifndef _BMP280_H_
#define _BMP280_H_

#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/dsp/print_format.h>

typedef struct bmp280_data
{
    int temperature;
    int pressure;
}bmp280_data_t;

extern const struct device *const dev;
const struct device *check_bme280_device(void);
int read_sensor_data(bmp280_data_t *bmp280_data);

#endif