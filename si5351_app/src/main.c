#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "si5351.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define I2C_DEV DT_LABEL(DT_NODELABEL(i2c0))
#define SI5351_I2C_ADDR 0x60

static void i2c_scan(const struct device *i2c_dev) {
    LOG_INF("Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t dummy = 0;
        if (i2c_write(i2c_dev, &dummy, 0, addr) == 0) {
            LOG_INF("Device found at address 0x%02x", addr);
        }
    }
}

int main(void) {
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!i2c_dev) {
        LOG_ERR("I2C device not found");
        return;
    }

    int ret = i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER);
    if (ret) {
        LOG_ERR("I2C configure error: %d", ret);
        return;
    }

    i2c_scan(i2c_dev);

    struct si5351_dev si_dev = {
        .i2c_dev = i2c_dev,
        .i2c_addr = SI5351_I2C_ADDR,
    };

    ret = si5351_init(&si_dev, 970);
    if (ret) {
        LOG_ERR("Si5351 init failed: %d", ret);
        return -1; // Stop here if init fails
    }
    k_msleep(10);

    ret = si5351_setup_clk0(&si_dev, 1000000, SI5351_DRIVE_STRENGTH_8MA);
    if (ret) {
        LOG_ERR("CLK0 setup failed: %d", ret);
        return -1;
    }
    k_msleep(10);

    ret = si5351_setup_clk2(&si_dev, 25000000, SI5351_DRIVE_STRENGTH_4MA);
    if (ret) {
        LOG_ERR("CLK2 setup failed: %d", ret);
        return-1;
    }
    k_msleep(10);

    ret = si5351_enable_outputs(&si_dev, (1 << 0) | (1 << 2));
    if (ret) {
        LOG_ERR("Enable outputs failed: %d", ret);
        return -1;
    }

    LOG_INF("Si5351 configured: CLK0 @ 10 MHz, CLK2 @ 25 MHz");

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}