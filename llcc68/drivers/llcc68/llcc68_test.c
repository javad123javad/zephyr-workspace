#include <zephyr/ztest.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include "llcc68.h"
#include "llcc68_internal.h"
#include "llcc68_regs.h"

static const struct device *llcc68_dev;

/* Mock functions for testing */
static int mock_spi_transceive(const struct spi_dt_spec *spec,
                             const struct spi_buf_set *tx_bufs,
                             const struct spi_buf_set *rx_bufs)
{
    /* Simple mock that copies tx data to rx buffers */
    if (tx_bufs && rx_bufs && tx_bufs->count > 0 && rx_bufs->count > 0) {
        for (size_t i = 0; i < rx_bufs->count; i++) {
            if (i < tx_bufs->count && rx_bufs->buffers[i].buf && tx_bufs->buffers[i].buf) {
                size_t len = MIN(rx_bufs->buffers[i].len, tx_bufs->buffers[i].len);
                memcpy(rx_bufs->buffers[i].buf, tx_bufs->buffers[i].buf, len);
            }
        }
    }
    return 0;
}

static void test_setup(void)
{
    llcc68_dev = llcc68_get_device();
    zassert_not_null(llcc68_dev, "LLCC68 device not found");
}

static void test_spi_communication(void)
{
    uint8_t test_data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    uint8_t read_data[10];
    int ret;

    /* Test register write/read */
    ret = llcc68_reg_write(llcc68_dev, LLCC68_REG_SYNC_WORD_0, test_data, 1);
    zassert_equal(ret, 0, "Register write failed");

    ret = llcc68_reg_read(llcc68_dev, LLCC68_REG_SYNC_WORD_0, read_data, 1);
    zassert_equal(ret, 0, "Register read failed");
    zassert_equal(read_data[0], test_data[0], "Register data mismatch");
}

static void test_state_transitions(void)
{
    int ret;

    /* Test state transitions */
    ret = llcc68_set_state(llcc68_dev, LLCC68_STATE_SLEEP);
    zassert_equal(ret, 0, "Failed to set sleep state");

    ret = llcc68_set_state(llcc68_dev, LLCC68_STATE_STANDBY_RC);
    zassert_equal(ret, 0, "Failed to set standby RC state");

    ret = llcc68_set_state(llcc68_dev, LLCC68_STATE_STANDBY_XOSC);
    zassert_equal(ret, 0, "Failed to set standby XOSC state");

    ret = llcc68_set_state(llcc68_dev, LLCC68_STATE_FS);
    zassert_equal(ret, 0, "Failed to set FS state");
}

static void test_lora_config(void)
{
    struct llcc68_modem_config config;
    int ret;

    /* Configure LoRa parameters */
    config.frequency = 868000000;
    config.bandwidth = LLCC68_LORA_BW_125_KHZ;
    config.datarate = LLCC68_LORA_SF_7;
    config.coding_rate = LLCC68_LORA_CR_4_5;
    config.preamble_len = 8;
    config.implicit_header = false;
    config.crc_enable = true;
    config.tx_power = 14;

    ret = llcc68_set_lora_config(llcc68_dev, &config);
    zassert_equal(ret, 0, "Failed to configure LoRa parameters");
}

static void test_buffer_operations(void)
{
    uint8_t test_data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    uint8_t read_data[10];
    int ret;

    /* Test buffer write/read */
    ret = llcc68_write_buffer(llcc68_dev, 0, test_data, sizeof(test_data));
    zassert_equal(ret, 0, "Buffer write failed");

    ret = llcc68_read_buffer(llcc68_dev, 0, read_data, sizeof(read_data));
    zassert_equal(ret, 0, "Buffer read failed");

    /* Verify data */
    for (int i = 0; i < sizeof(test_data); i++) {
        zassert_equal(read_data[i], test_data[i], "Buffer data mismatch at index %d", i);
    }
}

static void test_send_receive(void)
{
    uint8_t test_data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    uint8_t read_data[10];
    uint8_t len;
    int ret;

    /* Test send operation */
    ret = llcc68_send(llcc68_dev, test_data, sizeof(test_data));
    zassert_equal(ret, 0, "Send operation failed");

    /* Simulate receive operation */
    struct llcc68_data *data = llcc68_dev->data;
    memcpy(data->rx_buffer, test_data, sizeof(test_data));
    data->rx_len = sizeof(test_data);

    /* Test receive operation */
    ret = llcc68_receive(llcc68_dev, read_data, &len);
    zassert_equal(ret, 0, "Receive operation failed");
    zassert_equal(len, sizeof(test_data), "Received data length mismatch");

    /* Verify data */
    for (int i = 0; i < len; i++) {
        zassert_equal(read_data[i], test_data[i], "Received data mismatch at index %d", i);
    }
}

void test_main(void)
{
    ztest_test_suite(llcc68_tests,
                    ztest_unit_test_setup_teardown(test_spi_communication, test_setup, NULL),
                    ztest_unit_test_setup_teardown(test_state_transitions, test_setup, NULL),
                    ztest_unit_test_setup_teardown(test_lora_config, test_setup, NULL),
                    ztest_unit_test_setup_teardown(test_buffer_operations, test_setup, NULL),
                    ztest_unit_test_setup_teardown(test_send_receive, test_setup, NULL));

    ztest_run_test_suite(llcc68_tests);
}
