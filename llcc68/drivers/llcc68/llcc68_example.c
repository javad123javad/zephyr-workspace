#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "llcc68.h"

#define TX_INTERVAL_MS 5000

static const struct device *llcc68_dev;
static struct k_work_delayable tx_work;
static uint8_t tx_counter;
static bool rx_mode = false;

/* Event handler for LLCC68 interrupts */
static void event_handler(const struct device *dev, uint16_t events)
{
    if (events & LLCC68_IRQ_TX_DONE) {
        printk("TX done\n");
        
        /* Switch to RX mode after transmission */
        llcc68_set_rx(dev, 0);
    }

    if (events & LLCC68_IRQ_RX_DONE) {
        uint8_t data[LLCC68_MAX_PACKET_LEN];
        uint8_t len;
        int16_t rssi;
        int8_t snr;
        
        /* Get received data */
        llcc68_receive(dev, data, &len);
        llcc68_get_rssi(dev, &rssi);
        llcc68_get_snr(dev, &snr);
        
        printk("RX done: %d bytes, RSSI: %d dBm, SNR: %d dB\n", len, rssi, snr);
        
        /* Print received data */
        for (int i = 0; i < len; i++) {
            printk("%02X ", data[i]);
        }
        printk("\n");
    }

    if (events & LLCC68_IRQ_PREAMBLE_DETECTED) {
        printk("Preamble detected\n");
    }

    if (events & LLCC68_IRQ_SYNC_WORD_VALID) {
        printk("Sync word valid\n");
    }

    if (events & LLCC68_IRQ_HEADER_VALID) {
        printk("Header valid\n");
    }

    if (events & LLCC68_IRQ_HEADER_ERR) {
        printk("Header error\n");
    }

    if (events & LLCC68_IRQ_CRC_ERR) {
        printk("CRC error\n");
    }

    if (events & LLCC68_IRQ_CAD_DONE) {
        printk("CAD done\n");
    }

    if (events & LLCC68_IRQ_CAD_DETECTED) {
        printk("CAD detected\n");
    }

    if (events & LLCC68_IRQ_TIMEOUT) {
        printk("Timeout\n");
    }
}

/* Work handler for periodic transmission */
static void tx_work_handler(struct k_work *work)
{
    uint8_t data[10];
    
    /* Prepare data packet */
    data[0] = tx_counter++;
    for (int i = 1; i < 10; i++) {
        data[i] = i;
    }
    
    printk("Sending packet #%d\n", data[0]);
    
    /* Send data */
    llcc68_send(llcc68_dev, data, 10);
    
    /* Schedule next transmission */
    k_work_schedule(&tx_work, K_MSEC(TX_INTERVAL_MS));
}

void main(void)
{
    struct llcc68_modem_config config;
    int ret;
    
    printk("LLCC68 Test Application\n");
    
    /* Get LLCC68 device */
    llcc68_dev = llcc68_get_device();
    if (!llcc68_dev) {
        printk("LLCC68 device not found\n");
        return;
    }
    
    /* Configure LoRa parameters */
    config.frequency = 868000000;
    config.bandwidth = LLCC68_LORA_BW_125_KHZ;
    config.datarate = LLCC68_LORA_SF_7;
    config.coding_rate = LLCC68_LORA_CR_4_5;
    config.preamble_len = 8;
    config.implicit_header = false;
    config.implicit_len = 0;
    config.crc_enable = true;
    config.tx_power = 14;
    
    ret = llcc68_set_lora_config(llcc68_dev, &config);
    if (ret < 0) {
        printk("Failed to configure LLCC68: %d\n", ret);
        return;
    }
    
    /* Set event handler */
    llcc68_set_event_handler(llcc68_dev, event_handler);
    
    /* Initialize work for periodic transmission */
    k_work_init_delayable(&tx_work, tx_work_handler);
    
    if (rx_mode) {
        /* Set to RX mode for reception test */
        printk("Starting in RX mode\n");
        llcc68_set_rx(llcc68_dev, 0);
    } else {
        /* Schedule first transmission */
        printk("Starting in TX mode\n");
        k_work_schedule(&tx_work, K_MSEC(1000));
    }
}
