#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
// UART device and buffer definitions
#define UART_DEV_NAME (DT_NODELABEL(bs_serial))  // UART3
#define BUF_SIZE 128                                  // Buffer size
#define TIMEOUT 100                                  // Timeout in ms
#define STACK_SIZE 1024                             // Stack size
#define PRIORITY 7                                  // Priority
#define NUM_BUFFERS 8                               // Number of buffers
// Buffers for double buffering
static uint8_t rx_buf1[BUF_SIZE];                   // Buffer 1

static const struct device *uart_dev;               // UART device
static uint8_t *current_buf = rx_buf1;              // Start with buffer 1
static uint8_t *next_buf;

// Define a structure to hold data for the FIFO
struct uart_data {
    void *fifo_reserved;  // First word reserved for FIFO (used internally by Zephyr)
    uint8_t buf[BUF_SIZE];  // Data buffer
    size_t len; // Data length
};

// Define thread stack and control structure
K_THREAD_STACK_DEFINE(stack, STACK_SIZE);
static struct k_thread thread_data;

/* Static array of data buffers */
static struct uart_data rx_buffers[NUM_BUFFERS];

// Declare a FIFO queue
K_FIFO_DEFINE(uart_fifo);

/* Static buffer index tracking */
static uint8_t buffer_index = 0;

// UART RX callback function
void uart_dma_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        // Allocate a new data structure for FIFO
        struct uart_data *data = &rx_buffers[buffer_index];
        if (data != NULL) {
            data->len = evt->data.rx.len;  // Store received length
            memset(data->buf, 0, BUF_SIZE);  // Clear buffer
            memcpy(data->buf, &current_buf[evt->data.rx.offset], evt->data.rx.len);  // Copy received data
            data->len = evt->data.rx.len;  // Store received length
            k_fifo_put(&uart_fifo, data);  // Push data into FIFO
            /* Move to the next buffer in the array (wrap around) */
            buffer_index = (buffer_index + 1) % NUM_BUFFERS;
        } else {
            printk("Failed to allocate memory for FIFO\n");
        }

        break;
    case UART_RX_BUF_RELEASED:
    {
        next_buf = evt->data.rx_buf.buf;

        break;
    }

    case UART_RX_BUF_REQUEST:
        int err;
        err = uart_rx_buf_rsp(uart_dev, next_buf, BUF_SIZE);
        if(err)
        {
            printk("Failed to switch buffer: %d\n", err);
        }

        break;

    case UART_RX_DISABLED:
        uart_rx_enable(uart_dev, next_buf, BUF_SIZE, TIMEOUT);
        break;

    default:
        break;
    }

}

// Thread for processing received data from FIFO
void uart_process_thread(void*, void*, void*)
{
    struct uart_data *rx_data;

    while (1) {
        // Retrieve data from FIFO (blocks until data is available)
        rx_data = k_fifo_get(&uart_fifo, K_FOREVER);
        if (rx_data != NULL) {
            printk("Received: %s\n",rx_data->buf);
        }
    }
}

// Main function
int  main(void)
{
    int ret;
    // Initialize UART device
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(bs_serial));
    if (!uart_dev) {
        printk("Failed to get UART device\n");
        return -1;
    }
    uart_rx_disable(uart_dev);  // Disable UART RX
    // Enable DMA-based UART reception with the first buffer
    current_buf = rx_buf1;
    /* Set up UART configuration */
    struct uart_config uart_cfg = {
        .baudrate = 115200,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };

    ret = uart_configure(uart_dev, &uart_cfg);
    if (ret) {
        printk("Failed to configure UART: %d\n", ret);
        return ret;
    }
    
    ret = uart_rx_enable(uart_dev, current_buf, BUF_SIZE, TIMEOUT);
    if (ret) {
        printk("Failed to enable UART RX\n");
        return -1;
    }

    // Set the UART callback function
    uart_callback_set(uart_dev, uart_dma_callback, NULL);

    // Create a thread for processing FIFO data
    k_thread_create(&thread_data, stack, STACK_SIZE,
                    uart_process_thread, NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);

    // Start the thread
    k_thread_start(&thread_data);
}

