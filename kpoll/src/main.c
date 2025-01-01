/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/__assert.h"
#include "zephyr/sys/printk.h"
#include "zephyr/sys/util.h"
#include <stdio.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <string.h>

//struct k_fifo my_fifo;
K_FIFO_DEFINE(my_fifo);
struct data_item {
	void *fifo_reserved;
	int cmd;
};

static struct k_poll_signal my_signal = K_POLL_SIGNAL_INITIALIZER(my_signal);

void producer_thread(void)
{
	int i = 0;
	while(1)
	{
		printk("Generate new data: %d\n", i);
		struct data_item item = {.cmd = i};
		size_t tx_size = sizeof(struct data_item);

		char* mem_ptr = k_malloc(tx_size);
		__ASSERT_NO_MSG(mem_ptr != 0);

		memcpy(mem_ptr,  &item, tx_size);

		k_fifo_put(&my_fifo, mem_ptr);

		i++;
		i > 100 ? i = 0: i;
		k_msleep(1000);
	}
}
/* A thread for rising dummy signal */
void signal_thread(void)
{
	while (1) {
		k_poll_signal_raise(&my_signal, 0x1989);
		k_msleep(1000);
	
	}
}
// Polling
struct k_poll_event events[2] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &my_fifo, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &my_signal, 0),

};

// Define producer thread
K_THREAD_DEFINE(prdc_thread, 1020, producer_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(sig_thread, 1024, signal_thread, NULL, NULL, NULL, 5, 0,0);

int main(void)
{
	int ret = 0;
	//k_fifo_init(&my_fifo);
	while (1) {
		ret = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		if(ret == 0)
		{
			if(events[0].state == K_POLL_STATE_FIFO_DATA_AVAILABLE)
			{
				//consume data here
				printk("FIFO data is available\n");
				struct data_item *data = k_fifo_get(&my_fifo, K_FOREVER);
				printk("Data: %d\n", data->cmd);
				k_free(data);

			}
			int signaled, result;

			k_poll_signal_check(&my_signal, &signaled, &result);
			if(signaled && result == 0x1989)
			{
				printk("A signal rised!\n");
			}

			k_poll_signal_reset(&my_signal);
			events[1].state = K_POLL_STATE_NOT_READY;
		}
		events[0].state = K_POLL_STATE_NOT_READY;

		k_msleep(1000);

	}
	return 0;
}
