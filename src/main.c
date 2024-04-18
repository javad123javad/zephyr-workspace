/*
 * Copyright (c) 2024 Javad Rahimipetroudi <javad.rahimipetroudi@mind.be>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <time.h>
#include <assert.h>

#include <zephyr/kernel.h>
#include <zephyr/net/sntp.h>
#include <arpa/inet.h>

#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/timeutil.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sntp_demo, LOG_LEVEL_DBG);

// Get RTC device
static const struct device *rtc = DEVICE_DT_GET(DT_ALIAS(rtc));

#define SERVER_PORT	123
#define SERVER_ADDR	"192.168.1.100"

#define TIME_BUF_LEN	80
#define DELAY_TIME	1000
/**
 * @brief: This method connects to the NTP server and gets the current time
 *
 */
int sntp_get_time(struct sntp_ctx *ctx, const char* server_addr, const unsigned int server_port, time_t * out_time)
{
	int ret = 0;
	struct sntp_time sntp_time;
	struct sockaddr_in addr;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(SERVER_PORT);

	inet_pton(AF_INET, SERVER_ADDR, &addr.sin_addr);


	ret = sntp_init(ctx, (struct sockaddr *) &addr, sizeof(struct sockaddr_in));
	if(ret < 0)
	{
		LOG_ERR("Failed to initialize SNTP, errno: %d", ret);
		return ret;
	}

	LOG_INF("Sending SNTP IPv4 request...");

	ret = sntp_query(ctx, 10*MSEC_PER_SEC, &sntp_time);
	if (ret < 0) {
		LOG_ERR("SNTP IPv4 request failed: %d", ret);
		return ret;
	}

	LOG_INF("time since Epoch: high word: %u, low word: %u",
			(uint32_t)(sntp_time.seconds >> 32), (uint32_t)sntp_time.seconds);
	*out_time = (sntp_time.seconds >> 32) + sntp_time.seconds;

	return ret;
}

/**
 * @brief: This method update internal RTC of the microcontroller
 *
 */
int rtc_update_time(const time_t * sntp_time)
{
	int ret = 0;
	struct rtc_time datetime_set;
	
	gmtime_r(sntp_time, (struct tm *)(&datetime_set));
	ret = rtc_set_time(rtc, &datetime_set);
	return ret;
}

/**
 * @brief: This method fetchs current time from internal RTC
 *
 */
int rtc_get_current_time(struct rtc_time *datetime)
{
	int ret = 0;
	assert(datetime != NULL);

	ret = rtc_get_time(rtc, datetime);
	return ret;
}

/**
 * @brief: This method convers time from epoch to local human 
 * readable time
 *
 */
int epoch_to_datetime(time_t * epoch_time, char *date_time, const size_t len)
{
	int ret = 0;
	struct tm  ts;
	
	assert(epoch_time != NULL);

	// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
	ts = *localtime(epoch_time);
	strftime(date_time, len, "%a %Y-%m-%d %H:%M:%S %Z", &ts);


	return ret;
}

int main(void)
{
	struct sntp_ctx ctx;
	time_t ntp_time;
	char   time_buf[TIME_BUF_LEN] = {0};
	struct rtc_time timedate_get;
	int ret = 0;

	ret = sntp_get_time(&ctx, SERVER_ADDR, SERVER_PORT, &ntp_time);
	assert(ret == 0);	

	ret = rtc_update_time(&ntp_time);
	assert(ret == 0);

	while(1)
	{
		rtc_get_current_time(&timedate_get);
		LOG_INF("Time: %d:%d:%d", timedate_get.tm_hour,timedate_get.tm_min,timedate_get.tm_sec);
		k_msleep(DELAY_TIME);
	}

	return 0;
}
