/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "unistd.h"
#include "zephyr/kernel.h"
#include "zephyr/net/net_if.h"
#include "zephyr/net/net_mgmt.h"
#include "zephyr/sys/printk.h"
#include <stdio.h>
#include <string.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/conn_mgr_monitor.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hello_world, LOG_LEVEL_DBG);

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#include <zephyr/net/tls_credentials.h>
#include "ca_certificate.h"
#endif

/* HTTP server to connect to */
#define HTTP_HOST "google.com"
/* Port to connect to, as string */
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#define HTTP_PORT "443"      
#else 
#define HTTP_PORT "80"       
#endif  
/* HTTP path to request */
#define HTTP_PATH "/"
#define REQUEST "GET " HTTP_PATH " HTTP/1.0\r\nHost: " HTTP_HOST "\r\n\r\n"

static struct net_mgmt_event_callback net_wifi_handler;
static char response[1024];

int connected  = 0;
static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
#if 0
	const struct wifi_status *status = (const struct wifi_status *)cb->info;
	if (status->status) {
		LOG_ERR( "Connection request failed (%d)", status->status);
	} else {
		LOG_INF( "WIFI Connected");
		connected = 1;
	}
#endif
}

void wifi_conn_handler(struct net_mgmt_event_callback *cb,uint32_t mgmt_event, struct net_if *iface)
{

	switch (mgmt_event) {
		case NET_EVENT_WIFI_CONNECT_RESULT:
			//handle_wifi_connect_result(cb);
			LOG_INF("WIFI connected");
			connected = 1;
			break;
		default:
			break;
	}

}
int wifi_connect()
{
	printk("Trying to connect\n");
	int ret = 0;
	char *wifi_ssid = "Zeus";
	char *wifi_pass = "qwe123!@#";
	struct wifi_connect_req_params wifi_param = {
		.ssid = wifi_ssid,
		.ssid_length = 0,
		.psk = wifi_pass,
		.psk_length = 0,
		.channel = 0,
		.security = WIFI_SECURITY_TYPE_PSK,
	};
	wifi_param.ssid_length = strlen(wifi_ssid);
	wifi_param.psk_length = strlen(wifi_pass);
	/* Bind callback to the net handler */
	net_mgmt_init_event_callback(&net_wifi_handler, wifi_conn_handler, NET_EVENT_WIFI_CONNECT_RESULT);
	net_mgmt_add_event_callback(&net_wifi_handler);

	int idx = 10;
	struct net_if *iface = net_if_get_default();
	if(iface == NULL)
	{
		LOG_ERR("No Iface found!\n");
		return -1;
	}
	while (idx-- > 0) {
		ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_param, sizeof(struct wifi_connect_req_params));
		if(ret == 0)
		{
			LOG_INF("NetMGR OK!");
			break;
		}
		else {
			LOG_ERR("NtMGR error ! code: %d",ret);
		}
		k_msleep(500);

	}
	return ret;

}
#define CHECK(r) { if (r == -1) { printf("Error: " #r "\n");} }
#define SSTRLEN(s) (sizeof(s) - 1)

int socket_connect()
{
	int ret = 0;
	static struct addrinfo hints;
	struct addrinfo *res;
	int st, sock; 
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	tls_credential_add(CA_CERTIFICATE_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
			ca_certificate, sizeof(ca_certificate));
#endif
	printf("Preparing HTTP GET request for http://" HTTP_HOST
			":" HTTP_PORT HTTP_PATH "\n");

	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	st = getaddrinfo( HTTP_HOST, HTTP_PORT, &hints, &res);
	printf("getaddrinfo status: %d\n", st);

	if (st != 0) {
		printf("Unable to resolve address, quitting\n");
		return 0;
	}
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	sock = socket(res->ai_family, res->ai_socktype, IPPROTO_TLS_1_2);
#else
	sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
#endif
	CHECK(sock);
	printf("sock = %d\n", sock);
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	sec_tag_t sec_tag_opt[] = {
		[0]=CA_CERTIFICATE_TAG,
	};
	printk("STEP 01\n");
	CHECK(setsockopt(sock, SOL_TLS, TLS_SEC_TAG_LIST,
				sec_tag_opt, sizeof(sec_tag_opt)));
	printk("STEP 02\n");
	CHECK(setsockopt(sock, SOL_TLS, TLS_HOSTNAME,
				HTTP_HOST, sizeof(HTTP_HOST)))
#endif

		CHECK(connect(sock, res->ai_addr, res->ai_addrlen));
	CHECK(send(sock, REQUEST, SSTRLEN(REQUEST), 0));

	printf("Response:\n\n");

	while (1) {
		int len = recv(sock, response, sizeof(response) - 1, 0);

		if (len < 0) {
			printf("Error reading response\n");
			return 0;
		}

		if (len == 0) {
			break;
		}

		response[len] = 0;
		printf("%s", response);
	}

	printf("\n");

	(void)close(sock);

	return ret;
}
int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	int idx = 10;
	wifi_connect();
	while (1) {
		if(connected)
		{
			LOG_INF("Getting HTTP...");
			socket_connect();
			break;
		}
		k_msleep(1000);
	}
	while (1) {
		k_msleep(1000);	
	}
}
