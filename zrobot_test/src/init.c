#include <zephyr/ztest.h>
#include <stdio.h>
#include "init.h"
ZTEST_SUITE(init_tests, NULL, NULL, NULL, NULL, NULL);

void sys_init()
{
	printf("Robot init...\r\n");
}
