/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#define MOTORF_LEFT_A DT_NODELABEL(m11)
#define MOTORF_LEFT_B DT_NODELABEL(m12)

#define MOTORF_RIGHT_A DT_NODELABEL(m21)
#define MOTORF_RIGHT_B DT_NODELABEL(m22)

#define MOTORB_LEFT_A DT_NODELABEL(m31)
#define MOTORB_LEFT_B DT_NODELABEL(m32)

#define MOTORB_RIGHT_A DT_NODELABEL(m41)
#define MOTORB_RIGHT_B DT_NODELABEL(m42)

int main(void)
{
	const struct pwm_dt_spec mfl_a = PWM_DT_SPEC_GET(MOTORF_LEFT_A);
	const struct pwm_dt_spec mfl_b = PWM_DT_SPEC_GET(MOTORF_LEFT_B);

	const struct pwm_dt_spec mfr_a = PWM_DT_SPEC_GET(MOTORF_RIGHT_A);
	const struct pwm_dt_spec mfr_b = PWM_DT_SPEC_GET(MOTORF_RIGHT_B);

	const struct pwm_dt_spec mbl_a = PWM_DT_SPEC_GET(MOTORB_LEFT_A);
	const struct pwm_dt_spec mbl_b = PWM_DT_SPEC_GET(MOTORB_LEFT_B);

	const struct pwm_dt_spec mbr_a = PWM_DT_SPEC_GET(MOTORB_RIGHT_A);
	const struct pwm_dt_spec mbr_b = PWM_DT_SPEC_GET(MOTORB_RIGHT_B);

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	if (
		!pwm_is_ready_dt(&mfl_a)
|| !pwm_is_ready_dt(&mfl_b)
|| !pwm_is_ready_dt(&mfr_a)
|| !pwm_is_ready_dt(&mfr_b)
|| !pwm_is_ready_dt(&mbl_a)
|| !pwm_is_ready_dt(&mbl_b)
|| !pwm_is_ready_dt(&mbr_a)
|| !pwm_is_ready_dt(&mbr_b)) {
		printk("Error: PWM device  is not ready\n");
		return 0;
	}

	int ret = pwm_set_dt(&mfl_a, mfl_a.period, 0);
	if(ret)
	{
		printk("Error setting motor pulse width: %d\n", ret);
	}
	k_msleep(1000);

	
	ret = pwm_set_dt(&mfl_a, mfl_a.period, mfl_a.period/2);
	if(ret)
	{
		printk("Opps: Unable to set the pwm: %d\n",ret);
	}
	printk("PWM is set!\n");
	while (1) {
		k_msleep(1000);
	}
	return 0;
}
