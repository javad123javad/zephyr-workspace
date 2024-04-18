Overview
********

The is an Zephyr SNTP demo. The demo performs the following acts:
* Get current date/time from NTP server.
* Update inter RTC block.
* Get current date/time from RTC and display in the console.


The source code shows how to:

#. Get a pin specification from the :ref:`devicetree <dt-guide>` as a
   :c:struct:`gpio_dt_spec`
#. Configure the GPIO pin as an output
#. Toggle the pin forever

See :zephyr:code-sample:`pwm-blinky` for a similar sample that uses the PWM API instead.



Requirements
************

Your board must:

#. Have an network connection via Ethernet/WiFi.
#. Haave a RTC block

You should also define `rtc` alias in the board device tree.

Building and Running
********************

First clone the repository in zephy parent directory. Then build and flash as follows:

```bash
west build -p always -b nucleo_h723zg workspace/sntp_demo/
```

