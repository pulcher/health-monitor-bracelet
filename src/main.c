/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/display.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include "gui.h"
#include "app_ble.h"
#include <drivers/i2c.h>
#include "max30102.h"


#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(app);

#define I2C_DEV "I2C_1"

static void on_app_ble_event(app_ble_event_data_t * evt_data)
{
	switch(evt_data->type) {
		case APP_BLE_CONNECTED:
			printk("Connected\n");
			gui_set_bt_state(GUI_BT_STATE_CONNECTED);
			break;

		case APP_BLE_DISCONNECTED:
			printk("Disconnected\n");
			gui_set_bt_state(GUI_BT_STATE_ADVERTISING);

			app_ble_start_advertising();
			break;

		case APP_BLE_ON_LED_WRITE:
			printk("Led %s\n", evt_data->led_state ? "On" : "Off");
			gui_set_bt_led_state(evt_data->led_state);
			break;
	}
}

void on_gui_event(gui_event_t *event)
{
	switch(event->evt_type) {
		case GUI_EVT_BUTTON_PRESSED:
			app_ble_send_button_state(event->button_checked);
			break;
	}
}

void main(void)
{
    const struct device *i2c_dev;
	uint8_t cmp_data[16];
	uint8_t data[16];
	int i, ret;

	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	if (!is_max30102_available(i2c_dev)) {
		printk("I2C: MAX30102 not found.\n");
		return;          
	}

  	if (get_max30102_part_id(i2c_dev) != MAX30102_EXPECTED_PARTID) {
    	printk("not expected partid");
    	return;
  	}

	max30102_softReset(i2c_dev);
        printk("Located MAX30102...\n");

        //Set reasonably to make sure there is clear sawtooth figure on the serial plotter
        /*!
         *@brief Use macro definition to configure sensor 
         *@param ledBrightness LED brightness, default value: 0x1F（6.4mA), Range: 0~255（0=Off, 255=50mA）
         *@param sampleAverage Average multiple samples then draw once, reduce data throughput, default 4 samples average
         *@param ledMode LED mode, default to use red light and IR at the same time
         *@param sampleRate Sampling rate, default 400 samples every second
         *@param pulseWidth Pulse width: the longer the pulse width, the wider the detection range. Default to be Max range
         *@param adcRange Measurement Range, default 4096 (nA), 15.63(pA) per LSB
         */
        max30102_sensorConfiguration(/*ledBrightness=*/60, /*sampleAverage=*/SAMPLEAVG_8, \
                                        /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_400, \
                                        /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_16384);

	printk("Cionfigured MAX30102...\n");


	uint8_t error = 0u;

	i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));

	gui_config_t gui_config = {.event_callback = on_gui_event};
	gui_init(&gui_config);

	app_ble_config_t app_ble_config = {.event_callback = on_app_ble_event};
	app_ble_init(&app_ble_config);

	app_ble_start_advertising();

	printk("pulcher Advertising started\n");

	gui_set_bt_state(GUI_BT_STATE_ADVERTISING);

	while (1) {
          uint8_t error;
          int nDevices;
 
 /*         printk("Scanning...\n");
 
          nDevices = 0;
          for(uint16_t address = 80; address < 90; address++ ) 
          {
            struct i2c_msg msgs[1];
            uint8_t dst = 1;

            msgs[0].buf = &dst;
            msgs[0].len = 1U;
            msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

            error = i2c_transfer(i2c_dev, &msgs[0], 1, address);
            printk("Target Address: %x(%d), Error Code: %d, ", address, address, error);
            if (error == 0)
            {
              printk("I2C device found at address 0x");
              if (address<16) 
                printk("0");
              printk("%x",address);
 
              nDevices++;
            }
            /*else if (error==4) 
            {
              printk("Unknow error at address 0x");
              if (address<16) 
                printk("0");
              printk(address);
            }    */
 /*           printk("  !\n");
          }
          
          if (nDevices == 0)
            printk("No I2C devices found\n");
          else
            printk("done\n");
 */
          k_sleep(K_MSEC(3000));
	}
}
