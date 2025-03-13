/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_isr.h"

#define gpio_name_to_num(x) #x,
char *gpio_name[] = { name_gpio0 name_gpio1 name_gpio2 name_gpio3 name_gpio4 name_gpio5 name_gpio6
			      name_gpio7 name_gpio8 name_gpio9 name_gpioA name_gpioB name_gpioC
				      name_gpioD name_gpioE name_gpioF };
#undef gpio_name_to_num

void gpio_int_default()
{
	return;
}

GPIO_CFG plat_gpio_cfg[] = {
	//  chip,      number,   is_init, is_latch, direction,    status,     property,    int_type,              int_cb
	// GPIO 0 group
	{ CHIP_GPIO, 0, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  ISR_GPIO_FM_ASIC_0_THERMTRIP_R_N },
	{ CHIP_GPIO, 1, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  ISR_GPIO_RST_ATH_PWR_ON_PLD_R1_N },
	{ CHIP_GPIO, 2, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  ISR_GPIO_ATH_CURRENT_SENSE_0_NPCM_R },
	{ CHIP_GPIO, 3, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  ISR_GPIO_ATH_CURRENT_SENSE_1_NPCM_R },
	{ CHIP_GPIO, 4, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  ISR_GPIO_FM_ATH_HBM3_CATTRIP_ALARM_LV33_R },
	{ CHIP_GPIO, 5, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  ISR_GPIO_ALL_VR_PM_ALERT_R_N },
	{ CHIP_GPIO, 6, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  ISR_GPIO_ATH_SMB_ALERT_NPCM_LVC33_R_N },
	{ CHIP_GPIO, 7, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_EDGE_BOTH,
	  ISR_GPIO_FM_PLD_UBC_EN_R },
	// GPIO 1 group
	{ CHIP_GPIO, 8, ENABLE, DISABLE, GPIO_OUTPUT, GPIO_HIGH, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 9, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 10, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 11, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 12, DISABLE, DISABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL }, // CR_SIN1 (default console)
	{ CHIP_GPIO, 13, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL }, // CR_SOUT1 (default console)
	{ CHIP_GPIO, 14, DISABLE, DISABLE, GPIO_INPUT, GPIO_HIGH, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 15, DISABLE, DISABLE, GPIO_INPUT, GPIO_HIGH, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 2 group
	{ CHIP_GPIO, 16, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 17, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 18, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 19, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 20, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 21, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 22, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 23, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 3 group
	{ CHIP_GPIO, 24, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 25, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 26, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 27, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 28, DISABLE, DISABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 29, DISABLE, DISABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 30, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 31, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 4 group
	{ CHIP_GPIO, 32, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 33, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 34, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 35, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 36, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 37, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 38, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 39, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 5 group
	{ CHIP_GPIO, 40, DISABLE, DISABLE, GPIO_OUTPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 41, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 42, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 43, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 44, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 45, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 46, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 47, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 6 group
	{ CHIP_GPIO, 48, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 49, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 50, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 51, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 52, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 53, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 54, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 55, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 7 group
	{ CHIP_GPIO, 56, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 57, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 58, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 59, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 60, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 61, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 62, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 63, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 8 group
	{ CHIP_GPIO, 64, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 65, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 66, ENABLE, DISABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 67, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 68, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL }, // unused on npcm4xx
	{ CHIP_GPIO, 69, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL }, // unused on npcm4xx
	{ CHIP_GPIO, 70, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 71, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO 9 group
	{ CHIP_GPIO, 72, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 73, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 74, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 75, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 76, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 77, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 78, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 79, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO A group
	{ CHIP_GPIO, 80, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 81, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 82, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 83, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 84, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 85, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 86, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 87, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO B group
	{ CHIP_GPIO, 88, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 89, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 90, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 91, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 92, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 93, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 94, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 95, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO C group
	{ CHIP_GPIO, 96, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 97, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 98, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 99, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 100, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 101, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 102, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 103, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO D group
	{ CHIP_GPIO, 104, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 105, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 106, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 107, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 108, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 109, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 110, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 111, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	// GPIO E group
	{ CHIP_GPIO, 112, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 113, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 114, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 115, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 116, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 117, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 118, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 119, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL }, // unused on npcm4xx
	// GPIO F group
	{ CHIP_GPIO, 120, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 121, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 122, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
	{ CHIP_GPIO, 123, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE,
	  NULL },
};

bool pal_load_gpio_config(void)
{
	memcpy(&gpio_cfg[0], &plat_gpio_cfg[0], sizeof(plat_gpio_cfg));
	return 1;
};