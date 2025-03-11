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

/*
  NAME: I2C TARGET INIT
  FILE: plat_i2c_target.c
  DESCRIPTION: Provide i2c target EN/CFG table "I2C_TARGET_EN_TABLE[]/I2C_TARGET_CFG_TABLE[]" for init target config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_target.h" is included by "hal_i2c_target.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "libutil.h"
#include "plat_i2c_target.h"
#include "plat_i2c.h"
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_pldm_sensor.h"
#include "pldm_sensor.h"
#include "plat_version.h"
#include "plat_isr.h"

#define DEVICE_TYPE 0x01
#define REGISTER_LAYOUT_VERSION 0x01
#define AEGIS_CARRIER_BOARD_ID 0x0000
#define AEGIS_CPLD_ADDR (0x4C >> 1)

typedef struct __attribute__((__packed__)) {
	uint8_t device_type;
	uint8_t register_layout_version;
	uint8_t num_idx; //Number of PDR indexes in this register
	uint8_t reserved_1;

	uint16_t sbi; //Sensor base index (SBI= 248*R) used in this register's array
	uint16_t max_pdr_idx; //Max PDR sensor index exported in total (MAX_PDRIDX)

	uint8_t sensor_r_len[]; //sensor[0] reading length ~ sensor[247] reading length
} plat_sensor_init_data_0_1;
// size = sizeof(plat_sensor_init_data_0_1) + num_idx

typedef struct __attribute__((__packed__)) {
	uint8_t sensor_index_offset; // Sensor index offset (e.g. PDR sensor index offset)
	uint32_t sensor_value; // Sensor value (4 bytes)
} sensor_entry;

typedef struct __attribute__((__packed__)) {
	uint8_t device_type; // Device type (Aegis = 0x01, Rainbow = 0x02)
	uint8_t register_layout_version; // Register layout version (e.g. VERSION_1 = 0x01)
	uint16_t sensor_base_index; // Sensor base index (SBI)
	uint8_t max_sbi_off; // Max sensor base index offset in this register (0 <= MAX_SBI_OFF <= 49)
	// The following is a flexible array of sensor entries.
	// The number of entries is (max_sbi_off + 1)
	sensor_entry sensor_entries[];
} plat_sensor_init_data_2_5;
// size = sizeof(plat_sensor_init_data_2_5) + num_sensors * sizeof(SensorEntry);
// num_sensors = max_sbi_off + 1

typedef struct __attribute__((__packed__)) {
	uint16_t carrier_board_id; //  MTIA Gen1 - Aegis=0x00
	uint32_t bic_fw_version;
	uint32_t cpld_fw_version;
} plat_sensor_init_data_6;
// size = sizeof(plat_sensor_init_data_6)

LOG_MODULE_REGISTER(plat_i2c_target);
/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE,	TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
};

plat_sensor_init_data_0_1 *sensor_init_data_0_1_table[2] = { NULL };
plat_sensor_init_data_2_5 *sensor_init_data_2_5_table[4] = { NULL };
plat_sensor_init_data_6 *sensor_init_data_6_table[1] = { NULL };

void *allocate_sensor_data_table(void **table, size_t size)
{
	if (*table) {
		free(*table);
		*table = NULL;
	}

	*table = malloc(size);
	if (!*table) {
		LOG_ERR("Memory allocation failed!");
		return NULL;
	}
	return *table;
}

void initialize_sensor_data_0_1(int table_index)
{
	// Calculate num_idx
	int num_idx = (PLAT_SENSOR_NUM_MAX - 1) - (248 * table_index);
	num_idx = (num_idx > 0) ? ((num_idx > 248) ? 248 : num_idx) : 0;

	// Calculate the memory size
	size_t table_size = sizeof(plat_sensor_init_data_0_1) + num_idx * sizeof(uint8_t);
	plat_sensor_init_data_0_1 *sensor_data = allocate_sensor_data_table(
		(void **)&sensor_init_data_0_1_table[table_index], table_size);

	if (!sensor_data)
		return;

	sensor_data->device_type = DEVICE_TYPE;
	sensor_data->register_layout_version = REGISTER_LAYOUT_VERSION;
	sensor_data->num_idx = num_idx;
	sensor_data->reserved_1 = 0xFF;
	sensor_data->sbi = table_index * 248;
	sensor_data->max_pdr_idx =
		(table_index == 0x00) ? PLAT_SENSOR_NUM_MAX - 2 : 0xFFFF; // PDR indexe is on 0 base
	memset(sensor_data->sensor_r_len, 4, num_idx * sizeof(uint8_t));
}

void initialize_sensor_data_2_5(int table_index)
{
	int num_idx = (PLAT_SENSOR_NUM_MAX - 1) - (50 * table_index);
	num_idx = (num_idx > 0) ? ((num_idx > 50) ? 50 : num_idx) : 0;

	size_t table_size = sizeof(plat_sensor_init_data_2_5) + num_idx * sizeof(sensor_entry);
	plat_sensor_init_data_2_5 *sensor_data = allocate_sensor_data_table(
		(void **)&sensor_init_data_2_5_table[table_index], table_size);

	if (!sensor_data)
		return;

	sensor_data->device_type = DEVICE_TYPE;
	sensor_data->register_layout_version = REGISTER_LAYOUT_VERSION;
	sensor_data->sensor_base_index = table_index * 50;
	sensor_data->max_sbi_off = (num_idx > 0) ? num_idx - 1 : 0;
	if (num_idx > 0) {
		for (int i = 0; i < num_idx; i++) {
			sensor_data->sensor_entries[i].sensor_index_offset =
				i; // sensor_index_offset range: 0~49
			sensor_data->sensor_entries[i].sensor_value = 0x00000000;
		}
	}
}

void initialize_sensor_data_6(int table_index)
{
	size_t table_size = sizeof(plat_sensor_init_data_6);
	if (sensor_init_data_6_table[table_index]) {
		free(sensor_init_data_6_table[table_index]);
		sensor_init_data_6_table[table_index] = NULL;
	}

	sensor_init_data_6_table[0] = (plat_sensor_init_data_6 *)malloc(table_size);
	plat_sensor_init_data_6 *sensor_data = allocate_sensor_data_table(
		(void **)&sensor_init_data_6_table[table_index], table_size);

	if (!sensor_data)
		return;

	uint8_t data[4] = { 0 };
	uint32_t bic_version = 0;
	uint32_t cpld_version = 0;
	if (!plat_i2c_read(I2C_BUS5, AEGIS_CPLD_ADDR, 0x44, data, 4)) {
		LOG_ERR("Failed to read cpld version from cpld");
		return;
	}
	cpld_version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	bic_version =
		(BIC_FW_YEAR_MSB << 24) | (BIC_FW_YEAR_LSB << 16) | (BIC_FW_WEEK << 8) | BIC_FW_VER;

	sensor_data->carrier_board_id = AEGIS_CARRIER_BOARD_ID;
	sensor_data->bic_fw_version = bic_version;
	sensor_data->cpld_fw_version = cpld_version;
}

void update_sensor_data_0_2_table()
{
	for (int table_index = 0; table_index < 4; table_index++) {
		if (!sensor_init_data_2_5_table[table_index])
			continue;

		plat_sensor_init_data_2_5 *sensor_data = sensor_init_data_2_5_table[table_index];
		int num_idx = sensor_data->max_sbi_off + 1;

		for (int i = 0; i < num_idx; i++) {
			int sensor_number =
				sensor_data->sensor_base_index + i + 1; // sensor number is 1 base
			uint8_t status = SENSOR_UNAVAILABLE;
			int reading = 0;
			uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;

			status = pldm_sensor_get_reading_from_cache(sensor_number, &reading,
								    &sensor_operational_state);
			sensor_data->sensor_entries[i].sensor_value =
				(status == SENSOR_READ_SUCCESS) ? reading : 0xFFFFFFFF;
		}
	}
}

size_t get_sensor_data_size(uint8_t reg_offset)
{
	switch (reg_offset) {
	case 0:
	case 1:
		return sizeof(plat_sensor_init_data_0_1) +
		       sensor_init_data_0_1_table[reg_offset]->num_idx * sizeof(uint8_t);
	case 2:
	case 3:
	case 4:
	case 5: {
		plat_sensor_init_data_2_5 *sensor_data = sensor_init_data_2_5_table[reg_offset - 2];
		if (sensor_data->max_sbi_off > 0)
			return sizeof(plat_sensor_init_data_2_5) +
			       (sensor_data->max_sbi_off + 1) * sizeof(sensor_entry);
		else
			return sizeof(plat_sensor_init_data_2_5);
	}
	case 6:
		return sizeof(plat_sensor_init_data_6);
	default:
		return 0;
	}
}

void copy_sensor_data_to_buffer(struct i2c_target_data *data, uint8_t reg_offset)
{
	size_t struct_size = get_sensor_data_size(reg_offset);
	if (struct_size > sizeof(data->target_rd_msg.msg)) {
		struct_size = sizeof(data->target_rd_msg.msg);
	}
	data->target_rd_msg.msg_length = struct_size;
	memcpy(data->target_rd_msg.msg, sensor_init_data_0_1_table[reg_offset], struct_size);
}

static bool command_reply_data_handle(void *arg)
{
	struct i2c_target_data *data = (struct i2c_target_data *)arg;

	/*TODO: put board telemetry here*/

	/* Only check fisrt byte from received data */
	if (data->wr_buffer_idx == 1) {
		uint8_t reg_offset = data->target_wr_msg.msg[0];
		copy_sensor_data_to_buffer(data, reg_offset);
	}
	if (data->wr_buffer_idx == 2) {
		data->target_rd_msg.msg_length = 20;
		if (data->target_wr_msg.msg[1] == data->target_wr_msg.msg[0] * 2) {
			for (int i = 0; i < 20; i++)
				data->target_rd_msg.msg[i] = data->target_wr_msg.msg[0] + i;
		} else {
			for (int i = 0; i < 20; i++)
				data->target_rd_msg.msg[i] = data->target_wr_msg.msg[0] + 2 * i;
		}
	}
	return false;
}

void sensor_data_table_init(void)
{
	for (int i = 0; i < 2; i++)
		initialize_sensor_data_0_1(i);
	for (int i = 0; i < 4; i++)
		initialize_sensor_data_2_5(i);
	for (int i = 0; i < 1; i++)
		initialize_sensor_data_6(i);
}

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0x40, 0xA, command_reply_data_handle },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
	{ 0xFF, 0xA },
};
