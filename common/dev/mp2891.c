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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "util_pmbus.h"

LOG_MODULE_REGISTER(mp2891);

#define DAC_2P5MV_EN_BIT BIT(13)
#define MFR_VID_RES_MASK GENMASK(15, 14)
#define IOUT_SCALE_MASK GENMASK(2, 0)
#define IOUT_MASK GENMASK(10, 0)

#define MFR_SVI3_IOUT_RPT 0x65
#define MFR_VOUT_LOOP_CTRL 0xBD

#define VR_MPS_PAGE_0 0x00
#define VR_MPS_PAGE_1 0x01
#define VR_MPS_PAGE_2 0x02

#define VR_MPS_VEND_ID 0x4D5053 // ASCI: MPS
#define MP2891_DEV_ID 0x2891
#define MAX_CMD_LINE 720
#define VR_REG_STORE 0x17

/* --------- PAGE0 ---------- */
#define VR_REG_VENDOR_ID 0x99
#define VR_REG_STAT_CML 0x7E

/* --------- PAGE1 ---------- */
#define VR_REG_EXPECTED_USER_CRC 0xF0

/* --------- PAGE2 ---------- */
#define VR_REG_DEV_ID 0x93

enum {
	ATE_CONF_ID = 0,
	ATE_PAGE_NUM,
	ATE_REG_ADDR_HEX,
	ATE_REG_ADDR_DEC,
	ATE_REG_NAME,
	ATE_REG_DATA_HEX,
	ATE_REG_DATA_DEC,
	ATE_COL_MAX,
};

struct mp2891_data {
	uint16_t cfg_id;
	uint8_t page;
	uint8_t reg_addr;
	uint8_t reg_data[4];
	uint8_t reg_len;
};

struct mp2891_config {
	uint8_t mode;
	uint16_t cfg_id;
	uint16_t wr_cnt;
	uint16_t product_id_exp;
	struct mp2891_data *pdata;
};

static bool mp2891_set_page(uint8_t bus, uint8_t addr, uint8_t page)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = PMBUS_PAGE;
	i2c_msg.data[1] = page;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to set page to 0x%02X", page);
		return false;
	}

	k_msleep(100);

	return true;
}

static bool mp2891_write_data(uint8_t bus, uint8_t addr, struct mp2891_data *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = data->reg_len + 1;
	i2c_msg.data[0] = data->reg_addr;
	memcpy(&i2c_msg.data[1], &data->reg_data[0], data->reg_len);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write register 0x%02X with data:", data->reg_addr);
		LOG_HEXDUMP_ERR(&data->reg_data[0], data->reg_len, "");
		return false;
	}

	return true;
}

bool mp2891_get_fw_version(uint8_t bus, uint8_t addr, uint32_t *rev)
{
	CHECK_NULL_ARG_WITH_RETURN(rev, false);

	if (mp2891_set_page(bus, addr, VR_MPS_PAGE_1) == false) {
		LOG_ERR("Failed to set page before reading config revision");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_EXPECTED_USER_CRC;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_INF("Failed to read config revision");
		return false;
	}

	*rev = (i2c_msg.data[1] << 8) | i2c_msg.data[0];

	return true;
}

static bool mp2891_get_vendor_id(uint8_t bus, uint8_t addr, uint8_t *vendor_id)
{
	CHECK_NULL_ARG_WITH_RETURN(vendor_id, false);

	if (mp2891_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before reading vendor id");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 4;
	i2c_msg.data[0] = VR_REG_VENDOR_ID;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read vendor id");
		return false;
	}

	*vendor_id = (i2c_msg.data[1]) | (i2c_msg.data[2] << 8) | (i2c_msg.data[3] << 16);

	return true;
}

static bool mp2891_get_device_id(uint8_t bus, uint8_t addr, uint8_t *device_id)
{
	CHECK_NULL_ARG_WITH_RETURN(device_id, false);

	if (mp2891_set_page(bus, addr, VR_MPS_PAGE_2) == false) {
		LOG_ERR("Failed to set page before reading device id");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 2;
	i2c_msg.data[0] = VR_REG_DEV_ID;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read device id");
		return false;
	}

	*device_id = i2c_msg.data[0] | (i2c_msg.data[1] << 8);

	return true;
}

static bool mp2891_store(uint8_t bus, uint8_t addr)
{
	if (mp2891_set_page(bus, addr, VR_MPS_PAGE_0) == false) {
		LOG_ERR("Failed to set page before store data");
		return false;
	}

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;

	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;

	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = VR_REG_STORE;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to send store command 0x%02X", VR_REG_STORE);
		return false;
	}

	return true;
}

static bool mp2891_pre_update(uint8_t bus, uint8_t addr)
{
	uint8_t vend_id = 0;
	if (mp2891_get_vendor_id(bus, addr, &vend_id) == false) {
		LOG_ERR("Failed to read device id");
		return false;
	}
	if (vend_id != VR_MPS_VEND_ID) {
		LOG_ERR("Invalid vendor id 0x%x", vend_id);
		return false;
	}

	uint8_t dev_id = 0;
	if (mp2891_get_device_id(bus, addr, &dev_id) == false) {
		LOG_ERR("Failed to read device id");
		return false;
	}
	if (dev_id != MP2891_DEV_ID) {
		LOG_ERR("Invalid device id 0x%x", dev_id);
		return false;
	}

	return true;
}

static bool parsing_image(uint8_t *img_buff, uint32_t img_size, struct mp2891_config *dev_cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);
	CHECK_NULL_ARG_WITH_RETURN(dev_cfg, false);

	bool ret = false;

	/* Parsing image */
	int max_line = MAX_CMD_LINE;
	dev_cfg->pdata = (struct mp2891_data *)malloc(sizeof(struct mp2891_data) * max_line);
	if (!dev_cfg->pdata) {
		LOG_ERR("pdata malloc failed!");
		goto exit;
	}

	struct mp2891_data *cur_line = &dev_cfg->pdata[0];
	uint8_t cur_ele_idx = 0;
	uint32_t data_store = 0;
	uint8_t data_idx = 0;
	dev_cfg->wr_cnt = 0;

	for (int i = 0; i < img_size; i++) {
		/* check valid */
		if (!img_buff[i]) {
			LOG_ERR("Get invalid buffer data at index %d", i);
			goto exit;
		}

		if ((cur_ele_idx == ATE_CONF_ID) && (i + 2 < img_size)) {
			if (!strncmp(&img_buff[i], "END", 3)) {
				break;
			}
		}

		if (((img_buff[i] != 0x09) && img_buff[i] != 0x0d)) {
			// pass non hex charactor
			int val = ascii_to_val(img_buff[i]);
			if (val == -1)
				continue;

			data_store = (data_store << 4) | val;
			data_idx++;
			continue;
		}

		switch (cur_ele_idx) {
		case ATE_CONF_ID:
			cur_line->cfg_id = data_store & 0xffff;
			break;

		case ATE_PAGE_NUM:
			cur_line->page = data_store & 0xff;
			break;

		case ATE_REG_ADDR_HEX:
			cur_line->reg_addr = data_store & 0xff;
			break;

		case ATE_REG_ADDR_DEC:
			break;

		case ATE_REG_NAME:
			break;

		case ATE_REG_DATA_HEX:
			*((uint32_t *)cur_line->reg_data) = data_store;
			cur_line->reg_len = data_idx % 2 == 0 ? data_idx / 2 : (data_idx / 2 + 1);
			break;

		case ATE_REG_DATA_DEC:
			break;

		default:
			LOG_ERR("Got unknow element index %d", cur_ele_idx);
			goto exit;
		}

		data_idx = 0;
		data_store = 0;

		if (img_buff[i] == 0x09) {
			cur_ele_idx++;
		} else if (img_buff[i] == 0x0d) {
			LOG_DBG("vr[%d] page: %d addr:%x", dev_cfg->wr_cnt, cur_line->page,
				cur_line->reg_addr);
			LOG_HEXDUMP_DBG(cur_line->reg_data, cur_line->reg_len, "data:");

			cur_ele_idx = 0;
			dev_cfg->wr_cnt++;
			if (dev_cfg->wr_cnt > max_line) {
				LOG_ERR("Line record count is overlimit");
				goto exit;
			}
			cur_line++;
			i++; //skip 'a'
		}
	}

	ret = true;

exit:
	if (ret == false)
		SAFE_FREE(dev_cfg->pdata);

	return ret;
}

bool mp2891_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	uint8_t ret = false;

	/* Step1. Image parsing */
	struct mp2891_config dev_cfg = { 0 };
	if (parsing_image(img_buff, img_size, &dev_cfg) == false) {
		LOG_ERR("Failed to parsing image!");
		goto exit;
	}

	/* Step2. Before update */
	if (mp2891_pre_update(bus, addr) == false) {
		LOG_ERR("Failed to pre-update!");
		goto exit;
	}

	/* Step3. Firmware update */
	uint8_t last_page = 0xFF;
	struct mp2891_data *cur_data;
	uint16_t line_idx = 0;

	//Program Page0 and Page1 registers
	for (line_idx = 0; line_idx < dev_cfg.wr_cnt; line_idx++) {
		cur_data = &dev_cfg.pdata[line_idx];
		if (last_page != cur_data->page) {
			if (mp2891_set_page(bus, addr, cur_data->page) == false) {
				goto exit;
			}
			last_page = cur_data->page;
		}
		if (mp2891_write_data(bus, addr, cur_data) == false)
			goto exit;

		uint8_t percent = ((line_idx + 1) * 100) / dev_cfg.wr_cnt;
		if (percent % 10 == 0)
			LOG_INF("updated: %d%% (line: %d/%d page: %d)", percent, line_idx + 1,
				dev_cfg.wr_cnt, cur_data->page);
	}

	if (mp2891_store(bus, addr) == false) {
		LOG_ERR("Failed to store data to MTP!");
		goto exit;
	}

	k_msleep(1000);

	ret = true;
exit:
	SAFE_FREE(dev_cfg.pdata);
	return ret;
}

float mp2891_get_resolution(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_FAIL_TO_ACCESS);

	uint8_t offset = cfg->offset;
	float reso = 0;

	I2C_MSG msg;
	uint8_t i2c_max_retry = 5;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;

	switch (offset) {
	case PMBUS_READ_VOUT:
		msg.data[0] = MFR_VOUT_LOOP_CTRL;

		if (i2c_master_read(&msg, i2c_max_retry)) {
			LOG_WRN("I2C read failed");
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint16_t mfr_vout_loop_ctrl = (msg.data[1] << 8) | msg.data[0];

		if (mfr_vout_loop_ctrl & DAC_2P5MV_EN_BIT) {
			reso = 0.0025;
		} else if ((mfr_vout_loop_ctrl & MFR_VID_RES_MASK) == BIT(14)) {
			reso = 0.005;
		} else if ((mfr_vout_loop_ctrl & MFR_VID_RES_MASK) == BIT(15)) {
			reso = 0.002;
		} else {
			LOG_WRN("vout_reso_set not supported: 0x%x", mfr_vout_loop_ctrl);
			return SENSOR_FAIL_TO_ACCESS;
		}
		return reso;
	case PMBUS_READ_IOUT:
		msg.data[0] = MFR_SVI3_IOUT_RPT;

		if (i2c_master_read(&msg, i2c_max_retry)) {
			LOG_WRN("I2C read failed");
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint16_t mfr_svi3_iout_prt_data = (msg.data[1] << 8) | msg.data[0];

		switch (mfr_svi3_iout_prt_data & IOUT_SCALE_MASK) {
		case 0:
			reso = 1;
			break;
		case 1:
			reso = 0.03125;
			break;
		case 2:
			reso = 0.0625;
			break;
		case 3:
			reso = 0.125;
			break;
		case 4:
			reso = 0.25;
			break;
		case 5:
			reso = 0.5;
			break;
		case 6:
			reso = 1;
			break;
		case 7:
			reso = 2;
			break;
		default:
			LOG_WRN("iout_reso_set not supported: 0x%x", mfr_svi3_iout_prt_data);
			return SENSOR_FAIL_TO_ACCESS;
		}
		return reso;
	default:
		LOG_WRN("offset not supported: 0x%x", offset);
		return SENSOR_FAIL_TO_ACCESS;
	}
}

uint8_t mp2891_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed");
		return SENSOR_FAIL_TO_ACCESS;
	}

	float val;
	if (cfg->offset == PMBUS_READ_VOUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		float resolution = mp2891_get_resolution(cfg);
		if (resolution == 0)
			return SENSOR_FAIL_TO_ACCESS;

		val = slinear11_to_float(read_value) * resolution;

	} else if (cfg->offset == PMBUS_READ_TEMPERATURE_1 || cfg->offset == PMBUS_READ_POUT) {
		uint16_t read_value = (msg.data[1] << 8) | msg.data[0];
		val = slinear11_to_float(read_value);

	} else if (cfg->offset == PMBUS_READ_IOUT) {
		uint16_t read_value = ((msg.data[1] << 8) | msg.data[0]) & IOUT_MASK;
		float resolution = mp2891_get_resolution(cfg);
		if (resolution == 0)
			return SENSOR_FAIL_TO_ACCESS;

		val = (float)read_value * resolution;

	} else {
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = val;
	sval->fraction = (val - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t mp2891_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = mp2891_read;
	return SENSOR_INIT_SUCCESS;
}
