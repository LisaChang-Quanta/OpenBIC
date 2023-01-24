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

#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/peci.h>
#include "sensor.h"
#include "hal_peci.h"
#include "libutil.h"
#include "intel_peci.h"
#include "ipmi.h"
#include "util_sys.h"
#include "intel_dimm.h"

LOG_MODULE_REGISTER(dev_intel_peci);

#define DIMM_TEMP_OFS_0 0x01
#define DIMM_TEMP_OFS_1 0x02

static intel_peci_unit unit_info;

static bool get_power_sku_unit(uint8_t addr)
{
	static int is_read_before;

	if (is_read_before)
		return true;

	uint8_t command = PECI_RD_PKG_CFG0_CMD;
	uint8_t readlen = 0x05;

	int ret = 0;
	uint8_t *readbuf = (uint8_t *)malloc(readlen * sizeof(uint8_t));
	if (!readbuf) {
		LOG_ERR("[%s] fail to allocate readbuf memory\n", __func__);
		return false;
	}
	ret = peci_read(command, addr, RDPKG_IDX_PWR_SKU_UNIT_READ, 0, readlen, readbuf);
	if (ret) {
		LOG_DBG("[%s] peci read error\n", __func__);
		goto cleanup;
	}

	if (readbuf[0] != PECI_CC_RSP_SUCCESS) {
		if (readbuf[0] == PECI_CC_ILLEGAL_REQUEST) {
			LOG_ERR("[%s] Unknown request\n", __func__);
		} else {
			LOG_ERR("[%s] PECI control hardware, firmware or associated logic error\n",
			       __func__);
		}
		goto cleanup;
	}

	uint32_t pwr_sku_unit;
	pwr_sku_unit = readbuf[1] | (readbuf[2] << 8) | (readbuf[3] << 16) | (readbuf[4] << 24);
	unit_info.time_unit = (pwr_sku_unit >> 16) & 0xF;
	unit_info.energy_unit = (pwr_sku_unit >> 8) & 0x1F;
	unit_info.power_unit = pwr_sku_unit & 0xF;

	is_read_before = 1;

	SAFE_FREE(readbuf);
	return true;

cleanup:
	SAFE_FREE(readbuf);
	return false;
}

bool check_dimm_present(uint8_t dimm_channel, uint8_t dimm_num, uint8_t *present_result)
{
	// BIC can check DIMM present or not by NM IPMI get CPU and memory temperature commnad
	if (present_result == NULL) {
		LOG_ERR("[%s] input present result pointer is NULL\n", __func__);
		return false;
	}

	ipmi_msg *dimm_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (dimm_msg == NULL) {
		LOG_ERR("[%s] fail to allocate dimm message memory\n", __func__);
		return false;
	}
	memset(dimm_msg, 0, sizeof(ipmi_msg));

	get_cpu_memory_temp_req *get_dimm_temp_req =
		(get_cpu_memory_temp_req *)malloc(sizeof(get_cpu_memory_temp_req));
	if (get_dimm_temp_req == NULL) {
		LOG_ERR("[%s] fail to allocate request array memory\n", __func__);
		SAFE_FREE(dimm_msg);
		return false;
	}
	memset(get_dimm_temp_req, 0, sizeof(get_cpu_memory_temp_req));

	bool ret = false;
	uint32_t intel_iana = INTEL_IANA;

	memcpy(&get_dimm_temp_req->intel_id[0], (uint8_t *)&intel_iana, 3); // Intel Manufacturer ID
	if (dimm_channel < DIMM_CHANNEL_NUM_4) {
		get_dimm_temp_req->set_memory_channel = MEMORY_CHANNEL_0_TO_3;
	} else {
		get_dimm_temp_req->set_memory_channel = MEMORY_CHANNEL_4_TO_7;
	}

	switch (dimm_channel) {
	case DIMM_CHANNEL_NUM_0:
	case DIMM_CHANNEL_NUM_4:
		switch (dimm_num) {
		case DIMM_NUMBER_0:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_0_4_DIMM_NUM_0;
			break;
		case DIMM_NUMBER_1:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_0_4_DIMM_NUM_1;
			break;
		case DIMM_NUMBER_2:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_0_4_DIMM_NUM_2;
			break;
		case DIMM_NUMBER_3:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_0_4_DIMM_NUM_3;
			break;
		}
		break;
	case DIMM_CHANNEL_NUM_1:
	case DIMM_CHANNEL_NUM_5:
		switch (dimm_num) {
		case DIMM_NUMBER_0:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_1_5_DIMM_NUM_0;
			break;
		case DIMM_NUMBER_1:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_1_5_DIMM_NUM_1;
			break;
		case DIMM_NUMBER_2:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_1_5_DIMM_NUM_2;
			break;
		case DIMM_NUMBER_3:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_1_5_DIMM_NUM_3;
			break;
		}
		break;
	case DIMM_CHANNEL_NUM_2:
	case DIMM_CHANNEL_NUM_6:
		switch (dimm_num) {
		case DIMM_NUMBER_0:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_2_6_DIMM_NUM_0;
			break;
		case DIMM_NUMBER_1:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_2_6_DIMM_NUM_1;
			break;
		case DIMM_NUMBER_2:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_2_6_DIMM_NUM_2;
			break;
		case DIMM_NUMBER_3:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_2_6_DIMM_NUM_3;
			break;
		}
		break;
	case DIMM_CHANNEL_NUM_3:
	case DIMM_CHANNEL_NUM_7:
		switch (dimm_num) {
		case DIMM_NUMBER_0:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_3_7_DIMM_NUM_0;
			break;
		case DIMM_NUMBER_1:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_3_7_DIMM_NUM_1;
			break;
		case DIMM_NUMBER_2:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_3_7_DIMM_NUM_2;
			break;
		case DIMM_NUMBER_3:
			get_dimm_temp_req->cpu0_read_dimm_req = CHANNEL_3_7_DIMM_NUM_3;
			break;
		}
		break;
	default:
		LOG_ERR("[%s] dimm channel is invalid, dimm channel: %d\n", __func__, dimm_channel);
		goto safe_free;
		break;
	}

	uint8_t seq_source = 0xFF;
	*dimm_msg = construct_ipmi_message(seq_source, NETFN_NM_REQ, CMD_GET_CPU_MEMORY_TEMP, SELF,
					   ME_IPMB, CMD_GET_CPU_MEMORY_TEMP_DATA_LEN,
					   (uint8_t *)get_dimm_temp_req);
	ipmb_error ipmb_ret = ipmb_read(dimm_msg, IPMB_inf_index_map[dimm_msg->InF_target]);
	if ((ipmb_ret != IPMB_ERROR_SUCCESS) || (dimm_msg->completion_code != CC_SUCCESS)) {
		LOG_ERR("[%s] fail to send get dimm temperature command ret: 0x%x CC: 0x%x\n",
		       __func__, ipmb_ret, dimm_msg->completion_code);
		goto safe_free;
	}

	*present_result = dimm_msg->data[RESPONSE_DIMM_TEMP_INDEX];
	ret = true;

safe_free:
	SAFE_FREE(dimm_msg);
	SAFE_FREE(get_dimm_temp_req);
	return ret;
}

static bool read_cpu_power(uint8_t addr, int *reading)
{
	if (reading == NULL) {
		LOG_DBG("[%s] invalid argument\n", __func__);
		return false;
	}

	uint8_t command = PECI_RD_PKG_CFG0_CMD;
	uint8_t readlen = 0x05;
	uint8_t u8index[2] = { 0x03, 0x1F };
	uint16_t u16Param[2] = { 0x00FF, 0x0000 };
	int ret = 0;
	uint32_t pkg_energy, run_time, diff_energy, diff_time;
	static uint32_t last_pkg_energy = 0, last_run_time = 0;

	uint8_t *readbuf = (uint8_t *)malloc(2 * readlen * sizeof(uint8_t));
	if (!readbuf) {
		LOG_ERR("[%s] fail to allocate readbuf memory\n", __func__);
		return false;
	}

	ret = peci_read(command, addr, u8index[0], u16Param[0], readlen, readbuf);
	if (ret) {
		LOG_DBG("[%s] peci read error\n", __func__);
		goto cleanup;
	}
	ret = peci_read(command, addr, u8index[1], u16Param[1], readlen, &readbuf[5]);
	if (ret) {
		LOG_DBG("[%s] peci read error\n", __func__);
		goto cleanup;
	}
	if (readbuf[0] != PECI_CC_RSP_SUCCESS || readbuf[5] != PECI_CC_RSP_SUCCESS) {
		if (readbuf[0] == PECI_CC_ILLEGAL_REQUEST ||
		    readbuf[5] == PECI_CC_ILLEGAL_REQUEST) {
			LOG_ERR("[%s] Unknown request\n", __func__);
		} else {
			LOG_ERR("[%s] PECI control hardware, firmware or associated logic error\n",
			       __func__);
		}
		goto cleanup;
	}

	if (!get_power_sku_unit(addr)) {
		LOG_ERR("[%s] PECI get power sku unit failed!\n", __func__);
		goto cleanup;
	}

	pkg_energy = readbuf[4];
	pkg_energy = (pkg_energy << 8) | readbuf[3];
	pkg_energy = (pkg_energy << 8) | readbuf[2];
	pkg_energy = (pkg_energy << 8) | readbuf[1];

	run_time = readbuf[9];
	run_time = (run_time << 8) | readbuf[8];
	run_time = (run_time << 8) | readbuf[7];
	run_time = (run_time << 8) | readbuf[6];

	if (last_pkg_energy == 0 &&
	    last_run_time == 0) { // first read, need second data to calculate
		last_pkg_energy = pkg_energy;
		last_run_time = run_time;
		LOG_DBG("[%s] cpu power first read\n", __func__);
		goto cleanup;
	}

	if (pkg_energy >= last_pkg_energy) {
		diff_energy = pkg_energy - last_pkg_energy;
	} else {
		diff_energy = pkg_energy + (0xffffffff - last_pkg_energy + 1);
	}
	last_pkg_energy = pkg_energy;

	if (run_time >= last_run_time) {
		diff_time = run_time - last_run_time;
	} else {
		diff_time = run_time + (0xffffffff - last_run_time + 1);
	}
	last_run_time = run_time;

	if (diff_time == 0) {
		LOG_DBG("[%s] cpu power time elapsed is zero\n", __func__);
		goto cleanup;
	}

	float pwr_scale = 1;
	if (unit_info.energy_unit > unit_info.time_unit)
		pwr_scale =
			(float)(1 / (float)(1 << (unit_info.energy_unit - unit_info.time_unit)));
	else
		pwr_scale = (float)(1 << (unit_info.energy_unit - unit_info.time_unit));

	*reading = ((float)diff_energy / (float)diff_time) * pwr_scale;

	SAFE_FREE(readbuf);
	return true;

cleanup:
	SAFE_FREE(readbuf);
	return false;
}

static bool get_cpu_tjmax(uint8_t addr, int *reading)
{
	if (!reading) {
		LOG_DBG("[%s] invalid argument\n", __func__);
		return false;
	}

	const uint16_t param = 0x00;
	const uint8_t rlen = 0x05;
	uint8_t rbuf[rlen];
	memset(rbuf, 0, sizeof(rbuf));

	int ret = peci_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_TJMAX_TEMP, param, rlen, rbuf);
	if (ret != 0) {
		LOG_DBG("[%s] peci read error\n", __func__);
		return false;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = rbuf[3];
	return true;
}

static bool get_cpu_margin(uint8_t addr, int *reading)
{
	if (!reading) {
		LOG_DBG("[%s] invalid argument\n", __func__);
		return false;
	}

	const uint16_t param = 0xFF;
	const uint8_t rlen = 0x05;
	uint8_t rbuf[rlen];
	memset(rbuf, 0, sizeof(rbuf));

	int ret = peci_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_PKG_TEMP, param, rlen, rbuf);
	if (ret != 0) {
		LOG_DBG("[%s] peci read error\n", __func__);
		return false;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = ((int16_t)((rbuf[2] << 8) | rbuf[1]) >> 6) + 1;
	return true;
}

static bool get_cpu_pwr(uint8_t addr, int *reading)
{
	if (!reading) {
		LOG_DBG("[%s] invalid argument\n", __func__);
		return false;
	}

	int pwr = 0;
	if (read_cpu_power(addr, &pwr) == false) {
		LOG_DBG("[%s] read cpu power error\n", __func__);
		return false;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int16_t)pwr;
	return true;
}

static bool get_cpu_temp(uint8_t addr, int *reading)
{
	if (!reading) {
		LOG_DBG("[%s] invalid argument\n", __func__);
		return false;
	}

	sensor_val tjmax = { 0 };
	if (get_cpu_tjmax(addr, (int *)&tjmax) == false) {
		LOG_DBG("[%s] get cpu tjmax error\n", __func__);
		return false;
	}

	sensor_val margin = { 0 };
	if (get_cpu_margin(addr, (int *)&margin) == false){
		LOG_DBG("[%s] get cpu margin error\n", __func__);
		return false;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = tjmax.integer + margin.integer;
	return true;
}

static bool get_dimm_temp(uint8_t addr, uint8_t type, int *reading)
{
	if (!reading) {
		return false;
	}

	uint8_t temp_ofs = 0xFF;
	uint16_t param = 0xFF;
	switch (type) {
	case PECI_TEMP_CHANNEL0_DIMM0:
		param = 0x00;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL0_DIMM1:
		param = 0x00;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL1_DIMM0:
		param = 0x01;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL1_DIMM1:
		param = 0x01;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL2_DIMM0:
		param = 0x02;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL2_DIMM1:
		param = 0x02;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL3_DIMM0:
		param = 0x03;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL3_DIMM1:
		param = 0x03;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL4_DIMM0:
		param = 0x04;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL4_DIMM1:
		param = 0x04;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL5_DIMM0:
		param = 0x05;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL5_DIMM1:
		param = 0x05;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL6_DIMM0:
		param = 0x06;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL6_DIMM1:
		param = 0x06;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	case PECI_TEMP_CHANNEL7_DIMM0:
		param = 0x07;
		temp_ofs = DIMM_TEMP_OFS_0;
		break;
	case PECI_TEMP_CHANNEL7_DIMM1:
		param = 0x07;
		temp_ofs = DIMM_TEMP_OFS_1;
		break;
	default:
		break;
	}

	if (param == 0xFF || temp_ofs == 0xFF) {
		LOG_DBG("[%s] Unsupported peci temp channel %d\n", __func__, type);
		return false;
	}

	const uint8_t rlen = 0x05;
	uint8_t rbuf[rlen];
	memset(rbuf, 0, sizeof(rbuf));

	if (peci_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_DIMM_TEMP, param, rlen, rbuf) != 0) {
		LOG_DBG("[%s] peci read error\n", __func__);
		return false;
	}
	sensor_val *sval = (sensor_val *)reading;
	sval->integer = rbuf[temp_ofs];
	return true;
}

uint8_t intel_peci_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		LOG_DBG("[%s] Invalid argument\n", __func__);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	bool ret_val = false;
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	const uint8_t read_type = cfg->offset;
	if (read_type <= PECI_UNKNOWN || read_type >= PECI_MAX) {
		LOG_DBG("[%s] Sensor not found\n", __func__);
		return SENSOR_NOT_FOUND;
	}

	switch (read_type) {
	case PECI_TEMP_CHANNEL0_DIMM0:
	case PECI_TEMP_CHANNEL0_DIMM1:
	case PECI_TEMP_CHANNEL1_DIMM0:
	case PECI_TEMP_CHANNEL1_DIMM1:
	case PECI_TEMP_CHANNEL2_DIMM0:
	case PECI_TEMP_CHANNEL2_DIMM1:
	case PECI_TEMP_CHANNEL3_DIMM0:
	case PECI_TEMP_CHANNEL3_DIMM1:
	case PECI_TEMP_CHANNEL4_DIMM0:
	case PECI_TEMP_CHANNEL4_DIMM1:
	case PECI_TEMP_CHANNEL5_DIMM0:
	case PECI_TEMP_CHANNEL5_DIMM1:
	case PECI_TEMP_CHANNEL6_DIMM0:
	case PECI_TEMP_CHANNEL6_DIMM1:
	case PECI_TEMP_CHANNEL7_DIMM0:
	case PECI_TEMP_CHANNEL7_DIMM1:
		ret_val = get_dimm_temp(cfg->target_addr, read_type, reading);
		break;
	case PECI_TEMP_CPU_MARGIN:
		ret_val = get_cpu_margin(cfg->target_addr, reading);
		break;
	case PECI_TEMP_CPU_TJMAX:
		ret_val = get_cpu_tjmax(cfg->target_addr, reading);
		break;
	case PECI_TEMP_CPU:
		ret_val = get_cpu_temp(cfg->target_addr, reading);
		break;
	case PECI_PWR_CPU:
		ret_val = get_cpu_pwr(cfg->target_addr, reading);
		break;
	default:
		break;
	}

	if (!ret_val) {
		LOG_DBG("[%s] Sensor access error\n", __func__);
	}

	return ret_val ? SENSOR_READ_SUCCESS : SENSOR_FAIL_TO_ACCESS;
}

uint8_t intel_peci_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		LOG_DBG("[%s] PECI init failed, sensor_num exceeds SENSOR_NUM_MAX\n", __func__);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	static bool is_init = false;
	sensor_config[sensor_config_index_map[sensor_num]].read = intel_peci_read;
	if (!is_init) {
		int ret;
		ret = peci_init();
		if (ret) {
			LOG_DBG("[%s] PECI init error\n", __func__);
			return SENSOR_INIT_UNSPECIFIED_ERROR;
		}
		is_init = true;
	}
	return SENSOR_INIT_SUCCESS;
}
