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

#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

#include "sensor.h"
#include "plat_pldm_sensor.h"

#define VR_MAX_NUM 11
#define VR_MUTEX_LOCK_TIMEOUT_MS 1000

enum VR_INDEX_E {
	VR_INDEX_E_OSFP_P3V3 = 0,
	VR_INDEX_E_P0V85,
	VR_INDEX_E_P0V75_CH_N,
	VR_INDEX_E_P0V75_CH_S,
	VR_INDEX_E_P0V75_TRVDD_ZONEA,
	VR_INDEX_E_P0V75_TRVDD_ZONEB,
	VR_INDEX_E_P1V1_VDDC_HBM0_HBM2_HBM4,
	VR_INDEX_E_P0V9_TRVDD_ZONEA,
	VR_INDEX_E_P0V9_TRVDD_ZONEB,
	VR_INDEX_E_P1V1_VDDC_HBM1_HBM3_HBM5,
	VR_INDEX_E_VDDA_PCIE,
	VR_INDEX_MAX,
};

enum VR_RAIL_E {
	VR_RAIL_E_OSFP_P3V3 = 0,
	VR_RAIL_E_P0V85,
	VR_RAIL_E_P0V75_PVDD_CH_N,
	VR_RAIL_E_P0V75_MAX_PHY_N,
	VR_RAIL_E_P0V75_PVDD_CH_S,
	VR_RAIL_E_P0V75_MAX_PHY_S,
	VR_RAIL_E_P0V75_TRVDD_ZONEA,
	VR_RAIL_E_P1V8_VPP_HBM0_2_4,
	VR_RAIL_E_P0V75_TRVDD_ZONEB,
	VR_RAIL_E_P0V4_VDDQL_HBM0_2_4,
	VR_RAIL_E_P1V1_VDDC_HBM0_2_4,
	VR_RAIL_E_P0V75_VDDPHY_HBM0_2_4,
	VR_RAIL_E_P0V9_TRVDD_ZONEA,
	VR_RAIL_E_P1V8_VPP_HBM1_3_5,
	VR_RAIL_E_P0V9_TRVDD_ZONEB,
	VR_RAIL_E_P0V4_VDDQL_HBM1_3_5,
	VR_RAIL_E_P1V1_VDDC_HBM1_3_5,
	VR_RAIL_E_P0V75_VDDPHY_HBM1_3_5,
	VR_RAIL_E_P0V8_VDDA_PCIE,
	VR_RAIL_E_P1V2_VDDHTX_PCIE,
	VR_RAIL_E_MAX,
};

typedef struct vr_mapping_sensor {
	uint8_t index;
	uint8_t sensor_id;
	uint8_t *sensor_name;
} vr_mapping_sensor;

extern vr_mapping_sensor vr_rail_table[];

typedef struct _vr_pre_proc_arg {
	void *mutex;
	uint8_t vr_page;
} vr_pre_proc_arg;

extern vr_pre_proc_arg vr_pre_read_args[];
extern mp2971_init_arg mp2971_init_args[];
extern isl69259_init_arg isl69259_init_args[];

bool pre_vr_read(sensor_cfg *cfg, void *args);
bool post_vr_read(sensor_cfg *cfg, void *args, int *reading);
bool is_mb_dc_on();
void *vr_mutex_get(enum VR_INDEX_E vr_index);
void vr_mutex_init(void);
bool vr_rail_name_get(uint8_t rail, uint8_t **name);
bool vr_fail_enum_get(uint8_t *name, uint8_t *num);
bool vr_vout_user_settings_get(void *user_settings);
void user_settings_init(void);
void default_settings_init(void);
bool plat_get_vout_command(uint8_t rail, uint16_t *vout);
bool plat_set_vout_command(uint8_t rail, uint16_t vout, bool is_default, bool is_perm);

#endif
