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

typedef struct _vr_pre_read_arg {
	/* vr page to set */
	uint8_t vr_page;
} vr_pre_read_arg;

extern adc_asd_init_arg ast_adc_init_args[];
extern ina233_init_arg ina233_init_args[];
extern vr_pre_read_arg vr_pre_read_args[];
extern max11617_init_arg max11617_init_args[];
extern adc128d818_init_arg adc128d818_init_args[];
extern rtq6056_init_arg rtq6056_init_args[];
extern sq52205_init_arg sq52205_init_args[];
extern vistara_init_arg vistara_init_args[];

bool pre_vr_read(sensor_cfg *cfg, void *args);
bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading);
bool post_p085v_voltage_read(sensor_cfg *cfg, void *args, int *reading);
bool post_adc128d818_read(sensor_cfg *cfg, void *args, int *reading);

#endif
