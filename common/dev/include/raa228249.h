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
#ifndef RAA228249_H
#define RAA228249_H

#include "stdint.h"
#include "sensor.h"

bool raa228249_get_crc(uint8_t bus, uint8_t addr, uint32_t *crc);
bool raa228249_get_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool raa228249_get_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool raa228249_set_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool raa228249_set_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
int raa228249_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *remain);
bool raa228249_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size);
bool raa228249_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool raa228249_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool raa228249_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
			     uint16_t *vr_status);
bool raa228249_clear_vr_status(sensor_cfg *cfg, uint8_t rail);

#endif
