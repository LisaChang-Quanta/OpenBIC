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

#include <shell/shell.h>
#include <stdlib.h>
#include <logging/log.h>
#include "sensor.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_power_shell, LOG_LEVEL_DBG);

static int cmd_vr_get_all(const struct shell *shell, size_t argc, char **argv)
{
	LOG_DBG("argc: %d", argc);

	for (int i = 0; i < argc; i++)
		shell_info(shell, "argv[%d]: %s", i, argv[i]);

	/* TODO: list all vr sensor value */
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if ((get_board_type() == MINERVA_AEGIS_BD) && (i == 0))
			continue; // skip osfp p3v3 on AEGIS BD

		uint16_t vout = 0;
		uint8_t *sensor_name = NULL;
		if (!vr_rail_name_get((uint8_t)i, &sensor_name)) {
			shell_print(shell, "Can't find vr_rail_name by rail index: %d", i);
			continue;
		}

		if (!plat_get_vout_command(i, &vout)) {
			shell_print(shell, "Can't find vout by rail index: %d", i);
			continue;
		}

		shell_print(shell, "%4d|%-40s|%04x|%4d", i, sensor_name, vout, vout);
	}

	return 0;
}

static int cmd_vr_set(const struct shell *shell, size_t argc, char **argv)
{
	for (int i = 0; i < argc; i++)
		shell_info(shell, "argv[%d]: %s", i, argv[i]);

	if (argc == 4) {
		if (strcmp(argv[3], "perm")) {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	/* covert rail string to enum */
	enum VR_RAIL_E rail;
	if (vr_fail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	/* covert voltage to millivoltage */
	uint16_t mv = strtol(argv[2], NULL, 0);

	shell_info(shell, "Set %s(%d) to %d mV, %svolatile\n", argv[1], rail, mv,
		   (argc == 4) ? "non-" : "");

	/* TODO: set the vout */
	if ((get_board_type() == MINERVA_AEGIS_BD) && (rail == 0)) {
		shell_print(shell, "skip osfp p3v3 on AEGIS BD");
		return 0;
	}

	if (strcmp(argv[2], "default") == 0) {
		if (!plat_set_vout_command(rail, mv, true, false)) {
			shell_print(shell, "Can't set vout by rail index: %d", rail);
		}
	} else {
		if (!plat_set_vout_command(rail, mv, false, false)) {
			shell_print(shell, "Can't set vout by rail index: %d", rail);
		}
		/* TODO: save settings if perm arg exists */
		if (argc == 4) {
		}
	}

	return 0;
}

static void vr_rname_get(size_t idx, struct shell_static_entry *entry)
{
	uint8_t *name = NULL;
	vr_rail_name_get((uint8_t)idx, &name);

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(vr_rname, vr_rname_get);

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_vr_get_cmds,
			       SHELL_CMD(all, NULL, "get vr all volt/curr/power", cmd_vr_get_all),
			       SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_vr_cmds, SHELL_CMD(get, &sub_vr_get_cmds, "", NULL),
			       SHELL_CMD_ARG(set, &vr_rname,
					     "set <voltage-rail> <new-voltage>|default [perm]",
					     cmd_vr_set, 3, 1),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(voltage, &sub_vr_cmds, "vr set/get commands", NULL);
