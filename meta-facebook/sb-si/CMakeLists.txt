#SPDX - License - Identifier : Apache - 2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sb-rb)

set(common_path ../../common)
FILE(GLOB app_sources src/ipmi/*.c src/platform/*.c src/lib/*.c src/shell/*.c)
FILE(GLOB common_sources ${common_path}/service/*.c ${common_path}/service/*/*.c ${common_path}/hal/*.c ${common_path}/dev/*.c ${common_path}/shell/*.c ${common_path}/shell/commands/*.c)

target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ${common_sources})

# Common Lib
target_sources(app PRIVATE ${common_path}/lib/libutil.c)
target_sources(app PRIVATE ${common_path}/lib/power_status.c)
target_sources(app PRIVATE ${common_path}/lib/timer.c)
target_sources(app PRIVATE ${common_path}/lib/util_pmbus.c)
target_sources(app PRIVATE ${common_path}/lib/util_spi.c)
target_sources(app PRIVATE ${common_path}/lib/util_sys.c)
target_sources(app PRIVATE ${common_path}/lib/util_worker.c)

target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include/portability)
target_include_directories(app PRIVATE ${common_path}/include)
target_include_directories(app PRIVATE ${common_path} ${common_path}/service/ipmi/include ${common_path}/service/host ${common_path}/service/sensor ${common_path}/service/usb ${common_path}/service/ipmb ${common_path}/service/mctp ${common_path}/service/pldm ${common_path}/hal ${common_path}/dev/include ${common_path}/lib ${common_path}/shell)
target_include_directories(app PRIVATE ${common_path}/shell/commands)
target_include_directories(app PRIVATE src/ipmi/include src/platform src/lib)

# Platform Specific Includes
target_include_directories(app PRIVATE src/lib)
target_include_directories(app PRIVATE src/platform)
target_include_directories(app PRIVATE src/shell)

# Fail build if there are any warnings 
target_compile_options(app PRIVATE -Werror)

add_compile_definitions(PLDM_MAX_DATA_SIZE=1300)
