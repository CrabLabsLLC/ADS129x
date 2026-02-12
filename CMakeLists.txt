# Copyright (c) 2025 Makani Science
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_include_directories(include)
zephyr_library_sources(
    src/ads1293.c
    src/ads1293_hal_zephyr.c
)
