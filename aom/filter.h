/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#ifndef AV1_COMMON_FILTER_H_
#define AV1_COMMON_FILTER_H_

#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum ATTRIBUTE_PACKED {
  EIGHTTAP_REGULAR = 0,
  EIGHTTAP = EIGHTTAP_REGULAR,
  EIGHTTAP_SMOOTH  = 1,
  MULTITAP_SHARP = 2,
  EIGHTTAP_SHARP = MULTITAP_SHARP,
  BILINEAR = 3,
  INTERP_FILTERS_ALL,
  SWITCHABLE_FILTERS = BILINEAR,
  SWITCHABLE = SWITCHABLE_FILTERS + 1, /* the last switchable one */
  EXTRA_FILTERS = INTERP_FILTERS_ALL - SWITCHABLE_FILTERS,
} InterpFilter;
#define LOG_SWITCHABLE_FILTERS \
  2 /* (1 << LOG_SWITCHABLE_FILTERS) > SWITCHABLE_FILTERS */

#define MAX_SUBPEL_TAPS 12
#define SWITCHABLE_FILTER_CONTEXTS ((SWITCHABLE_FILTERS + 1) * 4)
#define INTER_FILTER_COMP_OFFSET (SWITCHABLE_FILTERS + 1)
#define INTER_FILTER_DIR_OFFSET ((SWITCHABLE_FILTERS + 1) * 2)

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AV1_COMMON_FILTER_H_
