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

#ifndef AV1_COMMON_ENTROPYMODE_H_
#define AV1_COMMON_ENTROPYMODE_H_


#ifdef __cplusplus
extern "C" {
#endif

#define BLOCK_SIZE_GROUPS 4

#define TX_SIZE_CONTEXTS 3

#define INTER_OFFSET(mode) ((mode)-NEARESTMV)
#define INTER_COMPOUND_OFFSET(mode) (uint8_t)((mode)-NEAREST_NEARESTMV)

// Number of possible contexts for a color index.
// As can be seen from av1_get_palette_color_index_context(), the possible
// contexts are (2,0,0), (2,2,1), (3,2,0), (4,1,0), (5,0,0). These are mapped to
// a value from 0 to 4 using 'palette_color_index_context_lookup' table.
#define PALETTE_COLOR_INDEX_CONTEXTS 5

// Palette Y mode context for a block is determined by number of neighboring
// blocks (top and/or left) using a palette for Y plane. So, possible Y mode'
// context values are:
// 0 if neither left nor top block uses palette for Y plane,
// 1 if exactly one of left or top block uses palette for Y plane, and
// 2 if both left and top blocks use palette for Y plane.
#define PALETTE_Y_MODE_CONTEXTS 3

// Palette UV mode context for a block is determined by whether this block uses
// palette for the Y plane. So, possible values are:
// 0 if this block doesn't use palette for Y plane.
// 1 if this block uses palette for Y plane (i.e. Y palette size > 0).
#define PALETTE_UV_MODE_CONTEXTS 2

// Map the number of pixels in a block size to a context
//   16(BLOCK_4X4)                          -> 0
//   32(BLOCK_4X8, BLOCK_8X4)               -> 1
//   64(BLOCK_8X8, BLOCK_4x16, BLOCK_16X4)  -> 2
//   ...
// 4096(BLOCK_64X64)                        -> 8
#define PALATTE_BSIZE_CTXS 9

#define KF_MODE_CONTEXTS 5


#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AV1_COMMON_ENTROPYMODE_H_
