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

#ifndef AV1_COMMON_ENTROPY_H_
#define AV1_COMMON_ENTROPY_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TOKEN_CDF_Q_CTXS 4

#define TXB_SKIP_CONTEXTS 13

#define EOB_COEF_CONTEXTS 22

#define SIG_COEF_CONTEXTS_2D 26
#define SIG_COEF_CONTEXTS_1D 16
#define SIG_COEF_CONTEXTS_EOB 4
#define SIG_COEF_CONTEXTS (SIG_COEF_CONTEXTS_2D + SIG_COEF_CONTEXTS_1D)

#define COEFF_BASE_CONTEXTS (SIG_COEF_CONTEXTS)
#define DC_SIGN_CONTEXTS 3

#define BR_TMP_OFFSET 12
#define BR_REF_CAT 4
#define LEVEL_CONTEXTS 21

#define NUM_BASE_LEVELS 2

#define BR_CDF_SIZE (4)
#define COEFF_BASE_RANGE (4 * (BR_CDF_SIZE - 1))

#define COEFF_CONTEXT_BITS 6
#define COEFF_CONTEXT_MASK ((1 << COEFF_CONTEXT_BITS) - 1)
#define MAX_BASE_BR_RANGE (COEFF_BASE_RANGE + NUM_BASE_LEVELS + 1)

#define BASE_CONTEXT_POSITION_NUM 12

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AV1_COMMON_ENTROPY_H_
