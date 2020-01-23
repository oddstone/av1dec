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

#ifndef AOM_DSP_PROB_H_
#define AOM_DSP_PROB_H_

#include <assert.h>
#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// TODO(negge): Rename this aom_prob once we remove vpxbool.
typedef uint16_t aom_cdf_prob;

#define CDF_SIZE(x) ((x) + 1)
#define CDF_PROB_BITS 15
#define CDF_PROB_TOP (1 << CDF_PROB_BITS)
#define CDF_INIT_TOP 32768
#define CDF_SHIFT (15 - CDF_PROB_BITS)
/*The value stored in an iCDF is CDF_PROB_TOP minus the actual cumulative
  probability (an "inverse" CDF).
  This function converts from one representation to the other (and is its own
  inverse).*/
#define AOM_ICDF(x) (CDF_PROB_TOP - (x))

#if CDF_SHIFT == 0

#define AOM_CDF2(a0) AOM_ICDF(a0), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF3(a0, a1) AOM_ICDF(a0), AOM_ICDF(a1), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF4(a0, a1, a2) \
  AOM_ICDF(a0), AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF5(a0, a1, a2, a3) \
  AOM_ICDF(a0)                   \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF6(a0, a1, a2, a3, a4)                        \
  AOM_ICDF(a0)                                              \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF7(a0, a1, a2, a3, a4, a5)                                  \
  AOM_ICDF(a0)                                                            \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5), \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF8(a0, a1, a2, a3, a4, a5, a6)                              \
  AOM_ICDF(a0)                                                            \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5), \
      AOM_ICDF(a6), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF9(a0, a1, a2, a3, a4, a5, a6, a7)                          \
  AOM_ICDF(a0)                                                            \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5), \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF10(a0, a1, a2, a3, a4, a5, a6, a7, a8)                     \
  AOM_ICDF(a0)                                                            \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5), \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(a8), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF11(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9)                 \
  AOM_ICDF(a0)                                                            \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5), \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(a8), AOM_ICDF(a9),             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF12(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10)               \
  AOM_ICDF(a0)                                                               \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5),    \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(a8), AOM_ICDF(a9), AOM_ICDF(a10), \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF13(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11)          \
  AOM_ICDF(a0)                                                               \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5),    \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(a8), AOM_ICDF(a9), AOM_ICDF(a10), \
      AOM_ICDF(a11), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF14(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12)     \
  AOM_ICDF(a0)                                                               \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5),    \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(a8), AOM_ICDF(a9), AOM_ICDF(a10), \
      AOM_ICDF(a11), AOM_ICDF(a12), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF15(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13) \
  AOM_ICDF(a0)                                                                \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5),     \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(a8), AOM_ICDF(a9), AOM_ICDF(a10),  \
      AOM_ICDF(a11), AOM_ICDF(a12), AOM_ICDF(a13), AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF16(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, \
                  a14)                                                        \
  AOM_ICDF(a0)                                                                \
  , AOM_ICDF(a1), AOM_ICDF(a2), AOM_ICDF(a3), AOM_ICDF(a4), AOM_ICDF(a5),     \
      AOM_ICDF(a6), AOM_ICDF(a7), AOM_ICDF(a8), AOM_ICDF(a9), AOM_ICDF(a10),  \
      AOM_ICDF(a11), AOM_ICDF(a12), AOM_ICDF(a13), AOM_ICDF(a14),             \
      AOM_ICDF(CDF_PROB_TOP), 0

#else
#define AOM_CDF2(a0)                                       \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 2) + \
            ((CDF_INIT_TOP - 2) >> 1)) /                   \
               ((CDF_INIT_TOP - 2)) +                      \
           1)                                              \
  , AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF3(a0, a1)                                       \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 3) +     \
            ((CDF_INIT_TOP - 3) >> 1)) /                       \
               ((CDF_INIT_TOP - 3)) +                          \
           1)                                                  \
  ,                                                            \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 3) + \
                ((CDF_INIT_TOP - 3) >> 1)) /                   \
                   ((CDF_INIT_TOP - 3)) +                      \
               2),                                             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF4(a0, a1, a2)                                   \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 4) +     \
            ((CDF_INIT_TOP - 4) >> 1)) /                       \
               ((CDF_INIT_TOP - 4)) +                          \
           1)                                                  \
  ,                                                            \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 4) + \
                ((CDF_INIT_TOP - 4) >> 1)) /                   \
                   ((CDF_INIT_TOP - 4)) +                      \
               2),                                             \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 4) + \
                ((CDF_INIT_TOP - 4) >> 1)) /                   \
                   ((CDF_INIT_TOP - 4)) +                      \
               3),                                             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF5(a0, a1, a2, a3)                               \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 5) +     \
            ((CDF_INIT_TOP - 5) >> 1)) /                       \
               ((CDF_INIT_TOP - 5)) +                          \
           1)                                                  \
  ,                                                            \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 5) + \
                ((CDF_INIT_TOP - 5) >> 1)) /                   \
                   ((CDF_INIT_TOP - 5)) +                      \
               2),                                             \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 5) + \
                ((CDF_INIT_TOP - 5) >> 1)) /                   \
                   ((CDF_INIT_TOP - 5)) +                      \
               3),                                             \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 5) + \
                ((CDF_INIT_TOP - 5) >> 1)) /                   \
                   ((CDF_INIT_TOP - 5)) +                      \
               4),                                             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF6(a0, a1, a2, a3, a4)                           \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 6) +     \
            ((CDF_INIT_TOP - 6) >> 1)) /                       \
               ((CDF_INIT_TOP - 6)) +                          \
           1)                                                  \
  ,                                                            \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 6) + \
                ((CDF_INIT_TOP - 6) >> 1)) /                   \
                   ((CDF_INIT_TOP - 6)) +                      \
               2),                                             \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 6) + \
                ((CDF_INIT_TOP - 6) >> 1)) /                   \
                   ((CDF_INIT_TOP - 6)) +                      \
               3),                                             \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 6) + \
                ((CDF_INIT_TOP - 6) >> 1)) /                   \
                   ((CDF_INIT_TOP - 6)) +                      \
               4),                                             \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 6) + \
                ((CDF_INIT_TOP - 6) >> 1)) /                   \
                   ((CDF_INIT_TOP - 6)) +                      \
               5),                                             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF7(a0, a1, a2, a3, a4, a5)                       \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 7) +     \
            ((CDF_INIT_TOP - 7) >> 1)) /                       \
               ((CDF_INIT_TOP - 7)) +                          \
           1)                                                  \
  ,                                                            \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 7) + \
                ((CDF_INIT_TOP - 7) >> 1)) /                   \
                   ((CDF_INIT_TOP - 7)) +                      \
               2),                                             \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 7) + \
                ((CDF_INIT_TOP - 7) >> 1)) /                   \
                   ((CDF_INIT_TOP - 7)) +                      \
               3),                                             \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 7) + \
                ((CDF_INIT_TOP - 7) >> 1)) /                   \
                   ((CDF_INIT_TOP - 7)) +                      \
               4),                                             \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 7) + \
                ((CDF_INIT_TOP - 7) >> 1)) /                   \
                   ((CDF_INIT_TOP - 7)) +                      \
               5),                                             \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 7) + \
                ((CDF_INIT_TOP - 7) >> 1)) /                   \
                   ((CDF_INIT_TOP - 7)) +                      \
               6),                                             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF8(a0, a1, a2, a3, a4, a5, a6)                   \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 8) +     \
            ((CDF_INIT_TOP - 8) >> 1)) /                       \
               ((CDF_INIT_TOP - 8)) +                          \
           1)                                                  \
  ,                                                            \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 8) + \
                ((CDF_INIT_TOP - 8) >> 1)) /                   \
                   ((CDF_INIT_TOP - 8)) +                      \
               2),                                             \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 8) + \
                ((CDF_INIT_TOP - 8) >> 1)) /                   \
                   ((CDF_INIT_TOP - 8)) +                      \
               3),                                             \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 8) + \
                ((CDF_INIT_TOP - 8) >> 1)) /                   \
                   ((CDF_INIT_TOP - 8)) +                      \
               4),                                             \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 8) + \
                ((CDF_INIT_TOP - 8) >> 1)) /                   \
                   ((CDF_INIT_TOP - 8)) +                      \
               5),                                             \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 8) + \
                ((CDF_INIT_TOP - 8) >> 1)) /                   \
                   ((CDF_INIT_TOP - 8)) +                      \
               6),                                             \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 8) + \
                ((CDF_INIT_TOP - 8) >> 1)) /                   \
                   ((CDF_INIT_TOP - 8)) +                      \
               7),                                             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF9(a0, a1, a2, a3, a4, a5, a6, a7)               \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) +     \
            ((CDF_INIT_TOP - 9) >> 1)) /                       \
               ((CDF_INIT_TOP - 9)) +                          \
           1)                                                  \
  ,                                                            \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) + \
                ((CDF_INIT_TOP - 9) >> 1)) /                   \
                   ((CDF_INIT_TOP - 9)) +                      \
               2),                                             \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) + \
                ((CDF_INIT_TOP - 9) >> 1)) /                   \
                   ((CDF_INIT_TOP - 9)) +                      \
               3),                                             \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) + \
                ((CDF_INIT_TOP - 9) >> 1)) /                   \
                   ((CDF_INIT_TOP - 9)) +                      \
               4),                                             \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) + \
                ((CDF_INIT_TOP - 9) >> 1)) /                   \
                   ((CDF_INIT_TOP - 9)) +                      \
               5),                                             \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) + \
                ((CDF_INIT_TOP - 9) >> 1)) /                   \
                   ((CDF_INIT_TOP - 9)) +                      \
               6),                                             \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) + \
                ((CDF_INIT_TOP - 9) >> 1)) /                   \
                   ((CDF_INIT_TOP - 9)) +                      \
               7),                                             \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 9) + \
                ((CDF_INIT_TOP - 9) >> 1)) /                   \
                   ((CDF_INIT_TOP - 9)) +                      \
               8),                                             \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF10(a0, a1, a2, a3, a4, a5, a6, a7, a8)           \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) +     \
            ((CDF_INIT_TOP - 10) >> 1)) /                       \
               ((CDF_INIT_TOP - 10)) +                          \
           1)                                                   \
  ,                                                             \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               2),                                              \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               3),                                              \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               4),                                              \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               5),                                              \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               6),                                              \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               7),                                              \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               8),                                              \
      AOM_ICDF((((a8)-9) * ((CDF_INIT_TOP >> CDF_SHIFT) - 10) + \
                ((CDF_INIT_TOP - 10) >> 1)) /                   \
                   ((CDF_INIT_TOP - 10)) +                      \
               9),                                              \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF11(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9)        \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +      \
            ((CDF_INIT_TOP - 11) >> 1)) /                        \
               ((CDF_INIT_TOP - 11)) +                           \
           1)                                                    \
  ,                                                              \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               2),                                               \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               3),                                               \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               4),                                               \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               5),                                               \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               6),                                               \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               7),                                               \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               8),                                               \
      AOM_ICDF((((a8)-9) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) +  \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               9),                                               \
      AOM_ICDF((((a9)-10) * ((CDF_INIT_TOP >> CDF_SHIFT) - 11) + \
                ((CDF_INIT_TOP - 11) >> 1)) /                    \
                   ((CDF_INIT_TOP - 11)) +                       \
               10),                                              \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF12(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10)    \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +       \
            ((CDF_INIT_TOP - 12) >> 1)) /                         \
               ((CDF_INIT_TOP - 12)) +                            \
           1)                                                     \
  ,                                                               \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               2),                                                \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               3),                                                \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               4),                                                \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               5),                                                \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               6),                                                \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               7),                                                \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               8),                                                \
      AOM_ICDF((((a8)-9) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +   \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               9),                                                \
      AOM_ICDF((((a9)-10) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) +  \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               10),                                               \
      AOM_ICDF((((a10)-11) * ((CDF_INIT_TOP >> CDF_SHIFT) - 12) + \
                ((CDF_INIT_TOP - 12) >> 1)) /                     \
                   ((CDF_INIT_TOP - 12)) +                        \
               11),                                               \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF13(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11) \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +         \
            ((CDF_INIT_TOP - 13) >> 1)) /                           \
               ((CDF_INIT_TOP - 13)) +                              \
           1)                                                       \
  ,                                                                 \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               2),                                                  \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               3),                                                  \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               4),                                                  \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               5),                                                  \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               6),                                                  \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               7),                                                  \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               8),                                                  \
      AOM_ICDF((((a8)-9) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +     \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               9),                                                  \
      AOM_ICDF((((a9)-10) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +    \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               10),                                                 \
      AOM_ICDF((((a10)-11) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +   \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               11),                                                 \
      AOM_ICDF((((a11)-12) * ((CDF_INIT_TOP >> CDF_SHIFT) - 13) +   \
                ((CDF_INIT_TOP - 13) >> 1)) /                       \
                   ((CDF_INIT_TOP - 13)) +                          \
               12),                                                 \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF14(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12) \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +              \
            ((CDF_INIT_TOP - 14) >> 1)) /                                \
               ((CDF_INIT_TOP - 14)) +                                   \
           1)                                                            \
  ,                                                                      \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               2),                                                       \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               3),                                                       \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               4),                                                       \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               5),                                                       \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               6),                                                       \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               7),                                                       \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               8),                                                       \
      AOM_ICDF((((a8)-9) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +          \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               9),                                                       \
      AOM_ICDF((((a9)-10) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +         \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               10),                                                      \
      AOM_ICDF((((a10)-11) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +        \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               11),                                                      \
      AOM_ICDF((((a11)-12) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +        \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               12),                                                      \
      AOM_ICDF((((a12)-13) * ((CDF_INIT_TOP >> CDF_SHIFT) - 14) +        \
                ((CDF_INIT_TOP - 14) >> 1)) /                            \
                   ((CDF_INIT_TOP - 14)) +                               \
               13),                                                      \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF15(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13) \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +                   \
            ((CDF_INIT_TOP - 15) >> 1)) /                                     \
               ((CDF_INIT_TOP - 15)) +                                        \
           1)                                                                 \
  ,                                                                           \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               2),                                                            \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               3),                                                            \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               4),                                                            \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               5),                                                            \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               6),                                                            \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               7),                                                            \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               8),                                                            \
      AOM_ICDF((((a8)-9) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +               \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               9),                                                            \
      AOM_ICDF((((a9)-10) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +              \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               10),                                                           \
      AOM_ICDF((((a10)-11) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +             \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               11),                                                           \
      AOM_ICDF((((a11)-12) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +             \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               12),                                                           \
      AOM_ICDF((((a12)-13) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +             \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               13),                                                           \
      AOM_ICDF((((a13)-14) * ((CDF_INIT_TOP >> CDF_SHIFT) - 15) +             \
                ((CDF_INIT_TOP - 15) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 15)) +                                    \
               14),                                                           \
      AOM_ICDF(CDF_PROB_TOP), 0
#define AOM_CDF16(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, \
                  a14)                                                        \
  AOM_ICDF((((a0)-1) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +                   \
            ((CDF_INIT_TOP - 16) >> 1)) /                                     \
               ((CDF_INIT_TOP - 16)) +                                        \
           1)                                                                 \
  ,                                                                           \
      AOM_ICDF((((a1)-2) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               2),                                                            \
      AOM_ICDF((((a2)-3) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               3),                                                            \
      AOM_ICDF((((a3)-4) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               4),                                                            \
      AOM_ICDF((((a4)-5) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               5),                                                            \
      AOM_ICDF((((a5)-6) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               6),                                                            \
      AOM_ICDF((((a6)-7) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               7),                                                            \
      AOM_ICDF((((a7)-8) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               8),                                                            \
      AOM_ICDF((((a8)-9) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +               \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               9),                                                            \
      AOM_ICDF((((a9)-10) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +              \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               10),                                                           \
      AOM_ICDF((((a10)-11) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +             \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               11),                                                           \
      AOM_ICDF((((a11)-12) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +             \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               12),                                                           \
      AOM_ICDF((((a12)-13) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +             \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               13),                                                           \
      AOM_ICDF((((a13)-14) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +             \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               14),                                                           \
      AOM_ICDF((((a14)-15) * ((CDF_INIT_TOP >> CDF_SHIFT) - 16) +             \
                ((CDF_INIT_TOP - 16) >> 1)) /                                 \
                   ((CDF_INIT_TOP - 16)) +                                    \
               15),                                                           \
      AOM_ICDF(CDF_PROB_TOP), 0

#endif

/* Symbols for coding which components are zero jointly */
#define MV_JOINTS 4
enum MV_JOINT_TYPE {
    MV_JOINT_ZERO = 0,   /* Zero vector */
    MV_JOINT_HNZVZ = 1,  /* Vert zero, hor nonzero */
    MV_JOINT_HZVNZ = 2,  /* Hor zero, vert nonzero */
    MV_JOINT_HNZVNZ = 3, /* Both components nonzero */
} ;

/* Symbols for coding magnitude class of nonzero components */
#define MV_CLASSES 11
enum MV_CLASS_TYPE {
    MV_CLASS_0 = 0,   /* (0, 2]     integer pel */
    MV_CLASS_1 = 1,   /* (2, 4]     integer pel */
    MV_CLASS_2 = 2,   /* (4, 8]     integer pel */
    MV_CLASS_3 = 3,   /* (8, 16]    integer pel */
    MV_CLASS_4 = 4,   /* (16, 32]   integer pel */
    MV_CLASS_5 = 5,   /* (32, 64]   integer pel */
    MV_CLASS_6 = 6,   /* (64, 128]  integer pel */
    MV_CLASS_7 = 7,   /* (128, 256] integer pel */
    MV_CLASS_8 = 8,   /* (256, 512] integer pel */
    MV_CLASS_9 = 9,   /* (512, 1024] integer pel */
    MV_CLASS_10 = 10, /* (1024,2048] integer pel */
} ;

#define CLASS0_BITS 1 /* bits at integer precision for class 0 */
#define CLASS0_SIZE (1 << CLASS0_BITS)
#define MV_OFFSET_BITS (MV_CLASSES + CLASS0_BITS - 2)
#define MV_BITS_CONTEXTS 6
#define MV_FP_SIZE 4

#define MV_MAX_BITS (MV_CLASSES + CLASS0_BITS + 2)
#define MV_MAX ((1 << MV_MAX_BITS) - 1)
#define MV_VALS ((MV_MAX << 1) + 1)

#define MV_IN_USE_BITS 14
#define MV_UPP (1 << MV_IN_USE_BITS)
#define MV_LOW (-(1 << MV_IN_USE_BITS))
/*
typedef struct {
    aom_cdf_prob classes_cdf[CDF_SIZE(MV_CLASSES)];
    aom_cdf_prob class0_fp_cdf[CLASS0_SIZE][CDF_SIZE(MV_FP_SIZE)];
    aom_cdf_prob fp_cdf[CDF_SIZE(MV_FP_SIZE)];
    aom_cdf_prob sign_cdf[CDF_SIZE(2)];
    aom_cdf_prob class0_hp_cdf[CDF_SIZE(2)];
    aom_cdf_prob hp_cdf[CDF_SIZE(2)];
    aom_cdf_prob class0_cdf[CDF_SIZE(CLASS0_SIZE)];
    aom_cdf_prob bits_cdf[MV_OFFSET_BITS][CDF_SIZE(2)];
} nmv_component;

typedef struct {
    aom_cdf_prob joints_cdf[CDF_SIZE(MV_JOINTS)];
    nmv_component comps[2];
} NmvContext;
*/

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_DSP_PROB_H_
