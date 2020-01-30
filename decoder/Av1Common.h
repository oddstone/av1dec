/*
 * Copyright 2020, av1dec authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include "enums.h"
#include <memory>
#include <vector>

namespace Yami {

class BitReader;
struct YuvFrame;
}

namespace YamiAv1 {

class Parser;
struct FrameHeader;
struct SequenceHeader;

typedef std::shared_ptr<SequenceHeader> SequencePtr;
typedef std::shared_ptr<const SequenceHeader> ConstSequencePtr;
typedef std::shared_ptr<FrameHeader> FramePtr;
typedef std::shared_ptr<const FrameHeader> ConstFramePtr;
typedef std::vector<std::shared_ptr<Yami::YuvFrame>> FrameStore;

const int Num_4x4_Blocks_Wide[BLOCK_SIZES_ALL] = {
    1, 1, 2, 2, 2, 4, 4, 4, 8, 8, 8,
    16, 16, 16, 32, 32, 1, 4, 2, 8, 4, 16
};

const int Num_4x4_Blocks_High[BLOCK_SIZES_ALL] = {
    1, 2, 1, 2, 4, 2, 4, 8, 4, 8, 16,
    8, 16, 32, 16, 32, 4, 1, 8, 2, 16, 4
};

const int Block_Width[BLOCK_SIZES_ALL] = {
    4, 4, 8, 8, 8, 16, 16, 16, 32, 32, 32,
    64, 64, 64, 128, 128, 4, 16, 8, 32, 16, 64
};

const int Block_Height[BLOCK_SIZES_ALL] = {
    4, 8, 4, 8, 16, 8, 16, 32, 16, 32, 64,
    32, 64, 128, 64, 128, 16, 4, 32, 8, 64, 16
};

enum TxSet {
    TX_SET_DCTONLY = 0,
    TX_SET_INTRA_1,
    TX_SET_INTRA_2,
    TX_SET_INTER_1 = 1,
    TX_SET_INTER_2,
    TX_SET_INTER_3,
};

const static TX_SIZE Tx_Size_Sqr[TX_SIZES_ALL] = {
    TX_4X4,
    TX_8X8,
    TX_16X16,
    TX_32X32,
    TX_64X64,
    TX_4X4,
    TX_4X4,
    TX_8X8,
    TX_8X8,
    TX_16X16,
    TX_16X16,
    TX_32X32,
    TX_32X32,
    TX_4X4,
    TX_4X4,
    TX_8X8,
    TX_8X8,
    TX_16X16,
    TX_16X16
};

const static TX_SIZE Tx_Size_Sqr_Up[TX_SIZES_ALL] = {
    TX_4X4,
    TX_8X8,
    TX_16X16,
    TX_32X32,
    TX_64X64,
    TX_8X8,
    TX_8X8,
    TX_16X16,
    TX_16X16,
    TX_32X32,
    TX_32X32,
    TX_64X64,
    TX_64X64,
    TX_16X16,
    TX_16X16,
    TX_32X32,
    TX_32X32,
    TX_64X64,
    TX_64X64
};

const static int Tx_Width[TX_SIZES_ALL] = {
    4, 8, 16, 32, 64, 4, 8, 8, 16, 16, 32, 32, 64, 4, 16, 8, 32, 16, 64
};

const static int Tx_Height[TX_SIZES_ALL] = {
    4, 8, 16, 32, 64, 8, 4, 16, 8, 32, 16, 64, 32, 16, 4, 32, 8, 64, 16
};

const static TX_SIZE Max_Tx_Size_Rect[BLOCK_SIZES_ALL] = {
    TX_4X4, TX_4X8, TX_8X4, TX_8X8,
    TX_8X16, TX_16X8, TX_16X16, TX_16X32,
    TX_32X16, TX_32X32, TX_32X64, TX_64X32,
    TX_64X64, TX_64X64, TX_64X64, TX_64X64,
    TX_4X16, TX_16X4, TX_8X32, TX_32X8,
    TX_16X64, TX_64X16
};

const static int Max_Tx_Depth[BLOCK_SIZES_ALL] = {
    0, 1, 1, 1,
    2, 2, 2, 3,
    3, 3, 4, 4,
    4, 4, 4, 4,
    2, 2, 3, 3,
    4, 4
};

const static TX_SIZE Split_Tx_Size[TX_SIZES_ALL] = {
    TX_4X4,
    TX_4X4,
    TX_8X8,
    TX_16X16,
    TX_32X32,
    TX_4X4,
    TX_4X4,
    TX_8X8,
    TX_8X8,
    TX_16X16,
    TX_16X16,
    TX_32X32,
    TX_32X32,
    TX_4X8,
    TX_8X4,
    TX_8X16,
    TX_16X8,
    TX_16X32,
    TX_32X16
};

static const int Tx_Width_Log2[TX_SIZES_ALL] = {
    2, 3, 4, 5, 6, 2, 3, 3, 4, 4, 5, 5, 6, 2, 4, 3, 5, 4, 6
};

static const int Tx_Height_Log2[TX_SIZES_ALL] = {
    2, 3, 4, 5, 6, 3, 2, 4, 3, 5, 4, 6, 5, 4, 2, 5, 3, 6, 4
};

#define ROUND2(x, n) (((n) == 0) ? (x) : (((x) + (1 << (n - 1))) >> (n)))
#define ROUND2SIGNED(x, n) ((x) >= 0 ? ROUND2((x), (n)) : -ROUND2(-(x), (n)))

#define ROUND2_64(x, n) (((n) == 0) ? (x) : (((x) + ((int64_t)1 << (n - 1))) >> (n)))
#define ROUND2SIGNED_64(x, n) ((x) >= 0 ? ROUND2_64((x), (n)) : -ROUND2_64(-(x), (n)))

template <class T>
T FloorLog2(T x)
{
    T s = 0;
    while (x != 0) {
        x = x >> 1;
        s++;
    }
    return s - 1;
}

#define CLIP1(x) CLIP3(0, ((1 << m_sequence.BitDepth) - 1), x)

#define SGRPROJ_PARAMS_BITS 4
#define SGRPROJ_PRJ_SUBEXP_K 4
#define SGRPROJ_PRJ_BITS 7
#define SGRPROJ_RST_BITS 4
#define SGRPROJ_MTABLE_BITS 20
#define SGRPROJ_RECIP_BITS 12
#define SGRPROJ_SGR_BITS 8

static const int Sgr_Params[(1 << SGRPROJ_PARAMS_BITS)][4] = {
    { 2, 12, 1, 4 }, { 2, 15, 1, 6 }, { 2, 18, 1, 8 }, { 2, 21, 1, 9 },
    { 2, 24, 1, 10 }, { 2, 29, 1, 11 }, { 2, 36, 1, 12 }, { 2, 45, 1, 13 },
    { 2, 56, 1, 14 }, { 2, 68, 1, 15 }, { 0, 0, 1, 5 }, { 0, 0, 1, 8 },
    { 0, 0, 1, 11 }, { 0, 0, 1, 14 }, { 2, 30, 0, 0 }, { 2, 75, 0, 0 }
};

enum SEG_LVL_FEATURE {
    SEG_LVL_ALT_Q,
    SEG_LVL_ALT_LF_Y_V,
    SEG_LVL_REF_FRAME = 5,
    SEG_LVL_SKIP,
    SEG_LVL_GLOBALMV,
    SEG_LVL_MAX,
};

enum CompMode {
    SINGLE_REFERENCE,
    COMPOUND_REFERENCE,
};

#define WARPEDMODEL_PREC_BITS 16

enum GlobalMotionType {
    IDENTITY,
    TRANSLATION,
    ROTZOOM,
    AFFINE,
};

struct Mv {
    int16_t mv[2];
    bool operator==(const Mv& rhs)
    {
        return rhs.mv[0] == mv[0]
            && rhs.mv[1] == mv[1];
    }
};

#define MV_BORDER 128

#define MV_INTRABC_CONTEXT 1

#define SUBPEL_MASK 15

using Yami::BitReader;
using Yami::YuvFrame;
}
