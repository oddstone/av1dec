#ifndef Av1Common_h
#define Av1Common_h
#include "enums.h"
#include <memory>

namespace Yami {
    namespace Av1 {
        class Parser;
        struct FrameHeader;
        struct SequenceHeader;

        typedef std::shared_ptr<SequenceHeader> SequencePtr;
        typedef std::shared_ptr<const SequenceHeader> ConstSequencePtr;
        typedef std::shared_ptr<FrameHeader> FramePtr;
        typedef std::shared_ptr<const FrameHeader> ConstFramePtr;
    }
}

const int Num_4x4_Blocks_Wide[BLOCK_SIZES_ALL] = {
    1, 1, 2, 2, 2, 4, 4, 4, 8, 8, 8,
    16, 16, 16, 32, 32, 1, 4, 2, 8, 4, 16
};

const int Num_4x4_Blocks_High[BLOCK_SIZES_ALL] = {
    1, 2, 1, 2, 4, 2, 4, 8, 4, 8, 16,
    8, 16, 32, 16, 32, 4, 1, 8, 2, 16, 4
};

const int Block_Width[BLOCK_SIZES_ALL] = {
    4,  4,  8,  8,   8,   16, 16, 16, 32, 32, 32,
    64, 64, 64, 128, 128, 4,  16, 8,  32, 16, 64
};

const int Block_Height[BLOCK_SIZES_ALL] = {
    4,  8,  4,   8,  16,  8,  16, 32, 16, 32, 64,
    32, 64, 128, 64, 128, 16, 4,  32, 8,  64, 16
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

static int FloorLog2(int x) {
    int s = 0;
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

static const int Sgr_Params[ (1 << SGRPROJ_PARAMS_BITS) ][ 4 ] = {
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
  ROTZOOM,
  TRANSLATION,
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

#endif
