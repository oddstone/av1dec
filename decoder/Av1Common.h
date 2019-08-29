#ifndef Av1Common_h
#define Av1Common_h
#include "enums.h"

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


#endif