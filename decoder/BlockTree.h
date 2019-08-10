#ifndef BlockTree_h
#define BlockTree_h


const int Num_4x4_Blocks_Wide[BLOCK_SIZES_ALL] = {
    1, 1, 2, 2, 2, 4, 4, 4, 8, 8, 8,
    16, 16, 16, 32, 32, 1, 4, 2, 8, 4, 16
};

const int Num_4x4_Blocks_High[BLOCK_SIZES_ALL] = {
    1, 2, 1, 2, 4, 2, 4, 8, 4, 8, 16,
    8, 16, 32, 16, 32, 4, 1, 8, 2, 16, 4
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

struct BlockTree {
public:
    virtual void parse() = 0;
    virtual bool decode(std::shared_ptr<YuvFrame>&) = 0;
};

#endif //BlockTree_h