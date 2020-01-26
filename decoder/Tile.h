#pragma once
#include "../aom/enums.h"
#include "EntropyDecoder.h"
#include <deque>
#include <memory>
#include <stdint.h>
#include <vector>

namespace Yami {

struct YuvFrame;
}

namespace YamiAv1 {

const static uint8_t Mi_Width_Log2[BLOCK_SIZES_ALL] = {
    0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3,
    4, 4, 4, 5, 5, 0, 2, 1, 3, 2, 4
};

const static uint8_t Mi_Height_Log2[BLOCK_SIZES_ALL] = {
    0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 4,
    3, 4, 5, 4, 5, 2, 0, 3, 1, 4, 2
};

class SymbolDecoder;
class Tile;
class SuperBlock;

class BlockDecoded {
public:
    BlockDecoded();
    void init(const Tile& tile);
    void clear_block_decoded_flags(int r, int c, int sbSize4);
    void setFlag(int plane, int r, int c);
    bool getFlag(int plane, int r, int c) const;

private:
    void clearFlag(int plane, int r, int c);
    static const int SIZE = 128 / 4 + 2;
    static const int OFFSET = 1;
    bool m_decoded[3][SIZE][SIZE];
    bool subsampling_x;
    bool subsampling_y;
    int NumPlanes;
    int MiColEnd;
    int MiRowEnd;
};

struct BlockContext {
    std::vector<std::vector<int16_t>> LevelContext;
    std::vector<std::vector<uint8_t>> DcContext;
    std::vector<bool> SegPredContext;
    void clear(uint32_t size);
    void reset(uint32_t plane, uint32_t start, uint32_t count);
    void set(uint32_t plane, uint32_t start, uint32_t count, int16_t culLevel, uint8_t dcCategory);
};

class Tile {
    friend class Block;
    friend class BlockDecoded;
    friend class TransformBlock;
    friend class Partition;
    friend class SuperBlock;

public:
    Tile(std::shared_ptr<const SequenceHeader> sequence, std::shared_ptr<FrameHeader> frame, uint32_t TileNum);
    bool parse(const uint8_t* data, uint32_t size);
    bool decode(std::shared_ptr<Yami::YuvFrame>& frame, const FrameStore& frameStore);

    void frame_end_update_cdf();

    std::shared_ptr<FrameHeader> m_frame;
    std::shared_ptr<const SequenceHeader> m_sequence;
    std::unique_ptr<EntropyDecoder> m_entropy;

private:
    bool decodeBlock(uint32_t r, uint32_t c, BLOCK_SIZE bSize);
    bool is_inside(uint32_t r, uint32_t c) const;
    void clear_above_context();
    void clear_left_context();

    uint32_t MiRowStart;
    uint32_t MiRowEnd;
    uint32_t MiColStart;
    uint32_t MiColEnd;
    uint32_t CurrentQIndex;

    BlockDecoded m_decoded;

    BlockContext m_above;
    BlockContext m_left;
    std::deque<std::shared_ptr<SuperBlock>> m_sbs;
    int8_t DeltaLF[FRAME_LF_COUNT];
    Cdfs m_cdfs;
    const uint32_t m_tileNum;
};
}
