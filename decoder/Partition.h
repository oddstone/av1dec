#pragma once

#include "Tile.h"
#include "BlockTree.h"

namespace YamiAv1 {

class Partition : public BlockTree {
public:
    Partition(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    void parse();
    bool decode(std::shared_ptr<Yami::YuvFrame>& frame, const FrameStore& frameStore);

protected:
    void parseBlock(uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    void parseSubPartition(uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    void getPartitionTypeCtx(uint8_t& ctx, uint8_t& bsl);
    PARTITION_TYPE readPartitionType();
    bool readSplitOrHorz();
    bool readSplitOrVert();
    std::vector<std::shared_ptr<BlockTree>> m_blocks;
    Tile& m_tile;
    FrameHeader& m_frame;
    EntropyDecoder& m_entropy;
    uint32_t m_r;
    uint32_t m_c;
    BLOCK_SIZE m_bSize;
};
}
