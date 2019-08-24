#ifndef Partition_h
#define Partition_h

#include "Av1Tile.h"
#include "BlockTree.h"

class Partition : public BlockTree {
public:
    Partition(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    void parse();
    bool decode(std::shared_ptr<YuvFrame>& frame);
protected:
    void parseBlock(uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    void parseSubPartition(uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    PARTITION_TYPE readPartitionType();
    std::vector<std::shared_ptr<BlockTree>> m_blocks;
    Tile& m_tile;
    FrameHeader& m_frame;
    EntropyDecoder& m_entropy;
    uint32_t m_r;
    uint32_t m_c;
    BLOCK_SIZE m_bSize;

};

#endif

