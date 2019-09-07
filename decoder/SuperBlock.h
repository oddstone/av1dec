#ifndef SuperBlock_h
#define SuperBlock_h

#include "Av1Tile.h"
#include "Partition.h"

class SuperBlock : public Partition {
public:
    SuperBlock(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    void parse();
    bool decode(std::shared_ptr<YuvFrame>& frame);
};

#endif

