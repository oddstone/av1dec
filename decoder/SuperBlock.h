#pragma once

#include "Tile.h"
#include "Partition.h"

namespace YamiAv1 {

class SuperBlock : public Partition {
public:
    SuperBlock(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    void parse();
    bool decode(std::shared_ptr<Yami::YuvFrame>& frame, const FrameStore& frameStore);
};
}
