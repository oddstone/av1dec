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

#include "BlockTree.h"
#include "Tile.h"

namespace YamiAv1 {

class Partition : public BlockTree {
public:
    Partition(Tile& tile, int r, int c, BLOCK_SIZE sbSize);
    void parse();
    bool decode(std::shared_ptr<Yami::YuvFrame>& frame, const FrameStore& frameStore);

protected:
    void parseBlock(int r, int c, BLOCK_SIZE sbSize);
    void parseSubPartition(int r, int c, BLOCK_SIZE sbSize);
    void getPartitionTypeCtx(uint8_t& ctx, uint8_t& bsl);
    PARTITION_TYPE readPartitionType();
    bool readSplitOrHorz();
    bool readSplitOrVert();
    std::vector<std::shared_ptr<BlockTree>> m_blocks;
    Tile& m_tile;
    FrameHeader& m_frame;
    EntropyDecoder& m_entropy;
    int m_r;
    int m_c;
    BLOCK_SIZE m_bSize;
};
}
