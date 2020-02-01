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

#include "SuperBlock.h"
#include "Block.h"
#include "Parser.h"
#include "VideoFrame.h"
#include "log.h"

namespace YamiAv1 {

SuperBlock::SuperBlock(Tile& tile, int r, int c, BLOCK_SIZE sbSize)
    : Partition(tile, r, c, sbSize)
{
}

void SuperBlock::parse()
{
    m_tile.m_frame->m_loopRestoration.read_lr(m_tile, m_r, m_c, m_bSize);
    Partition::parse();
}

bool SuperBlock::decode(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore)
{
    int sbSize4 = Num_4x4_Blocks_Wide[m_bSize];
    m_tile.m_decoded.clear_block_decoded_flags(m_r, m_c, sbSize4);
    return Partition::decode(frame, frameStore);
}
}
