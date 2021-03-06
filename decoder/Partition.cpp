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

#include "Partition.h"
#include "Av1Common.h"
#include "Block.h"
#include "Parser.h"
#include "VideoFrame.h"
#include "log.h"
namespace YamiAv1 {
BLOCK_SIZE Partition_Subsize[10][BLOCK_SIZES_ALL] = {
    { BLOCK_4X4,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X128,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X4,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X128,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X4,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X4,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X4,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X128,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X128,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X4,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X8,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID },
    { BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X16,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X32,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X64,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
        BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID }
};

Partition::Partition(Tile& tile, int r, int c, BLOCK_SIZE bSize)
    : m_tile(tile)
    , m_frame(*tile.m_frame)
    , m_entropy(*tile.m_entropy)
    , m_r(r)
    , m_c(c)
    , m_bSize(bSize)
{
}

void Partition::parse()
{
    int r = m_r;
    int c = m_c;
    BLOCK_SIZE bSize = m_bSize;
    int MiRows = m_frame.MiRows;
    int MiCols = m_frame.MiCols;
    if (r >= MiRows || c >= MiCols)
        return;

    int num4x4 = Num_4x4_Blocks_Wide[bSize];
    int halfBlock4x4 = num4x4 >> 1;
    int quarterBlock4x4 = halfBlock4x4 >> 1;
    bool hasRows = (r + halfBlock4x4) < MiRows;
    bool hasCols = (c + halfBlock4x4) < MiCols;

    PARTITION_TYPE partition;
    if (bSize < BLOCK_8X8) {
        partition = PARTITION_NONE;
    } else if (hasRows && hasCols) {
        partition = readPartitionType();
    } else if (hasCols) {
        bool split_or_horz = readSplitOrHorz();
        partition = split_or_horz ? PARTITION_SPLIT : PARTITION_HORZ;
    } else if (hasRows) {
        bool split_or_vert = readSplitOrVert();
        partition = split_or_vert ? PARTITION_SPLIT : PARTITION_VERT;
    } else {
        partition = PARTITION_SPLIT;
    }
    BLOCK_SIZE subSize = Partition_Subsize[partition][bSize];
    BLOCK_SIZE splitSize = Partition_Subsize[PARTITION_SPLIT][bSize];

    if (partition == PARTITION_NONE) {
        parseBlock(r, c, subSize);
    } else if (partition == PARTITION_HORZ) {
        parseBlock(r, c, subSize);
        if (hasRows)
            parseBlock(r + halfBlock4x4, c, subSize);
    } else if (partition == PARTITION_VERT) {
        parseBlock(r, c, subSize);
        if (hasCols)
            parseBlock(r, c + halfBlock4x4, subSize);
    } else if (partition == PARTITION_SPLIT) {
        parseSubPartition(r, c, subSize);
        parseSubPartition(r, c + halfBlock4x4, subSize);
        parseSubPartition(r + halfBlock4x4, c, subSize);
        parseSubPartition(r + halfBlock4x4, c + halfBlock4x4, subSize);
    } else if (partition == PARTITION_HORZ_A) {
        parseBlock(r, c, splitSize);
        parseBlock(r, c + halfBlock4x4, splitSize);
        parseBlock(r + halfBlock4x4, c, subSize);
    } else if (partition == PARTITION_HORZ_B) {
        parseBlock(r, c, subSize);
        parseBlock(r + halfBlock4x4, c, splitSize);
        parseBlock(r + halfBlock4x4, c + halfBlock4x4, splitSize);
    } else if (partition == PARTITION_VERT_A) {
        parseBlock(r, c, splitSize);
        parseBlock(r + halfBlock4x4, c, splitSize);
        parseBlock(r, c + halfBlock4x4, subSize);
    } else if (partition == PARTITION_VERT_B) {
        parseBlock(r, c, subSize);
        parseBlock(r, c + halfBlock4x4, splitSize);
        parseBlock(r + halfBlock4x4, c + halfBlock4x4, splitSize);
    } else if (partition == PARTITION_HORZ_4) {
        parseBlock(r + quarterBlock4x4 * 0, c, subSize);
        parseBlock(r + quarterBlock4x4 * 1, c, subSize);
        parseBlock(r + quarterBlock4x4 * 2, c, subSize);
        if (r + quarterBlock4x4 * 3 < MiRows)
            parseBlock(r + quarterBlock4x4 * 3, c, subSize);
    } else {
        parseBlock(r, c + quarterBlock4x4 * 0, subSize);
        parseBlock(r, c + quarterBlock4x4 * 1, subSize);
        parseBlock(r, c + quarterBlock4x4 * 2, subSize);
        if (c + quarterBlock4x4 * 3 < MiCols)
            parseBlock(r, c + quarterBlock4x4 * 3, subSize);
    }
    return;
}

bool Partition::decode(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore)
{
    for (auto& b : m_blocks) {
        if (!b->decode(frame, frameStore))
            return false;
    }
    return true;
}

void Partition::parseSubPartition(int r, int c, BLOCK_SIZE bSize)
{
    std::shared_ptr<BlockTree> partition(new Partition(m_tile, r, c, bSize));
    partition->parse();
    m_blocks.push_back(partition);
}

void Partition::parseBlock(int r, int c, BLOCK_SIZE bSize)
{
    std::shared_ptr<BlockTree> block(new Block(m_tile, r, c, bSize));
    block->parse();
    m_blocks.push_back(block);
}

void Partition::getPartitionTypeCtx(uint8_t& ctx, uint8_t& bsl)
{
    bool AvailU = m_tile.is_inside(m_r - 1, m_c);
    bool AvailL = m_tile.is_inside(m_r, m_c - 1);
    bsl = Mi_Width_Log2[m_bSize];
    uint8_t above = AvailU && (Mi_Width_Log2[m_frame.getModeInfo(m_r - 1, m_c).MiSize] < bsl);
    uint8_t left = AvailL && (Mi_Height_Log2[m_frame.getModeInfo(m_r, m_c - 1).MiSize] < bsl);
    ctx = left * 2 + above;
}

PARTITION_TYPE Partition::readPartitionType()
{
    uint8_t ctx, bsl;
    getPartitionTypeCtx(ctx, bsl);
    return m_entropy.readPartition(ctx, bsl);
}

bool Partition::readSplitOrHorz()
{
    uint8_t ctx, bsl;
    getPartitionTypeCtx(ctx, bsl);
    return m_entropy.readSplitOrHorz(ctx, bsl, m_bSize);
}

bool Partition::readSplitOrVert()
{
    uint8_t ctx, bsl;
    getPartitionTypeCtx(ctx, bsl);
    return m_entropy.readSplitOrVert(ctx, bsl, m_bSize);
}
}