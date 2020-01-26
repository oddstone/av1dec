#include "SuperBlock.h"
#include "Block.h"
#include "Parser.h"
#include "VideoFrame.h"
#include "log.h"

namespace YamiAv1 {

SuperBlock::SuperBlock(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE sbSize)
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
