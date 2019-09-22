#include "SuperBlock.h"
#include "Av1Parser.h"
#include "Block.h"
#include "VideoFrame.h"
#include "log.h"


SuperBlock::SuperBlock(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE sbSize)
    : Partition(tile, r, c, sbSize)
{
}

void SuperBlock::parse()
{
    m_tile.m_frame->m_loopRestoration.read_lr(m_tile, m_r, m_c, m_bSize);
    Partition::parse();
}

bool SuperBlock::decode(std::shared_ptr<YuvFrame>& frame)
{
    int sbSize4 = Num_4x4_Blocks_Wide[m_bSize];
    m_tile.m_decoded.clearFlags(m_r, m_c, sbSize4);
    return Partition::decode(frame);
}

#if 0
void SuperBlock::read_lr()
{
    if (m_frame.allow_intrabc) {
        return;
    }
    int w = Num_4x4_Blocks_Wide[m_bSize];
    int h = Num_4x4_Blocks_High[m_bSize];
    RestorationType* FrameRestorationType = m_frame.m_loopRestoration.FrameRestorationType;
    int* LoopRestorationSize =  m_frame.m_loopRestoration.LoopRestorationSize;
    for (int plane = 0; plane < m_sequence.NumPlanes; plane++ ) {
        if ( FrameRestorationType[ plane ] != RESTORE_NONE ) {
            int subX = (plane == 0) ? 0 : m_sequence.subsampling_x;
            int subY = (plane == 0) ? 0 : m_sequence.subsampling_y;
            int unitSize = LoopRestorationSize[ plane ];
            int unitRows = count_units_in_frame( unitSize, Round2(m_frame.FrameHeight, subY) );
            int unitCols = count_units_in_frame( unitSize, Round2(m_frame.UpscaledWidth, subX) );
            int unitRowStart = ( m_r * ( MI_SIZE >> subY) + unitSize - 1 ) / unitSize;
            int unitRowEnd = std::min( unitRows, ( (m_r + h) * ( MI_SIZE >> subY) + unitSize - 1 ) / unitSize);
            int numerator;
            int denominator;
            if (m_frame.use_superres ) {
                numerator = (MI_SIZE >> subX) * m_frame.SuperresDenom;
                denominator = unitSize * FrameHeader::SUPERRES_NUM;
            } else {
                numerator = MI_SIZE >> subX;
                denominator = unitSize;
            }
            int unitColStart = ( m_c * numerator + denominator - 1 ) / denominator;
            int unitColEnd = std::min( unitCols, ( (m_c + w) * numerator + denominator - 1 ) / denominator);
            for (int unitRow = unitRowStart; unitRow < unitRowEnd; unitRow++ ) {
                for (int unitCol = unitColStart; unitCol < unitColEnd; unitCol++ ) {
                    read_lr_unit(plane, unitRow, unitCol);
                }
            }
        }
    }
}
#endif

