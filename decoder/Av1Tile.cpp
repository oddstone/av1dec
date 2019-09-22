#include "Av1Tile.h"
#include "Av1Parser.h"
#include "Av1Common.h"
#include "Block.h"
#include "SuperBlock.h"
#include "Partition.h"
#include "SymbolDecoder.h"
#include "VideoFrame.h"
#include "log.h"

BlockDecoded::BlockDecoded()
{
}

void BlockDecoded::init(const Tile& tile)
{
    const SequenceHeader& s = *tile.m_sequence;
    NumPlanes = s.NumPlanes;
    subsampling_x = s.subsampling_x;
    subsampling_y = s.subsampling_y;
    MiColEnd = tile.MiColEnd;
    MiRowEnd = tile.MiRowEnd;
}

void BlockDecoded::clearFlags(int r, int c, int sbSize4)
{
    for (int plane = 0; plane < NumPlanes; plane++) {
        int subX = (plane > 0) ? subsampling_x : 0;
        int subY = (plane > 0) ? subsampling_y : 0;
        int sbWidth4 = (MiColEnd - c) >> subX;
        int sbHeight4 = (MiRowEnd - r) >> subY;
        memset(m_decoded[plane], false, sizeof(m_decoded[plane]));
        for (int y = -1; y <= (sbSize4 >> subY); y++) {
            for (int x = -1; x <= (sbSize4 >> subX); x++) {
                if (y < 0 && x < sbWidth4)
                    setFlag(plane, y, x);
                else if (x < 0 && y < sbHeight4)
                    setFlag(plane, y, x);
                else
                    clearFlag(plane, y, x);
            }
        }
        clearFlag(plane, sbSize4 >> subY, -1);
    }
}

bool BlockDecoded::getFlag(int plane, int r, int c) const
{
    return m_decoded[plane][r + OFFSET][c + OFFSET];
}

void BlockDecoded::setFlag(int plane, int r, int c)
{
    m_decoded[plane][r + OFFSET][c + OFFSET] = true;
}

void BlockDecoded::clearFlag(int plane, int r, int c)
{
    m_decoded[plane][r + OFFSET][c + OFFSET] = false;
}


Tile::Tile(std::shared_ptr<const SequenceHeader> sequence, std::shared_ptr<FrameHeader> frame, uint32_t TileNum)
    : m_sequence(sequence)
    , m_frame(frame)
{
    uint32_t TileRow = TileNum / frame->TileCols;
    uint32_t TileCol = TileNum % frame->TileCols;
    MiRowStart = frame->MiRowStarts[TileRow];
    MiRowEnd = frame->MiRowStarts[TileRow + 1];
    MiColStart = frame->MiColStarts[TileCol];
    MiColEnd = frame->MiColStarts[TileCol + 1];
    CurrentQIndex = frame->m_quant.base_q_idx;
    m_decoded.init(*this);

}

bool Tile::is_inside(uint32_t r, uint32_t c)
{
    return (c >= MiColStart && c < MiColEnd && r >= MiRowStart && r < MiRowEnd);
}

bool Tile::decodeBlock(uint32_t r, uint32_t c, BLOCK_SIZE bSize)
{
    Block b(*this, r, c, bSize);
    b.parse();
    //b->decode()
    //readModeInfo(r, c, bSize);

    return true;
}
/*
bool Tile::decodePartition(uint32_t r, uint32_t c, BLOCK_SIZE bSize)
{
    uint32_t MiRows = m_frame->MiRows;
    uint32_t MiCols = m_frame->MiCols;
    if (r >= MiRows || c >= MiCols)
        return true;
    bool AvailU = is_inside(r - 1, c);
    bool AvailL = is_inside(r, c - 1);
    int num4x4 = Num_4x4_Blocks_Wide[bSize];
    int halfBlock4x4 = num4x4 >> 1;
    int quarterBlock4x4 = halfBlock4x4 >> 1;
    bool hasRows = (r + halfBlock4x4) < MiRows;
    bool hasCols = (c + halfBlock4x4) < MiCols;

    PARTITION_TYPE partition;
    if (bSize < BLOCK_8X8) {
        partition = PARTITION_NONE;
    } else if (hasRows && hasCols) {
        partition = readPartition(r, c, AvailU, AvailL, bSize);
    } else if (hasCols) {
        //split_or_horz S()
        //			partition = split_or_horz ? PARTITION_SPLIT : PARTITION_HORZ
        ASSERT(0);
    } else if (hasRows) {
        //split_or_vert S()
        //partition = split_or_vert ? PARTITION_SPLIT : PARTITION_VERT
        ASSERT(0);
    } else {
        partition = PARTITION_SPLIT;
    }
    BLOCK_SIZE subSize = Partition_Subsize[partition][bSize];
    BLOCK_SIZE splitSize = Partition_Subsize[PARTITION_SPLIT][bSize];

    if (partition == PARTITION_NONE) {
        decodeBlock(r, c, subSize);
    } else if (partition == PARTITION_HORZ) {
        decodeBlock(r, c, subSize);
        if (hasRows)
            decodeBlock(r + halfBlock4x4, c, subSize);
    } else if (partition == PARTITION_VERT) {
        decodeBlock(r, c, subSize);
        if (hasCols)
            decodeBlock(r, c + halfBlock4x4, subSize);
    } else if (partition == PARTITION_SPLIT) {
        decodePartition(r, c, subSize);
        decodePartition(r, c + halfBlock4x4, subSize);
        decodePartition(r + halfBlock4x4, c, subSize);
        decodePartition(r + halfBlock4x4, c + halfBlock4x4, subSize);
    } else if (partition == PARTITION_HORZ_A) {
        decodeBlock(r, c, splitSize);
        decodeBlock(r, c + halfBlock4x4, splitSize);
        decodeBlock(r + halfBlock4x4, c, subSize);
    } else if (partition == PARTITION_HORZ_B) {
        decodeBlock(r, c, subSize);
        decodeBlock(r + halfBlock4x4, c, splitSize);
        decodeBlock(r + halfBlock4x4, c + halfBlock4x4, splitSize);
    } else if (partition == PARTITION_VERT_A) {
        decodeBlock(r, c, splitSize);
        decodeBlock(r + halfBlock4x4, c, splitSize);
        decodeBlock(r, c + halfBlock4x4, subSize);
    } else if (partition == PARTITION_VERT_B) {
        decodeBlock(r, c, subSize);
        decodeBlock(r, c + halfBlock4x4, splitSize);
        decodeBlock(r + halfBlock4x4, c + halfBlock4x4, splitSize);
    } else if (partition == PARTITION_HORZ_4) {
        decodeBlock(r + quarterBlock4x4 * 0, c, subSize);
        decodeBlock(r + quarterBlock4x4 * 1, c, subSize);
        decodeBlock(r + quarterBlock4x4 * 2, c, subSize);
        if (r + quarterBlock4x4 * 3 < MiRows)
            decodeBlock(r + quarterBlock4x4 * 3, c, subSize);
    } else {
        decodeBlock(r, c + quarterBlock4x4 * 0, subSize);
        decodeBlock(r, c + quarterBlock4x4 * 1, subSize);
        decodeBlock(r, c + quarterBlock4x4 * 2, subSize);
        if (c + quarterBlock4x4 * 3 < MiCols)
            decodeBlock(r, c + quarterBlock4x4 * 3, subSize);
    }
    return true;
}
*/
bool Tile::parse(const uint8_t* data, uint32_t size)
{
    m_entropy.reset(new EntropyDecoder(data, size, m_frame->disable_cdf_update, m_frame->m_quant.base_q_idx));
    clear_above_context();
    for (int i = 0; i < FRAME_LF_COUNT; i++ )
	    DeltaLF[i] = 0;
    /*
	for ( i = 0; i < FRAME_LF_COUNT; i++ )
	DeltaLF[ i ] = 0
	for ( plane = 0; plane < NumPlanes; plane++ ) {
	for ( pass = 0; pass < 2; pass++ ) {
	RefSgrXqd[ plane ][ pass ] = Sgrproj_Xqd_Mid[ pass ]
	for ( i = 0; i < WIENER_COEFFS; i++ ) {
	RefLrWiener[ plane ][ pass ][ i ] = Wiener_Taps_Mid[ i ]
	}
	}
	}
	*/

	m_frame->m_loopRestoration.resetRefs(m_sequence->NumPlanes);
    BLOCK_SIZE sbSize = m_sequence->use_128x128_superblock ? BLOCK_128X128 : BLOCK_64X64;
    int sbSize4 = Num_4x4_Blocks_Wide[sbSize];
    for (uint32_t r = MiRowStart; r < MiRowEnd; r += sbSize4) {
        clear_left_context();
        for (uint32_t c = MiColStart; c < MiColEnd; c += sbSize4) {
            //ReadDeltas = delta_q_present
            //clear_cdef(r, c)
            //clear_block_decoded_flags(r, c, sbSize4)
            //read_lr(r, c, sbSize)
            //decode_partition(r, c, sbSize)
            //decodePartition(r, c, sbSize);
            std::shared_ptr<SuperBlock> sb(new SuperBlock(*this, r, c, sbSize));
            sb->parse();
            m_sbs.push_back(sb);
        
        }
    }
    return true;
}

void Tile::clear_above_context()
{
    m_above.clear(m_frame->AlignedMiCols);
}

void Tile::clear_left_context()
{
    m_left.clear(m_frame->AlignedMiRows);
}

bool Tile::decode(std::shared_ptr<YuvFrame>& frame)
{
    for (auto& sb : m_sbs) {
        if (!sb->decode(frame))
            return false;

    }
    return true;
}

const static int kMaxPlanes = 3;
void BlockContext::clear(uint32_t size)
{
    LevelContext.clear();
    DcContext.clear();
    LevelContext.resize(kMaxPlanes);
    DcContext.resize(kMaxPlanes);
    for (int i = 0; i < kMaxPlanes; i++) {
        LevelContext[i].assign(size, 0);
        DcContext[i].assign(size, 0);
    }
    SegPredContext.assign(size, false);
}

void BlockContext::reset(uint32_t plane, uint32_t start, uint32_t count)
{
    std::fill_n(&LevelContext[plane][start], count, 0);
    std::fill_n(&DcContext[plane][start], count, 0);
}

void BlockContext::set(uint32_t plane, uint32_t start, uint32_t count, int16_t culLevel, uint8_t dcCategory)
{
    std::fill_n(&LevelContext[plane][start], count, culLevel);
    std::fill_n(&DcContext[plane][start], count, dcCategory);
}
