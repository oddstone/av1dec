#include "Block.h"
#include "Av1Common.h"
#include "Av1Parser.h"
#include "Av1Tile.h"
#include "SymbolDecoder.h"
#include "TransformBlock.h"
#include "log.h"

#include <limits>

Block::Block(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE subSize)
    : m_frame(*tile.m_frame)
    , m_sequence(*tile.m_sequence)
    , m_entropy(*tile.m_entropy)
    , m_tile(tile)
    , m_decoded(tile.m_decoded)
    , MiRow(r)
    , MiCol(c)
    , MiSize(subSize)
    , bw4(Num_4x4_Blocks_Wide[subSize])
    , bh4(Num_4x4_Blocks_High[subSize])
    , sbMask(m_sequence.use_128x128_superblock ? 31 : 15)
    , subsampling_x(m_sequence.subsampling_x)
    , subsampling_y(m_sequence.subsampling_y)
    , AngleDeltaY(0)
    , AngleDeltaUV(0)
{
    if (bh4 == 1 && subsampling_y && (MiRow & 1) == 0)
        HasChroma = false;
    else if (bw4 == 1 && subsampling_x && (MiCol & 1) == 0)
        HasChroma = false;
    else
        HasChroma = m_sequence.NumPlanes > 1;
    AvailU = tile.is_inside(r - 1, c);
    AvailL = tile.is_inside(r, c - 1);
    AvailUChroma = AvailU;
    AvailLChroma = AvailL;
    if (HasChroma) {
        if (subsampling_y && bh4 == 1)
            AvailUChroma = tile.is_inside(r - 2, c);
        if (subsampling_x && bw4 == 1)
            AvailLChroma = tile.is_inside(r, c - 2);
    } else {
        AvailUChroma = false;
        AvailLChroma = false;
    }
}

BLOCK_SIZE Subsampled_Size[BLOCK_SIZES_ALL][2][2] = {
    { { BLOCK_4X4, BLOCK_4X4 }, { BLOCK_4X4, BLOCK_4X4 } },
    { { BLOCK_4X8, BLOCK_4X4 }, { BLOCK_INVALID, BLOCK_4X4 } },
    { { BLOCK_8X4, BLOCK_INVALID }, { BLOCK_4X4, BLOCK_4X4 } },
    { { BLOCK_8X8, BLOCK_8X4 }, { BLOCK_4X8, BLOCK_4X4 } },
    { { BLOCK_8X16, BLOCK_8X8 }, { BLOCK_INVALID, BLOCK_4X8 } },
    { { BLOCK_16X8, BLOCK_INVALID }, { BLOCK_8X8, BLOCK_8X4 } },
    { { BLOCK_16X16, BLOCK_16X8 }, { BLOCK_8X16, BLOCK_8X8 } },
    { { BLOCK_16X32, BLOCK_16X16 }, { BLOCK_INVALID, BLOCK_8X16 } },
    { { BLOCK_32X16, BLOCK_INVALID }, { BLOCK_16X16, BLOCK_16X8 } },
    { { BLOCK_32X32, BLOCK_32X16 }, { BLOCK_16X32, BLOCK_16X16 } },
    { { BLOCK_32X64, BLOCK_32X32 }, { BLOCK_INVALID, BLOCK_16X32 } },
    { { BLOCK_64X32, BLOCK_INVALID }, { BLOCK_32X32, BLOCK_32X16 } },
    { { BLOCK_64X64, BLOCK_64X32 }, { BLOCK_32X64, BLOCK_32X32 } },
    { { BLOCK_64X128, BLOCK_64X64 }, { BLOCK_INVALID, BLOCK_32X64 } },
    { { BLOCK_128X64, BLOCK_INVALID }, { BLOCK_64X64, BLOCK_64X32 } },
    { { BLOCK_128X128, BLOCK_128X64 }, { BLOCK_64X128, BLOCK_64X64 } },
    { { BLOCK_4X16, BLOCK_4X8 }, { BLOCK_INVALID, BLOCK_4X8 } },
    { { BLOCK_16X4, BLOCK_INVALID }, { BLOCK_8X4, BLOCK_8X4 } },
    { { BLOCK_8X32, BLOCK_8X16 }, { BLOCK_INVALID, BLOCK_4X16 } },
    { { BLOCK_32X8, BLOCK_INVALID }, { BLOCK_16X8, BLOCK_16X4 } },
    { { BLOCK_16X64, BLOCK_16X32 }, { BLOCK_INVALID, BLOCK_8X32 } },
    { { BLOCK_64X16, BLOCK_INVALID }, { BLOCK_32X16, BLOCK_32X8 } },
};

BLOCK_SIZE Block::get_plane_residual_size(int subsize, int plane)
{
    int subx = plane > 0 ? subsampling_x : 0;
    int suby = plane > 0 ? subsampling_y : 0;
    return Subsampled_Size[subsize][subx][suby];
}

int16_t Block::get_q_idx()
{
    return m_frame.m_segmentation.segmentation_enabled ? m_frame.get_qindex(true, segment_id) : m_frame.m_quant.base_q_idx;
}

void Block::compute_prediction()
{

    uint32_t subBlockMiRow = MiRow & sbMask;
    uint32_t subBlockMiCol = MiCol & sbMask;
    for (int plane = 0; plane < 1 + HasChroma * 2; plane++) {
        int planeSz = get_plane_residual_size(MiSize, plane);
        int num4x4W = Num_4x4_Blocks_Wide[planeSz];
        int num4x4H = Num_4x4_Blocks_High[planeSz];
        int log2W = MI_SIZE_LOG2 + Mi_Width_Log2[planeSz];
        int log2H = MI_SIZE_LOG2 + Mi_Height_Log2[planeSz];
        int subX = (plane > 0) ? subsampling_x : 0;
        int subY = (plane > 0) ? subsampling_y : 0;
        int baseX = (MiCol >> subX) * MI_SIZE;
        int baseY = (MiRow >> subY) * MI_SIZE;
        int candRow = (MiRow >> subY) << subY;
        int candCol = (MiCol >> subX) << subX;
        bool IsInterIntra = (is_inter && RefFrame[1] == INTRA_FRAME);
        if (IsInterIntra) {
            ASSERT(0 && "IsInterIntra");
        }
        if (is_inter) {
            ASSERT(0 && "is_inter");
        }
    }
}

void Block::predict_intra(int plane, int startX, int startY,
    int availL, int availU, bool decodedUpRight, bool decodedBottomLeft,
    int mode, int log2W, int log2H)
{
    if (mode != PAETH_PRED) {
        ASSERT(0 && "not PAETH_PRED");
    }
}

void Block::transform_block(int plane, int baseX, int baseY, TX_SIZE txSz, int x, int y)
{
    int startX = baseX + 4 * x;
    int startY = baseY + 4 * y;
    int subX = (plane > 0) ? subsampling_x : 0;
    int subY = (plane > 0) ? subsampling_y : 0;
    int row = (startY << subY) >> MI_SIZE_LOG2;
    int col = (startX << subX) >> MI_SIZE_LOG2;
    int subBlockMiRow = row & sbMask;
    int subBlockMiCol = col & sbMask;
    int stepX = Tx_Width[txSz] >> MI_SIZE_LOG2;
    int stepY = Tx_Height[txSz] >> MI_SIZE_LOG2;
    int maxX = m_frame.MiCols * MI_SIZE;
    int maxY = m_frame.MiRows * MI_SIZE;
    if (startX >= maxX || startY >= maxY) {
        return;
    }
    if (!skip) {
        std::shared_ptr<TransformBlock> tb(new TransformBlock(*this, plane, startX, startY, txSz));
        tb->parse();
        m_transformBlocks.push_back(tb);
    }
    for (int i = 0; i < stepY; i++) {
        for (int j = 0; j < stepX; j++) {
            /*        LoopfilterTxSizes[ plane ]
        [ (row >> subY) + i ]
        [ (col >> subX) + j ] = txSz*/
            m_decoded.setFlag(plane, (subBlockMiRow >> subY) + i, (subBlockMiCol >> subX) + j);
        }
    }
}

TX_SIZE Block::get_tx_size(int plane, TX_SIZE txSz)
{
    if (plane == 0)
        return txSz;
    TX_SIZE uvTx = Max_Tx_Size_Rect[get_plane_residual_size(MiSize, plane)];
    if (Tx_Width[uvTx] == 64 || Tx_Height[uvTx] == 64) {
        if (Tx_Width[uvTx] == 16) {
            return TX_16X32;
        }
        if (Tx_Height[uvTx] == 16) {
            return TX_32X16;
        }
        return TX_32X32;
    }
    return uvTx;
}

void Block::residual()
{
    int Block_Width = Num_4x4_Blocks_Wide[MiSize] << 2;
    int Block_Height = Num_4x4_Blocks_High[MiSize] << 2;
    int widthChunks = std::max(1, Block_Width >> 6);
    int heightChunks = std::max(1, Block_Height >> 6);
    int miSizeChunk = (widthChunks > 1 || heightChunks > 1) ? BLOCK_64X64 : MiSize;
    for (int chunkY = 0; chunkY < heightChunks; chunkY++) {
        for (int chunkX = 0; chunkX < widthChunks; chunkX++) {
            int miRowChunk = MiRow + (chunkY << 4);
            int miColChunk = MiCol + (chunkX << 4);
            int subBlockMiRow = miRowChunk & sbMask;
            for (int plane = 0; plane < 1 + HasChroma * 2; plane++) {
                TX_SIZE txSz = Lossless ? TX_4X4 : get_tx_size(plane, TxSize);
                int stepX = Tx_Width[txSz] >> 2;
                int stepY = Tx_Height[txSz] >> 2;
                int planeSz = get_plane_residual_size(miSizeChunk, plane);
                int num4x4W = Num_4x4_Blocks_Wide[planeSz];
                int num4x4H = Num_4x4_Blocks_High[planeSz];
                int subX = (plane > 0) ? subsampling_x : 0;
                int subY = (plane > 0) ? subsampling_y : 0;
                int baseX = (miColChunk >> subX) * MI_SIZE;
                int baseY = (miRowChunk >> subY) * MI_SIZE;
                if (is_inter && !Lossless && !plane) {
                    ASSERT(0 && "transform_tree");
                    //transform_tree( baseX, baseY, num4x4W * 4, num4x4H * 4 )
                } else {
                    int baseXBlock = (MiCol >> subX) * MI_SIZE;
                    int baseYBlock = (MiRow >> subY) * MI_SIZE;
                    for (int y = 0; y < num4x4H; y += stepY)
                        for (int x = 0; x < num4x4W; x += stepX)
                            transform_block(plane, baseXBlock, baseYBlock, txSz,
                                x + ((chunkX << 4) >> subX),
                                y + ((chunkY << 4) >> subY));
                }
            }
        }
    }
}

void Block::reset_block_context()
{
    for (int plane = 0; plane < 1 + 2 * HasChroma; plane++) {
        int subX = (plane > 0) ? subsampling_x : 0;
        int subY = (plane > 0) ? subsampling_y : 0;
        m_tile.m_above.reset(plane, MiCol >> subX, bw4);
        m_tile.m_left.reset(plane, MiRow >> subY, bh4);
    }
}

void Block::parse()
{
    mode_info();
    palette_tokens();
    read_block_tx_size();
    if (skip)
        reset_block_context();
    bool isCompund = RefFrame[1] > INTRA_FRAME;

    uint32_t r = MiRow;
    uint32_t c = MiCol;
    for (int y = 0; y < bh4; y++ ) {
        for (int x = 0; x < bw4; x++ ) {
            m_frame.YModes[r + y][c + x] = YMode;
            if (RefFrame[0] == INTRA_FRAME && HasChroma)
                m_frame.UVModes[r + y][c + x] = UVMode;
            for (int refList = 0; refList < 2; refList++ )
                m_frame.RefFrames[r + y][c + x][refList] = RefFrame[refList];
            if ( is_inter ) {
                ASSERT(0);
                /*
                if ( !use_intrabc ) {
                CompGroupIdxs[ r + y ][ c + x ] = comp_group_idx
                CompoundIdxs[ r + y ][ c + x ] = compound_idx
                }
                for ( dir = 0; dir < 2; dir++ ) {
                InterpFilters[ r + y ][ c + x ][ dir ] = interp_filter[ dir ]
                }
                for ( refList = 0; refList < 1 + isCompound; refList++ ) {
                Mvs[ r + y ][ c + x ][ refList ] = Mv[ refList ]
                }*/
            }
        }
    }

    compute_prediction();
    residual();
    for (int y = 0; y < bh4; y++) {
        for (int x = 0; x < bw4; x++) {
            m_frame.IsInters[r + y][c + x] = is_inter;
            //SkipModes[r + y][c + x] = skip_mode
            m_frame.Skips[r + y][c + x] = skip;
            m_frame.TxSizes[r + y][c + x] = TxSize;
            m_frame.MiSizes[r + y][c + x] = MiSize;
            m_frame.SegmentIds[r + y][c + x] = segment_id;
            /*PaletteSizes[0][r + y][c + x] = PaletteSizeY
            PaletteSizes[1][r + y][c + x] = PaletteSizeUV
                for (i = 0; i < PaletteSizeY; i++)
                    PaletteColors[0][r + y][c + x][i] = palette_colors_y[i]
                    for (i = 0; i < PaletteSizeUV; i++)
                        PaletteColors[1][r + y][c + x][i] = palette_colors_u[i]
                        for (i = 0; i < FRAME_LF_COUNT; i++)
                            DeltaLFs[r + y][c + x][i] = DeltaLF[i]
            */
        }
    }
}

uint8_t Block::getSkipCtx()
{
    uint8_t ctx = 0;
    if (AvailU)
        ctx += m_frame.Skips[MiRow - 1][MiCol];
    if (AvailL)
        ctx += m_frame.Skips[MiRow][MiCol - 1];
    return ctx;
}

bool Block::readSkip()
{
    uint8_t ctx = getSkipCtx();
    return m_entropy.readSkip(ctx);
}

void Block::readCdef()
{
    if (skip || m_frame.CodedLossless || !m_sequence.enable_cdef || m_frame.allow_intrabc)
        return;
    m_frame.m_cdef.read_cdef(m_entropy, MiRow, MiCol, MiSize);
}

void Block::read_delta_qindex(bool readDeltas)
{
    BLOCK_SIZE sbSize = m_sequence.use_128x128_superblock ? BLOCK_128X128 : BLOCK_64X64;
    if (MiSize == sbSize && skip)
        return;
    if (readDeltas) {
        ASSERT(0);
    }
}

void Block::read_delta_lf(bool ReadDeltas)
{
    if (!(ReadDeltas && m_frame.m_deltaLf.delta_lf_present))
        return;
    ASSERT(0);
}

PREDICTION_MODE Block::intra_frame_y_mode()
{
    uint8_t Intra_Mode_Context[INTRA_MODES] = {
        0, 1, 2, 3, 4, 4, 4, 4, 3, 0, 1, 2, 0
    };
    PREDICTION_MODE above = AvailU ? m_frame.YModes[MiRow - 1][MiCol] :DC_PRED;
    PREDICTION_MODE left = AvailL ? m_frame.YModes[MiRow][MiCol - 1] : DC_PRED;
    uint8_t aboveCtx = Intra_Mode_Context[above];
    uint8_t leftCtx = Intra_Mode_Context[left];
    return m_entropy.readIntraFrameYMode(aboveCtx, leftCtx);
}

void Block::intra_angle_info_y()
{
    if (MiSize >= BLOCK_8X8) {
        if (is_directional_mode(YMode)) {
            AngleDeltaY = m_entropy.readAngleDeltaY(YMode);
            if (AngleDeltaY < -3) {
                printf("Ok");
            }
        }
    }
}

UV_PREDICTION_MODE Block::uv_mode()
{

    CFL_ALLOWED_TYPE cflAllowed;
    if (Lossless && get_plane_residual_size(MiSize, 1) == BLOCK_4X4) {
        cflAllowed = CFL_ALLOWED;
    } else if (!Lossless && (std::max(bw4, bh4) <= 8)) {
        cflAllowed = CFL_ALLOWED;
    } else {
        cflAllowed = CFL_DISALLOWED;
    }
    return m_entropy.readUvMode(cflAllowed, YMode);
}

void Block::intra_angle_info_uv()
{
    if (MiSize >= BLOCK_8X8) {
        if (is_directional_mode(UVMode)) {
            AngleDeltaUV = m_entropy.readAngleDeltaUV(UVMode);
        }
    }
}

void Block::filter_intra_mode_info()
{
    use_filter_intra = false;
    if (m_sequence.enable_filter_intra && YMode == DC_PRED && PaletteSizeY == 0 && std::max(bw4, bh4) <= 8) {
        use_filter_intra = m_entropy.readUseFilterIntra(MiSize);
        if (use_filter_intra) {
            filter_intra_mode = m_entropy.readFilterIntraMode();
        }
    }
}

void Block::palette_tokens()
{
    if (PaletteSizeY) {
        ASSERT(0);
    }
}

void Block::intra_segment_id()
{
    if (m_frame.m_segmentation.segmentation_enabled)
        ASSERT(0);
    else
        segment_id = 0;
    Lossless = m_frame.LosslessArray[segment_id];
}

void Block::read_cfl_alphas()
{
    uint8_t cfl_alpha_signs = m_entropy.readCflAlphaSigns();
    uint8_t signU = (cfl_alpha_signs + 1) / 3;
    uint8_t signV = (cfl_alpha_signs + 1) % 3;
    if (signU != CFL_SIGN_ZERO) {
        CflAlphaU = m_entropy.readCflAlphaU(cfl_alpha_signs);
        if (signU == CFL_SIGN_NEG)
            CflAlphaU = -CflAlphaU;
    } else {
        CflAlphaU = 0;
    }
    if (signV != CFL_SIGN_ZERO) {
        CflAlphaV = m_entropy.readCflAlphaV(cfl_alpha_signs);
        if (signV == CFL_SIGN_NEG)
            CflAlphaV = -CflAlphaV;
    } else {
        CflAlphaV = 0;
    }
}

void Block::intra_frame_mode_info()
{
    bool SegIdPreSkip = m_frame.m_segmentation.SegIdPreSkip;
    if (SegIdPreSkip)
        intra_segment_id();
    skip = readSkip();
    if (!SegIdPreSkip)
        intra_segment_id();
    readCdef();
    bool ReadDeltas = m_frame.m_deltaQ.delta_q_present;
    read_delta_qindex(ReadDeltas);
    read_delta_lf(ReadDeltas);

    RefFrame[0] = INTRA_FRAME;
    RefFrame[1] = NONE_FRAME;

    bool use_intrabc;
    if (m_frame.allow_intrabc) {
        ASSERT(0 && "intrabc");
    } else {
        use_intrabc = false;
    }
    if (use_intrabc) {
        is_inter = true;
        YMode = DC_PRED;
        ASSERT(0);
    } else {
        is_inter = false;
        YMode = intra_frame_y_mode();
        intra_angle_info_y();
        if (HasChroma) {
            UVMode = uv_mode();
            if (UVMode == UV_CFL_PRED) {
                read_cfl_alphas();
            }
            intra_angle_info_uv();
        }

        PaletteSizeY = 0;
        PaletteSizeUV = 0;
        if (MiSize >= BLOCK_8X8 && bw4 <= 16 && bh4 <= 16 && m_frame.allow_screen_content_tools) {
            ASSERT(0);
            //palette_mode_info( )
        }

        filter_intra_mode_info();
    }
}

void Block::inter_frame_mode_info()
{
    ASSERT(0);
}

void Block::mode_info()
{
    if (m_frame.FrameIsIntra)
        intra_frame_mode_info();
    else
        inter_frame_mode_info();
}

int Block::get_above_tx_width(uint32_t row, uint32_t col)
{
    if ( row == MiRow ) {
        if ( !AvailU ) {
            return 64;
        } else if (m_frame.Skips[row - 1][col] && m_frame.IsInters[row - 1][col]) {
            return Block_Width[m_frame.MiSizes[row - 1][col]];
        }
    }
    return Tx_Width[m_frame.InterTxSizes[row - 1][col]];
}

int Block::get_left_tx_height(uint32_t row, uint32_t col )
{
    if ( col == MiCol ) {
        if ( !AvailL ) {
            return 64;
        } else if (m_frame.Skips[ row ][ col - 1 ] && m_frame.IsInters[ row ][ col - 1 ] ) {
            return Block_Height[m_frame.MiSizes[row][col - 1]];
        }
    }
    return Tx_Height[m_frame.InterTxSizes[row][col - 1]];
}

uint8_t Block::getTxDepthCtx(TX_SIZE maxRectTxSize)
{
    int maxTxWidth = Tx_Width[maxRectTxSize];
    int maxTxHeight = Tx_Height[maxRectTxSize];
    int aboveW;
    if (AvailU && m_frame.IsInters[MiRow - 1][MiCol]) {
        aboveW = Block_Width[m_frame.MiSizes[MiRow - 1][MiCol]];
    } else if (AvailU) {
        aboveW = get_above_tx_width(MiRow, MiCol);
    } else {
        aboveW = 0;
    }
    int leftH;
    if (AvailL && m_frame.IsInters[MiRow][MiCol - 1]) {
        leftH = Block_Height[m_frame.MiSizes[MiRow][MiCol - 1]];
    } else if (AvailL) {
        leftH = get_left_tx_height(MiRow, MiCol);
    } else {
        leftH = 0;
    }
    int ctx = (aboveW >= maxTxWidth) + (leftH >= maxTxHeight);
    return ctx;
}

void Block::read_tx_size(bool allowSelect)
{
    if (Lossless) {
        TxSize = TX_4X4;
        return;
    }
    TX_SIZE maxRectTxSize = Max_Tx_Size_Rect[MiSize];
    int maxTxDepth = Max_Tx_Depth[MiSize];
    TxSize = maxRectTxSize;
    if (MiSize > BLOCK_4X4 && allowSelect && m_frame.TxMode == TX_MODE_SELECT) {
        uint8_t tx_depth = m_entropy.readTxDepth(maxTxDepth, getTxDepthCtx(maxRectTxSize));
        for (uint8_t i = 0; i < tx_depth; i++)
            TxSize = Split_Tx_Size[TxSize];
    }
}

void Block::read_block_tx_size()
{
    if (m_frame.TxMode == TX_MODE_SELECT && MiSize > BLOCK_4X4 && is_inter && !skip && !m_frame.CodedLossless) {
        ASSERT(0);
    } else {
        read_tx_size(!skip || !is_inter);
        for (int row = MiRow; row < MiRow + bh4; row++) {
            for (int col = MiCol; col < MiCol + bw4; col++)
                m_frame.InterTxSizes[row][col] = TxSize;
        }
    }
}

bool Block::decode(std::shared_ptr<YuvFrame>& frame)
{
    for (auto& t : m_transformBlocks) {
        if (!t->decode(frame))
            return false;
    }
    return true;
}
