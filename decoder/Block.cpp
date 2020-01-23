#include "Block.h"
#include "Av1Common.h"
#include "InterPredict.h"
#include "IntraPredict.h"
#include "Parser.h"
#include "SymbolDecoder.h"
#include "Tile.h"
#include "TransformBlock.h"
#include "VideoFrame.h"
#include "log.h"

#include <limits>

namespace YamiAv1 {

using namespace Yami;

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
    , bw(bw4 << 2)
    , bh(bh4 << 2)
    , sbMask(m_sequence.use_128x128_superblock ? 31 : 15)
    , subsampling_x(m_sequence.subsampling_x)
    , subsampling_y(m_sequence.subsampling_y)
    , AngleDeltaY(0)
    , AngleDeltaUV(0)
    , m_localWarp(*this)
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

BLOCK_SIZE Block::get_plane_residual_size(int subsize, int plane)
{
    return m_sequence.get_plane_residual_size(subsize, plane);
}

int16_t Block::get_q_idx()
{
    return m_frame.m_segmentation.segmentation_enabled ? m_frame.get_qindex(true, segment_id) : m_frame.m_quant.base_q_idx;
}

void Block::compute_prediction(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore)
{

    uint32_t subBlockMiRow = MiRow & sbMask;
    uint32_t subBlockMiCol = MiCol & sbMask;
    std::vector<std::vector<uint8_t>> mask;
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
            PREDICTION_MODE mode;
            if (interintra_mode == II_DC_PRED)
                mode = DC_PRED;
            else if (interintra_mode == II_V_PRED)
                mode = V_PRED;
            else if (interintra_mode == II_H_PRED)
                mode = H_PRED;
            else
                mode = SMOOTH_PRED;

            std::vector<std::vector<uint8_t>> pred;
            IntraPredict predict(*this, frame, plane, baseX, baseY, log2W, log2H, pred);
            bool haveAboveRight = m_decoded.getFlag(plane, (subBlockMiRow >> subY) - 1, (subBlockMiCol >> subX) + num4x4W);
            bool haveBelowLeft = m_decoded.getFlag(plane, (subBlockMiRow >> subY) + num4x4H, (subBlockMiCol >> subX) - 1);
            predict.predict_intra(
                plane == 0 ? AvailL : AvailLChroma,
                plane == 0 ? AvailU : AvailUChroma,
                haveAboveRight, haveBelowLeft, mode);
            for (size_t row = 0; row < pred.size(); row++) {
                auto& v = pred[row];
                for (size_t col = 0; col < v.size(); col++) {
                    frame->setPixel(plane, baseX + col, baseY + row, v[col]);
                }
            }
        }
        if (is_inter) {
            uint32_t predW = bw >> subX;
            uint32_t predH = bh >> subY;
            bool someUseIntra = false;
            for (uint32_t r = 0; r < (num4x4H << subY); r++) {
                for (uint32_t c = 0; c < (num4x4W << subX); c++) {
                    if (m_frame.RefFrames[candRow + r][candCol + c][0] == INTRA_FRAME)
                        someUseIntra = true;
                }
            }
            if (someUseIntra) {
                predH = num4x4H * 4;
                predW = num4x4W * 4;
                candRow = MiRow;
                candCol = MiCol;
            }
            uint32_t r = 0;
            for (uint32_t y = 0; y < num4x4H * 4; y += predH) {
                uint32_t c = 0;
                for (uint32_t x = 0; x < num4x4W * 4; x += predW) {
                    InterPredict inter(*this, plane, *frame, frameStore, mask);
                    inter.predict_inter(baseX + x, baseY + y, predW, predH, candRow + r, candCol + c);
                    c++;
                }
                r++;
            }
        }
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
    std::shared_ptr<TransformBlock> tb(new TransformBlock(*this, plane, startX, startY, txSz, skip));
    tb->parse();
    m_transformBlocks.push_back(tb);
    for (int i = 0; i < stepY; i++) {
        for (int j = 0; j < stepX; j++) {
            /*        LoopfilterTxSizes[ plane ]
            [ (row >> subY) + i ]
            [ (col >> subX) + j ] = txSz*/
            //m_decoded.setFlag(plane, (subBlockMiRow >> subY) + i, (subBlockMiCol >> subX) + j);
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
static TX_SIZE find_tx_size(int w, int h)
{
    int txSz;
    for (txSz = 0; txSz < TX_SIZES_ALL; txSz++)
        if (Tx_Width[txSz] == w && Tx_Height[txSz] == h)
            break;
    return (TX_SIZE)txSz;
}

void Block::transform_tree(int startX, int startY, int w, int h)
{
    uint32_t maxX = m_frame.MiCols * MI_SIZE;
    uint32_t maxY = m_frame.MiRows * MI_SIZE;

    if (startX >= maxX || startY >= maxY) {
        return;
    }
    int row = startY >> MI_SIZE_LOG2;
    int col = startX >> MI_SIZE_LOG2;
    int lumaTxSz = m_frame.InterTxSizes[row][col];
    int lumaW = Tx_Width[lumaTxSz];
    int lumaH = Tx_Height[lumaTxSz];
    if (w <= lumaW && h <= lumaH) {
        TX_SIZE txSz = find_tx_size(w, h);
        transform_block(0, startX, startY, txSz, 0, 0);
    } else {
        if (w > h) {
            transform_tree(startX, startY, w / 2, h);
            transform_tree(startX + w / 2, startY, w / 2, h);
        } else if (w < h) {
            transform_tree(startX, startY, w, h / 2);
            transform_tree(startX, startY + h / 2, w, h / 2);
        } else {
            transform_tree(startX, startY, w / 2, h / 2);
            transform_tree(startX + w / 2, startY, w / 2, h / 2);
            transform_tree(startX, startY + h / 2, w / 2, h / 2);
            transform_tree(startX + w / 2, startY + h / 2, w / 2, h / 2);
        }
    }
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
                    transform_tree(baseX, baseY, num4x4W * 4, num4x4H * 4);
                } else {
                    int baseXBlock = (MiCol >> subX) * MI_SIZE;
                    int baseYBlock = (MiRow >> subY) * MI_SIZE;
                    for (int y = 0; y < num4x4H; y += stepY) {
                        for (int x = 0; x < num4x4W; x += stepX) {
                            transform_block(plane, baseXBlock, baseYBlock, txSz,
                                x + ((chunkX << 4) >> subX),
                                y + ((chunkY << 4) >> subY));
                        }
                    }
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
        m_tile.m_above.reset(plane, MiCol >> subX, bw4>>subX);
        m_tile.m_left.reset(plane, MiRow >> subY, bh4>>subY);
    }
}

void Block::parse()
{
    mode_info();
    palette_tokens();
    read_block_tx_size();
    if (skip)
        reset_block_context();
    bool isCompound = RefFrame[1] > INTRA_FRAME;

    uint32_t r = MiRow;
    uint32_t c = MiCol;
    for (int y = 0; y < bh4; y++) {
        for (int x = 0; x < bw4; x++) {
            m_frame.YModes[r + y][c + x] = YMode;
            if (RefFrame[0] == INTRA_FRAME && HasChroma)
                m_frame.UVModes[r + y][c + x] = UVMode;
            for (int refList = 0; refList < 2; refList++)
                m_frame.RefFrames[r + y][c + x][refList] = RefFrame[refList];
            if (is_inter) {
                if (!use_intrabc) {
                    m_frame.CompGroupIdxs[r + y][c + x] = comp_group_idx;
                    m_frame.CompoundIdxs[r + y][c + x] = compound_idx;
                }
                for (int dir = 0; dir < 2; dir++) {
                    m_frame.InterpFilters[r + y][c + x][dir] = interp_filter[dir];
                }
                for (int refList = 0; refList < 1 + isCompound; refList++) {
                    m_frame.Mvs[r + y][c + x][refList] = m_mv[refList];
                }
            }
        }
    }
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

                */
            for (int i = 0; i < FRAME_LF_COUNT; i++) {
                m_frame.DeltaLFs[i][r + y][c + x] = m_tile.DeltaLF[i];
            }
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

bool Block::read_skip()
{
    uint8_t ctx = getSkipCtx();
    return m_entropy.readSkip(ctx);
}

void Block::read_cdef()
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
    PREDICTION_MODE above = AvailU ? m_frame.YModes[MiRow - 1][MiCol] : DC_PRED;
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
    skip = read_skip();
    if (!SegIdPreSkip)
        intra_segment_id();
    read_cdef();
    bool ReadDeltas = m_frame.m_deltaQ.delta_q_present;
    read_delta_qindex(ReadDeltas);
    read_delta_lf(ReadDeltas);

    RefFrame[0] = INTRA_FRAME;
    RefFrame[1] = NONE_FRAME;

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

void Block::inter_segment_id(bool preSkip)
{
    uint8_t predictedSegmentId;
    if (m_frame.m_segmentation.segmentation_enabled) {
        ASSERT(0);
    } else {
        segment_id = 0;
    }
}

bool Block::seg_feature_active(SEG_LVL_FEATURE feature)
{
    return m_frame.m_segmentation.seg_feature_active_idx(segment_id, feature);
}

int16_t Block::getSegFeature(SEG_LVL_FEATURE feature)
{
    return m_frame.m_segmentation.FeatureData[segment_id][feature];
}

bool Block::read_skip_mode()
{
    const Segmentation& seg = m_frame.m_segmentation;
    if (seg_feature_active(SEG_LVL_SKIP)
        || seg_feature_active(SEG_LVL_REF_FRAME)
        || seg_feature_active(SEG_LVL_GLOBALMV)
        || !m_frame.skip_mode_present
        || Block_Width[MiSize] < 8
        || Block_Height[MiSize] < 8) {
        skip_mode = false;
    } else {
        ASSERT(0);
    }
    return skip_mode;
}

uint8_t Block::getIsInterCtx()
{
    uint8_t ctx;
    if (AvailU && AvailL)
        ctx = (LeftIntra && AboveIntra) ? 3 : LeftIntra || AboveIntra;
    else if (AvailU || AvailL)
        ctx = 2 * (AvailU ? AboveIntra : LeftIntra);
    else
        ctx = 0;
    return ctx;
}

void Block::read_is_inter()
{
    const Segmentation& seg = m_frame.m_segmentation;
    if (skip_mode) {
        is_inter = true;
    } else if (seg_feature_active(SEG_LVL_REF_FRAME)) {
        is_inter = seg_feature_active(SEG_LVL_REF_FRAME) != INTRA_FRAME;
    } else if (seg_feature_active(SEG_LVL_GLOBALMV)) {
        is_inter = true;
    } else {
        is_inter = m_entropy.readIsInter(getIsInterCtx());
    }
}

static uint8_t check_backward(uint8_t refFrame)
{
    return ((refFrame >= BWDREF_FRAME) && (refFrame <= ALTREF_FRAME));
}
uint8_t Block::getCompModeCtx()
{
    uint8_t ctx;
    if (AvailU && AvailL) {
        if (AboveSingle && LeftSingle)
            ctx = check_backward(AboveRefFrame[0])
                ^ check_backward(LeftRefFrame[0]);
        else if (AboveSingle)
            ctx = 2 + (check_backward(AboveRefFrame[0]) || AboveIntra);
        else if (LeftSingle)
            ctx = 2 + (check_backward(LeftRefFrame[0]) || LeftIntra);
        else
            ctx = 4;
    } else if (AvailU) {
        if (AboveSingle)
            ctx = check_backward(AboveRefFrame[0]);
        else
            ctx = 3;
    } else if (AvailL) {
        if (LeftSingle)
            ctx = check_backward(LeftRefFrame[0]);
        else
            ctx = 3;
    } else {
        ctx = 1;
    };
    return ctx;
}
static bool is_samedir_ref_pair(uint8_t ref0, uint8_t ref1)
{
    return (ref0 >= BWDREF_FRAME) == (ref1 >= BWDREF_FRAME);
}
uint8_t Block::getCompReferenceTypeCtx()
{
    uint8_t ctx;
    uint8_t above0 = AboveRefFrame[0];
    uint8_t above1 = AboveRefFrame[1];
    uint8_t left0 = LeftRefFrame[0];
    uint8_t left1 = LeftRefFrame[1];
    bool aboveCompInter = AvailU && !AboveIntra && !AboveSingle;
    bool leftCompInter = AvailL && !LeftIntra && !LeftSingle;
    bool aboveUniComp = aboveCompInter && is_samedir_ref_pair(above0, above1);
    bool leftUniComp = leftCompInter && is_samedir_ref_pair(left0, left1);
    if (AvailU && !AboveIntra && AvailL && !LeftIntra) {
        bool samedir = is_samedir_ref_pair(above0, left0);
        if (!aboveCompInter && !leftCompInter) {
            ctx = 1 + 2 * samedir;
        } else if (!aboveCompInter) {
            if (!leftUniComp)
                ctx = 1;
            else
                ctx = 3 + samedir;
        } else if (!leftCompInter) {
            if (!aboveUniComp)
                ctx = 1;
            else
                ctx = 3 + samedir;
        } else {
            if (!aboveUniComp && !leftUniComp)
                ctx = 0;
            else if (!aboveUniComp || !leftUniComp)
                ctx = 2;
            else
                ctx = 3 + ((above0 == BWDREF_FRAME) == (left0 == BWDREF_FRAME));
        }
    } else if (AvailU && AvailL) {
        if (aboveCompInter)
            ctx = 1 + 2 * aboveUniComp;
        else if (leftCompInter)
            ctx = 1 + 2 * leftUniComp;
        else
            ctx = 2;
    } else if (aboveCompInter) {
        ctx = 4 * aboveUniComp;
    } else if (leftCompInter) {
        ctx = 4 * leftUniComp;
    } else {
        ctx = 2;
    }
    return ctx;
}
uint8_t Block::count_refs(uint8_t frameType)
{
    uint8_t c = 0;
    if (AvailU) {
        if (AboveRefFrame[0] == frameType)
            c++;
        if (AboveRefFrame[1] == frameType)
            c++;
    }
    if (AvailL) {
        if (LeftRefFrame[0] == frameType)
            c++;
        if (LeftRefFrame[1] == frameType)
            c++;
    }
    return c;
}

uint8_t Block::ref_count_ctx(uint8_t counts0, uint8_t counts1)
{
    if (counts0 < counts1)
        return 0;
    else if (counts0 == counts1)
        return 1;
    else
        return 2;
}

uint8_t Block::getUniCompRefCtx()
{
    return getSingleRefP1Ctx();
}

uint8_t Block::getUniCompRefP1Ctx()
{
    uint8_t last2Count = count_refs(LAST2_FRAME);
    uint8_t last3GoldCount = count_refs(LAST3_FRAME) + count_refs(GOLDEN_FRAME);
    uint8_t ctx = ref_count_ctx(last2Count, last3GoldCount);
    return ctx;
}

uint8_t Block::getUniCompRefP2Ctx()
{
    return getCompRefP2Ctx();
}

uint8_t Block::getCompRefCtx()
{
    uint8_t last12Count = count_refs(LAST_FRAME) + count_refs(LAST2_FRAME);
    uint8_t last3GoldCount = count_refs(LAST3_FRAME) + count_refs(GOLDEN_FRAME);
    uint8_t ctx = ref_count_ctx(last12Count, last3GoldCount);
    return ctx;
}

uint8_t Block::getCompRefP1Ctx()
{
    uint8_t lastCount = count_refs(LAST_FRAME);
    uint8_t last2Count = count_refs(LAST2_FRAME);
    uint8_t ctx = ref_count_ctx(lastCount, last2Count);
    return ctx;
}

uint8_t Block::getCompRefP2Ctx()
{
    uint8_t last3Count = count_refs(LAST3_FRAME);
    uint8_t goldCount = count_refs(GOLDEN_FRAME);
    uint8_t ctx = ref_count_ctx(last3Count, goldCount);
    return ctx;
}

uint8_t Block::getCompBwdRefCtx()
{
    uint8_t brfarf2Count = count_refs(BWDREF_FRAME) + count_refs(ALTREF2_FRAME);
    uint8_t arfCount = count_refs(ALTREF_FRAME);
    uint8_t ctx = ref_count_ctx(brfarf2Count, arfCount);
    return ctx;
}

uint8_t Block::getCompBwdRefP1Ctx()
{
    uint8_t brfCount = count_refs(BWDREF_FRAME);
    uint8_t arf2Count = count_refs(ALTREF2_FRAME);
    uint8_t ctx = ref_count_ctx(brfCount, arf2Count);
    return ctx;
}

void Block::readCompReference()
{
    COMP_REFERENCE_TYPE comp_ref_type = m_entropy.readCompReferenceType(getCompReferenceTypeCtx());
    if (comp_ref_type == UNIDIR_COMP_REFERENCE) {
        bool uni_comp_ref = m_entropy.readUniCompRef(getUniCompRefCtx());
        if (uni_comp_ref) {
            RefFrame[0] = BWDREF_FRAME;
            RefFrame[1] = ALTREF_FRAME;
        } else {
            bool uni_comp_ref_p1 = m_entropy.readUniCompRefP1(getUniCompRefP1Ctx());
            if (uni_comp_ref_p1) {
                bool uni_comp_ref_p2 = m_entropy.readUniCompRefP2(getUniCompRefP2Ctx());
                if (uni_comp_ref_p2) {
                    RefFrame[0] = LAST_FRAME;
                    RefFrame[1] = GOLDEN_FRAME;
                } else {
                    RefFrame[0] = LAST_FRAME;
                    RefFrame[1] = LAST3_FRAME;
                }
            } else {
                RefFrame[0] = LAST_FRAME;
                RefFrame[1] = LAST2_FRAME;
            }
        }
    } else {
        bool comp_ref = m_entropy.readCompRef(getCompRefCtx());
        if (!comp_ref) {
            bool comp_ref_p1 = m_entropy.readCompRefP1(getCompRefP1Ctx());
            RefFrame[0] = comp_ref_p1 ? LAST2_FRAME : LAST_FRAME;
        } else {
            bool comp_ref_p2 = m_entropy.readCompRefP2(getCompRefP2Ctx());
            RefFrame[0] = comp_ref_p2 ? GOLDEN_FRAME : LAST3_FRAME;
        }
        bool comp_bwdref = m_entropy.readCompBwdRef(getCompBwdRefCtx());
        if (!comp_bwdref) {
            bool comp_bwdref_p1 = m_entropy.readCompBwdRefP1(getCompBwdRefP1Ctx());
            RefFrame[1] = comp_bwdref_p1 ? ALTREF2_FRAME : BWDREF_FRAME;
        } else {
            RefFrame[1] = ALTREF_FRAME;
        }
    }
}

uint8_t Block::getSingleRefP1Ctx()
{
    uint8_t fwdCount, bwdCount;
    fwdCount = count_refs(LAST_FRAME);
    fwdCount += count_refs(LAST2_FRAME);
    fwdCount += count_refs(LAST3_FRAME);
    fwdCount += count_refs(GOLDEN_FRAME);
    bwdCount = count_refs(BWDREF_FRAME);
    bwdCount += count_refs(ALTREF2_FRAME);
    bwdCount += count_refs(ALTREF_FRAME);
    uint8_t ctx = ref_count_ctx(fwdCount, bwdCount);
    return ctx;
}

uint8_t Block::getSingleRefP2Ctx()
{
    return getCompBwdRefCtx();
}

uint8_t Block::getSingleRefP3Ctx()
{
    return getCompRefCtx();
}

uint8_t Block::getSingleRefP4Ctx()
{
    return getCompRefP1Ctx();
}

uint8_t Block::getSingleRefP5Ctx()
{
    return getCompRefP2Ctx();
}

uint8_t Block::getSingleRefP6Ctx()
{
    return getCompBwdRefP1Ctx();
}

void Block::readSingleReference()
{
    bool single_ref_p1 = m_entropy.readSingleRef(getSingleRefP1Ctx(), 1);
    if (single_ref_p1) {
        bool single_ref_p2 = m_entropy.readSingleRef(getSingleRefP2Ctx(), 2);
        if (!single_ref_p2) {
            bool single_ref_p6 = m_entropy.readSingleRef(getSingleRefP6Ctx(), 6);
            RefFrame[0] = single_ref_p6 ? ALTREF2_FRAME : BWDREF_FRAME;
        } else {
            RefFrame[0] = ALTREF_FRAME;
        }
    } else {
        bool single_ref_p3 = m_entropy.readSingleRef(getSingleRefP3Ctx(), 3);
        if (single_ref_p3) {
            bool single_ref_p5 = m_entropy.readSingleRef(getSingleRefP5Ctx(), 5);
            RefFrame[0] = single_ref_p5 ? GOLDEN_FRAME : LAST3_FRAME;
        } else {
            bool single_ref_p4 = m_entropy.readSingleRef(getSingleRefP4Ctx(), 4);
            RefFrame[0] = single_ref_p4 ? LAST2_FRAME : LAST_FRAME;
        }
    }
    RefFrame[1] = NONE_FRAME;
}
void Block::read_ref_frames()
{
    if (skip_mode) {
        RefFrame[0] = m_frame.SkipModeFrame[0];
        RefFrame[1] = m_frame.SkipModeFrame[1];
    } else if (seg_feature_active(SEG_LVL_REF_FRAME)) {
        RefFrame[0] = getSegFeature(SEG_LVL_REF_FRAME);
        RefFrame[1] = NONE_FRAME;
    } else if (seg_feature_active(SEG_LVL_SKIP)
        || seg_feature_active(SEG_LVL_GLOBALMV)) {
        RefFrame[0] = LAST_FRAME;
        RefFrame[1] = NONE_FRAME;
    } else {
        CompMode comp_mode;
        if (m_frame.reference_select && (std::min(bw4, bh4) >= 2)) {
            comp_mode = m_entropy.readCompMode(getCompModeCtx());
        } else {
            comp_mode = SINGLE_REFERENCE;
        }
        if (comp_mode == COMPOUND_REFERENCE) {
            readCompReference();
        } else {
            readSingleReference();
        }
    }
}

bool Block::needs_interp_filter()
{
    bool large = (std::min(bw4 << 2, bh4 << 2) >= 8);
    if (skip_mode || motion_mode == WARPED_CAUSAL) {
        return false;
    }
    if (large && YMode == GLOBALMV) {
        return m_frame.GmType[RefFrame[0]] == TRANSLATION;
    }
    if (large && YMode == GLOBAL_GLOBALMV) {
        return m_frame.GmType[RefFrame[0]] == TRANSLATION || m_frame.GmType[RefFrame[1]] == TRANSLATION;
    }
    return true;
}

uint8_t Block::getInterpFilterCtx(int dir)
{
    uint8_t ctx = ((dir & 1) * 2 + (RefFrame[1] > INTRA_FRAME)) * 4;
    uint8_t leftType = 3;
    uint8_t aboveType = 3;
    if (AvailL) {
        if (m_frame.RefFrames[MiRow][MiCol - 1][0] == RefFrame[0] || m_frame.RefFrames[MiRow][MiCol - 1][1] == RefFrame[0])
            leftType = m_frame.InterpFilters[MiRow][MiCol - 1][dir];
    }
    if (AvailU) {
        if (m_frame.RefFrames[MiRow - 1][MiCol][0] == RefFrame[0] || m_frame.RefFrames[MiRow - 1][MiCol][1] == RefFrame[0])
            aboveType = m_frame.InterpFilters[MiRow - 1][MiCol][dir];
    }
    if (leftType == aboveType)
        ctx += leftType;
    else if (leftType == 3)
        ctx += aboveType;
    else if (aboveType == 3)
        ctx += leftType;
    else
        ctx += 3;
    return ctx;
}

void Block::inter_block_mode_info()
{
    PaletteSizeY = 0;
    PaletteSizeUV = 0;
    read_ref_frames();
    bool isCompound = RefFrame[1] > INTRA_FRAME;

    FindMvStack find(*this);
    find.find_mv_stack();
    printf("++++++++++(%d, %d)\r\n", MiCol*4, MiRow*4);
    for (int i = 0; i < find.getNumMvFound(); i++) {
        printf("%d: (%d, %d)\r\n", i, find.RefStackMv[i]->mv[1], find.RefStackMv[i]->mv[0]);
    }
    printf("----------\r\n");

    if (skip_mode) {
        YMode = NEAREST_NEARESTMV;
    } else if (seg_feature_active(SEG_LVL_SKIP)
        || seg_feature_active(SEG_LVL_GLOBALMV)) {
        YMode = GLOBALMV;
    } else if (isCompound) {
        YMode = (PREDICTION_MODE)(NEAREST_NEARESTMV + m_entropy.readCompoundMode(find.getCompoundModeCtx()));
    } else {
        bool new_mv = m_entropy.readNewMv(find.getNewMvCtx());
        if (!new_mv) {
            YMode = NEWMV;
        } else {
            bool zero_mv = m_entropy.readZeroMv(find.getZeroMvCtx());
            if (!zero_mv) {
                YMode = GLOBALMV;
            } else {
                bool ref_mv = m_entropy.readRefMv(find.getRefMvCtx());
                YMode = !ref_mv ? NEARESTMV : NEARMV;
            }
        }
    }
    RefMvIdx = 0;
    if (YMode == NEWMV || YMode == NEW_NEWMV) {
        int NumMvFound = find.getNumMvFound();
        for (uint8_t idx = 0; idx < 2; idx++) {
            if (NumMvFound > idx + 1) {
                bool drl_mode = m_entropy.readDrlMode(find.getDrlModeCtx(idx));
                if (!drl_mode) {
                    RefMvIdx = idx;
                    break;
                }
                RefMvIdx = idx + 1;
            }
        }
    } else if (has_nearmv()) {
        RefMvIdx = 1;
        for (uint8_t idx = 1; idx < 3; idx++) {
            int NumMvFound = find.getNumMvFound();
            if (NumMvFound > idx + 1) {
                bool drl_mode = m_entropy.readDrlMode(find.getDrlModeCtx(idx));
                if (!drl_mode) {
                    RefMvIdx = idx;
                    break;
                }
                RefMvIdx = idx + 1;
            }
        }
    }
    assign_mv(find, isCompound);
    read_interintra_mode(isCompound);
    read_motion_mode(isCompound);
    read_compound_type(isCompound);
    if (m_frame.interpolation_filter == SWITCHABLE) {
        for (int dir = 0; dir < (m_sequence.enable_dual_filter ? 2 : 1); dir++) {
            if (needs_interp_filter()) {
                interp_filter[dir] = m_entropy.readInterpFilter(getInterpFilterCtx(dir));
            } else {
                interp_filter[dir] = EIGHTTAP_REGULAR;
            }
        }
        if (!m_sequence.enable_dual_filter)
            interp_filter[1] = interp_filter[0];
    } else {
        for (int dir = 0; dir < 2; dir++)
            interp_filter[dir] = m_frame.interpolation_filter;
    }
}

static int Wedge_Bits[BLOCK_SIZES_ALL] = {
    0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 4, 4, 0, 0
};

uint8_t Block::getCompGroupIdxCtx()
{
    uint8_t ctx = 0;
    if (AvailU) {
        if (!AboveSingle)
            ctx += m_frame.CompGroupIdxs[MiRow - 1][MiCol];
        else if (AboveRefFrame[0] == ALTREF_FRAME)
            ctx += 3;
    }
    if (AvailL) {
        if (!LeftSingle)
            ctx += m_frame.CompGroupIdxs[MiRow][MiCol - 1];
        else if (LeftRefFrame[0] == ALTREF_FRAME)
            ctx += 3;
    }
    ctx = std::min((uint8_t)5, ctx);
    return ctx;
}

uint8_t Block::getCompoundIdxCtx()
{
    uint8_t fwd = std::abs(m_frame.get_relative_dist(RefFrame[0]));
    uint8_t bck = std::abs(m_frame.get_relative_dist(RefFrame[1]));
    uint8_t ctx = (fwd == bck) ? 3 : 0;
    if (AvailU) {
        if (!AboveSingle)
            ctx += m_frame.CompoundIdxs[MiRow - 1][MiCol];
        else if (AboveRefFrame[0] == ALTREF_FRAME)
            ctx++;
    }
    if (AvailL) {
        if (!LeftSingle)
            ctx += m_frame.CompoundIdxs[MiRow][MiCol - 1];
        else if (LeftRefFrame[0] == ALTREF_FRAME)
            ctx++;
    }
    return ctx;
}

void Block::read_compound_type(bool isCompound)
{
    comp_group_idx = false;
    compound_idx = true;
    if (skip_mode) {
        compound_type = COMPOUND_AVERAGE;
        return;
    }
    if (isCompound) {
        int n = Wedge_Bits[MiSize];
        if (m_sequence.enable_masked_compound) {
            comp_group_idx = m_entropy.readCompGroupIdx(getCompGroupIdxCtx());
        }
        if (!comp_group_idx) {
            if (m_sequence.enable_jnt_comp) {
                compound_idx = m_entropy.readCompoundIdx(getCompoundIdxCtx());
                compound_type = compound_idx ? COMPOUND_AVERAGE : COMPOUND_DISTANCE;
            } else {
                compound_type = COMPOUND_AVERAGE;
            }
        } else {
            if (n == 0) {
                compound_type = COMPOUND_DIFFWTD;
            } else {
                compound_type = m_entropy.readCompoundType(MiSize);
            }
        }
        if (compound_type == COMPOUND_WEDGE) {
            wedge_index = m_entropy.readWedgeIndex(MiSize);
            wedge_sign = m_entropy.readWedgeSign();
        } else if (compound_type == COMPOUND_DIFFWTD) {
            mask_type = m_entropy.readMaskType();
        }
    } else {
        if (interintra) {
            compound_type = wedge_interintra ? COMPOUND_WEDGE : COMPOUND_INTRA;
        } else {
            compound_type = COMPOUND_AVERAGE;
        }
    }
}

bool Block::has_overlappable_candidates()
{
    if (AvailU) {
        for (uint32_t x4 = MiCol; x4 < std::min(m_frame.MiCols, MiCol + bw4); x4 += 2) {
            if (m_frame.RefFrames[MiRow - 1][x4 | 1][0] > INTRA_FRAME)
                return true;
        }
    }
    if (AvailL) {
        for (uint32_t y4 = MiRow; y4 < std::min(m_frame.MiRows, MiRow + bh4); y4 += 2) {
            if (m_frame.RefFrames[y4 | 1][MiCol - 1][0] > INTRA_FRAME)
                return true;
        }
    }
    return false;
}

Block::LocalWarp::LocalWarp(Block& block)
    : m_block(block)
    , m_tile(block.m_tile)
    , m_frame(block.m_frame)
    , w4(block.bw4)
    , h4(block.bh4)
    , MiCol(m_block.MiCol)
    , MiRow(m_block.MiRow)
{
}

void Block::LocalWarp::find_warp_samples()
{
    bool doTopLeft = true;
    bool doTopRight = true;
    if (m_block.AvailU) {
        BLOCK_SIZE srcSize = m_frame.MiSizes[MiRow - 1][MiCol];
        int srcW = Num_4x4_Blocks_Wide[srcSize];
        if (w4 <= srcW) {
            int colOffset = -(MiCol & (srcW - 1));
            if (colOffset < 0)
                doTopLeft = false;
            if (colOffset + srcW > w4)
                doTopRight = false;
            add_sample(-1, 0);
        } else {
            int miStep;
            for (int i = 0; i < std::min(w4, (int)(m_frame.MiCols - MiCol)); i += miStep) {
                srcSize = m_frame.MiSizes[MiRow - 1][MiCol + i];
                srcW = Num_4x4_Blocks_Wide[srcSize];
                miStep = std::min(w4, srcW);
                add_sample(-1, i);
            }
        }
    }
    if (m_block.AvailL) {
        BLOCK_SIZE srcSize = m_frame.MiSizes[MiRow][MiCol - 1];
        int srcH = Num_4x4_Blocks_High[srcSize];
        if (h4 <= srcH) {
            int rowOffset = -(MiRow & (srcH - 1));
            if (rowOffset < 0)
                doTopLeft = false;
            add_sample(0, -1);
        } else {
            int miStep;
            for (int i = 0; i < std::min(h4, (int)(m_frame.MiRows - MiRow)); i += miStep) {
                srcSize = m_frame.MiSizes[MiRow + i][MiCol - 1];
                srcH = Num_4x4_Blocks_High[srcSize];
                miStep = std::min(h4, srcH);
                add_sample(i, -1);
            }
        }
    }
    if (doTopLeft) {
        add_sample(-1, -1);
    }
    if (doTopRight) {
        if (std::max(w4, h4) <= 16) {
            add_sample(-1, w4);
        }
    }
    if (NumSamples == 0 && NumSamplesScanned > 0)
        NumSamples = 1;
}
void Block::LocalWarp::add_sample(int deltaRow, int deltaCol)
{
    static const int LEAST_SQUARES_SAMPLES_MAX = 8;
    if (NumSamplesScanned >= LEAST_SQUARES_SAMPLES_MAX)
        return;
    uint32_t mvRow = MiRow + deltaRow;
    uint32_t mvCol = MiCol + deltaCol;
    if (!m_tile.is_inside(mvRow, mvCol))
        return;
    if (m_frame.RefFrames[mvRow][mvCol][0] == NONE_FRAME)
        return;
    if (m_frame.RefFrames[mvRow][mvCol][0] != m_block.RefFrame[0])
        return;
    if (m_frame.RefFrames[mvRow][mvCol][1] != NONE_FRAME)
        return;
    BLOCK_SIZE candSz = m_frame.MiSizes[mvRow][mvCol];
    int candW4 = Num_4x4_Blocks_Wide[candSz];
    int candH4 = Num_4x4_Blocks_High[candSz];
    int candRow = mvRow & ~(candH4 - 1);
    int candCol = mvCol & ~(candW4 - 1);
    int midY = candRow * 4 + candH4 * 2 - 1;
    int midX = candCol * 4 + candW4 * 2 - 1;
    int threshold = CLIP3(16, 112, (int)std::max(m_block.bw, m_block.bh));
    int mvDiffRow = std::abs(m_frame.Mvs[candRow][candCol][0].mv[0] - m_block.m_mv[0].mv[0]);
    int mvDiffCol = std::abs(m_frame.Mvs[candRow][candCol][0].mv[1] - m_block.m_mv[0].mv[1]);
    bool valid = ((mvDiffRow + mvDiffCol) <= threshold);
    std::vector<int16_t> cand(4);
    cand[0] = midY * 8;
    cand[1] = midX * 8;
    cand[2] = midY * 8 + m_frame.Mvs[candRow][candCol][0].mv[0];
    cand[3] = midX * 8 + m_frame.Mvs[candRow][candCol][0].mv[1];
    NumSamplesScanned += 1;
    if (!valid && NumSamplesScanned > 1)
        return;
    CandList.resize(NumSamples + 1);
    CandList[NumSamples] = cand;
    if (valid)
        NumSamples += 1;
}

const static int DIV_LUT_NUM = 257;

const static int Div_Lut[DIV_LUT_NUM] = {
    16384, 16320, 16257, 16194, 16132, 16070, 16009, 15948, 15888, 15828, 15768,
    15709, 15650, 15592, 15534, 15477, 15420, 15364, 15308, 15252, 15197, 15142,
    15087, 15033, 14980, 14926, 14873, 14821, 14769, 14717, 14665, 14614, 14564,
    14513, 14463, 14413, 14364, 14315, 14266, 14218, 14170, 14122, 14075, 14028,
    13981, 13935, 13888, 13843, 13797, 13752, 13707, 13662, 13618, 13574, 13530,
    13487, 13443, 13400, 13358, 13315, 13273, 13231, 13190, 13148, 13107, 13066,
    13026, 12985, 12945, 12906, 12866, 12827, 12788, 12749, 12710, 12672, 12633,
    12596, 12558, 12520, 12483, 12446, 12409, 12373, 12336, 12300, 12264, 12228,
    12193, 12157, 12122, 12087, 12053, 12018, 11984, 11950, 11916, 11882, 11848,
    11815, 11782, 11749, 11716, 11683, 11651, 11619, 11586, 11555, 11523, 11491,
    11460, 11429, 11398, 11367, 11336, 11305, 11275, 11245, 11215, 11185, 11155,
    11125, 11096, 11067, 11038, 11009, 10980, 10951, 10923, 10894, 10866, 10838,
    10810, 10782, 10755, 10727, 10700, 10673, 10645, 10618, 10592, 10565, 10538,
    10512, 10486, 10460, 10434, 10408, 10382, 10356, 10331, 10305, 10280, 10255,
    10230, 10205, 10180, 10156, 10131, 10107, 10082, 10058, 10034, 10010, 9986,
    9963, 9939, 9916, 9892, 9869, 9846, 9823, 9800, 9777, 9754, 9732,
    9709, 9687, 9664, 9642, 9620, 9598, 9576, 9554, 9533, 9511, 9489,
    9468, 9447, 9425, 9404, 9383, 9362, 9341, 9321, 9300, 9279, 9259,
    9239, 9218, 9198, 9178, 9158, 9138, 9118, 9098, 9079, 9059, 9039,
    9020, 9001, 8981, 8962, 8943, 8924, 8905, 8886, 8867, 8849, 8830,
    8812, 8793, 8775, 8756, 8738, 8720, 8702, 8684, 8666, 8648, 8630,
    8613, 8595, 8577, 8560, 8542, 8525, 8508, 8490, 8473, 8456, 8439,
    8422, 8405, 8389, 8372, 8355, 8339, 8322, 8306, 8289, 8273, 8257,
    8240, 8224, 8208, 8192
};

void Block::LocalWarp::resolveDivisor(int64_t d, int& divShift, int& divFactor) const
{
    static const int DIV_LUT_BITS = 8;
    static const int DIV_LUT_PREC_BITS = 14;
    int n = FloorLog2(std::abs(d));
    int64_t e = std::abs(d) - ((int64_t)1 << n);
    int64_t f = n > DIV_LUT_BITS ? ROUND2(e, n - DIV_LUT_BITS) : (e << (DIV_LUT_BITS - n));
    divShift = (n + DIV_LUT_PREC_BITS);
    divFactor = d < 0 ? -Div_Lut[f] : Div_Lut[f];
}
inline int ls_product(int a, int b)
{
    return ((a * b) >> 2) + (a + b);
}

static const int WARPEDMODEL_NONDIAGAFFINE_CLAMP = 1 << 13;

inline int nondiag(int64_t v, int divFactor, int divShift)
{
    return CLIP3(-WARPEDMODEL_NONDIAGAFFINE_CLAMP + 1,
        WARPEDMODEL_NONDIAGAFFINE_CLAMP - 1,
        ROUND2SIGNED_64((int64_t)v * divFactor, divShift));
}
inline int diag(int64_t v, int divFactor, int divShift)
{
    return CLIP3((1 << WARPEDMODEL_PREC_BITS) - WARPEDMODEL_NONDIAGAFFINE_CLAMP + 1,
        (1 << WARPEDMODEL_PREC_BITS) + WARPEDMODEL_NONDIAGAFFINE_CLAMP - 1,
        ROUND2SIGNED_64((int64_t)v * divFactor, divShift));
}
void Block::LocalWarp::warpEstimation()
{
    int64_t A[2][2];
    int64_t Bx[2];
    int64_t By[2];
    memset(A, 0, sizeof(A));
    Bx[1] = Bx[0] = 0;
    By[1] = By[0] = 0;
    int midY = MiRow * 4 + h4 * 2 - 1;
    int midX = MiCol * 4 + w4 * 2 - 1;
    int suy = midY * 8;
    int sux = midX * 8;
    int duy = suy + m_block.m_mv[0].mv[0];
    int dux = sux + m_block.m_mv[0].mv[1];
    for (int i = 0; i < NumSamples; i++) {
        static const int LS_MV_MAX = 256;
        int sy = CandList[i][0] - suy;
        int sx = CandList[i][1] - sux;
        int dy = CandList[i][2] - duy;
        int dx = CandList[i][3] - dux;
        if (std::abs(sx - dx) < LS_MV_MAX && std::abs(sy - dy) < LS_MV_MAX) {
            A[0][0] += ls_product(sx, sx) + 8;
            A[0][1] += ls_product(sx, sy) + 4;
            A[1][1] += ls_product(sy, sy) + 8;
            Bx[0] += ls_product(sx, dx) + 8;
            Bx[1] += ls_product(sy, dx) + 4;
            By[0] += ls_product(sx, dy) + 4;
            By[1] += ls_product(sy, dy) + 8;
        }
    }
    int64_t det =A[0][0] * A[1][1] - A[0][1] * A[0][1];
    LocalValid = (det != 0);
    if (!LocalValid)
        return;

    int divShift, divFactor;
    resolveDivisor(det, divShift, divFactor);
    divShift -= WARPEDMODEL_PREC_BITS;
    if (divShift < 0) {
        divFactor = divFactor << (-divShift);
        divShift = 0;
    }
    LocalWarpParams[2] = diag(A[1][1] * Bx[0] - A[0][1] * Bx[1], divFactor, divShift);
    LocalWarpParams[3] = nondiag(-A[0][1] * Bx[0] + A[0][0] * Bx[1], divFactor, divShift);
    LocalWarpParams[4] = nondiag(A[1][1] * By[0] - A[0][1] * By[1], divFactor, divShift);
    LocalWarpParams[5] = diag(-A[0][1] * By[0] + A[0][0] * By[1], divFactor, divShift);
    int16_t mvx = m_block.m_mv[0].mv[1];
    int16_t mvy = m_block.m_mv[0].mv[0];
    int vx = mvx * (1 << (WARPEDMODEL_PREC_BITS - 3)) - (midX * (LocalWarpParams[2] - (1 << WARPEDMODEL_PREC_BITS)) + midY * LocalWarpParams[3]);
    int vy = mvy * (1 << (WARPEDMODEL_PREC_BITS - 3)) - (midX * LocalWarpParams[4] + midY * (LocalWarpParams[5] - (1 << WARPEDMODEL_PREC_BITS)));
    const static int WARPEDMODEL_TRANS_CLAMP = 1 << 23;
    LocalWarpParams[0] = CLIP3(-WARPEDMODEL_TRANS_CLAMP, WARPEDMODEL_TRANS_CLAMP - 1, vx);
    LocalWarpParams[1] = CLIP3(-WARPEDMODEL_TRANS_CLAMP, WARPEDMODEL_TRANS_CLAMP - 1, vy);
}

void Block::LocalWarp::setupShear()
{
    if (LocalValid) {
        int alpha, beta, gamma, delta;
        LocalValid = setupShear(LocalWarpParams, alpha, beta, gamma, delta);
    }
}

bool Block::LocalWarp::setupShear(const int warpParams[6], int& alpha, int& beta, int& gamma, int& delta) const
{
    int alpha0 = CLIP3(-32768, 32767, warpParams[2] - (1 << WARPEDMODEL_PREC_BITS));
    int beta0 = CLIP3(-32768, 32767, warpParams[3]);
    int divShift, divFactor;
    resolveDivisor(warpParams[2], divShift, divFactor);
    int64_t v = (warpParams[4] << WARPEDMODEL_PREC_BITS);
    int gamma0 = CLIP3(-32768, 32767, ROUND2SIGNED(v * divFactor, divShift));
    int64_t w = (warpParams[3] * warpParams[4]);
    int delta0 = CLIP3(-32768, 32767, warpParams[5] - ROUND2SIGNED(w * divFactor, divShift) - (1 << WARPEDMODEL_PREC_BITS));

    static const int WARP_PARAM_REDUCE_BITS = 6;
    alpha = ROUND2SIGNED(alpha0, WARP_PARAM_REDUCE_BITS) << WARP_PARAM_REDUCE_BITS;
    beta = ROUND2SIGNED(beta0, WARP_PARAM_REDUCE_BITS) << WARP_PARAM_REDUCE_BITS;
    gamma = ROUND2SIGNED(gamma0, WARP_PARAM_REDUCE_BITS) << WARP_PARAM_REDUCE_BITS;
    delta = ROUND2SIGNED(delta0, WARP_PARAM_REDUCE_BITS) << WARP_PARAM_REDUCE_BITS;
    if ((4 * std::abs(alpha) + 7 * std::abs(beta)) >= (1 << WARPEDMODEL_PREC_BITS))
        return false;
    if ((4 * std::abs(gamma) + 4 * std::abs(delta)) >= (1 << WARPEDMODEL_PREC_BITS))
        return false;
    return true;
}

void Block::read_motion_mode(bool isCompound)
{
    if (skip_mode) {
        motion_mode = SIMPLE_TRANSLATION;
        return;
    }
    if (!m_frame.is_motion_mode_switchable) {
        motion_mode = SIMPLE_TRANSLATION;
        return;
    }
    if (std::min(Block_Width[MiSize],
            Block_Height[MiSize])
        < 8) {
        motion_mode = SIMPLE_TRANSLATION;
        return;
    }
    if (!m_frame.force_integer_mv && (YMode == GLOBALMV || YMode == GLOBAL_GLOBALMV)) {
        if (m_frame.GmType[RefFrame[0]] > TRANSLATION) {
            motion_mode = SIMPLE_TRANSLATION;
            return;
        }
    }
    if (isCompound || RefFrame[1] == INTRA_FRAME || !has_overlappable_candidates()) {
        motion_mode = SIMPLE_TRANSLATION;
        return;
    }

    m_localWarp.find_warp_samples();
    if (m_frame.force_integer_mv || m_localWarp.NumSamples == 0 || !m_frame.allow_warped_motion || m_frame.is_scaled(RefFrame[0])) {
        bool use_obmc = m_entropy.readUseObmc(MiSize);
        motion_mode = use_obmc ? OBMC_CAUSAL : SIMPLE_TRANSLATION;
    } else {
        motion_mode = m_entropy.readMotionMode(MiSize);
    }
}

void Block::read_interintra_mode(bool isCompound)
{
    if (!skip_mode && m_sequence.enable_interintra_compound && !isCompound && MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32) {
        interintra = m_entropy.readInterIntra(MiSize);
        if (interintra) {
            interintra_mode = m_entropy.readInterIntraMode(MiSize);
            RefFrame[1] = INTRA_FRAME;
            AngleDeltaY = 0;
            AngleDeltaUV = 0;
            use_filter_intra = 0;
            wedge_interintra = m_entropy.readWedgeInterIntra(MiSize);
            if (wedge_interintra) {
                wedge_index = m_entropy.readWedgeIndex(MiSize);
                wedge_sign = 0;
            }
        }
    } else {
        interintra = false;
    }
}

PREDICTION_MODE Block::get_mode(int refList)
{
    PREDICTION_MODE compMode;
    if (refList == 0) {
        if (YMode < NEAREST_NEARESTMV)
            compMode = YMode;
        else if (YMode == NEW_NEWMV || YMode == NEW_NEARESTMV || YMode == NEW_NEARMV)
            compMode = NEWMV;
        else if (YMode == NEAREST_NEARESTMV || YMode == NEAREST_NEWMV)
            compMode = NEARESTMV;
        else if (YMode == NEAR_NEARMV || YMode == NEAR_NEWMV)
            compMode = NEARMV;
        else
            compMode = GLOBALMV;
    } else {
        if (YMode == NEW_NEWMV || YMode == NEAREST_NEWMV || YMode == NEAR_NEWMV)
            compMode = NEWMV;
        else if (YMode == NEAREST_NEARESTMV || YMode == NEW_NEARESTMV)
            compMode = NEARESTMV;
        else if (YMode == NEAR_NEARMV || YMode == NEW_NEARMV)
            compMode = NEARMV;
        else
            compMode = GLOBALMV;
    }
    return compMode;
}
int16_t Block::read_mv_component(uint8_t ctx, uint8_t comp)
{
    bool mv_sign = m_entropy.readMvSign(ctx, comp);
    int mag;
    int d;
    MV_CLASS_TYPE mv_class = m_entropy.readMvClass(ctx, comp);
    if (mv_class == MV_CLASS_0) {
        int mv_class0_bit = m_entropy.readMvClass0Bit(ctx, comp);
        int mv_class0_fr;
        int mv_class0_hp;
        if (m_frame.force_integer_mv)
            mv_class0_fr = 3;
        else
            mv_class0_fr = m_entropy.readMvClass0Fr(mv_class0_bit, ctx, comp);
        if (m_frame.allow_high_precision_mv)
            mv_class0_hp = m_entropy.readMvClass0Hp(ctx, comp);
        else
            mv_class0_hp = 1;
        mag = ((mv_class0_bit << 3) | (mv_class0_fr << 1) | mv_class0_hp) + 1;
    } else {
        d = 0;
        for (int i = 0; i < mv_class; i++) {
            int mv_bit = m_entropy.readMvBit(i, ctx, comp);
            d |= mv_bit << i;
        }
        mag = CLASS0_SIZE << (mv_class + 2);
        int mv_fr;
        int mv_hp;
        if (m_frame.force_integer_mv)
            mv_fr = 3;
        else
            mv_fr = m_entropy.readMvFr(ctx, comp);
        if (m_frame.allow_high_precision_mv)
            mv_hp = m_entropy.readMvHp(ctx, comp);
        else
            mv_hp = 1;
        mag += ((d << 3) | (mv_fr << 1) | mv_hp) + 1;
    }

    return mv_sign ? -mag : mag;
}
void Block::read_mv(Mv PredMv[2], int ref)
{
    int16_t diffMv[2];
    diffMv[0] = diffMv[1] = 0;

    uint8_t MvCtx;
    if (use_intrabc) {
        MvCtx = MV_INTRABC_CONTEXT;
    } else {
        MvCtx = 0;
    }
    MV_JOINT_TYPE mv_joint = m_entropy.readMvJoint(MvCtx);
    if (mv_joint == MV_JOINT_HZVNZ || mv_joint == MV_JOINT_HNZVNZ)
        diffMv[0] = read_mv_component(MvCtx, 0);
    if (mv_joint == MV_JOINT_HNZVZ || mv_joint == MV_JOINT_HNZVNZ)
        diffMv[1] = read_mv_component(MvCtx, 1);
    m_mv[ref].mv[0] = PredMv[ref].mv[0] + diffMv[0];
    m_mv[ref].mv[1] = PredMv[ref].mv[1] + diffMv[1];
}

void Block::assign_mv(const FindMvStack& find, bool isCompound)
{
    PREDICTION_MODE compMode;
    Mv PredMv[2];
    m_mv.resize(1 + isCompound);
    for (int i = 0; i < 1 + isCompound; i++) {
        if (use_intrabc) {
            compMode = NEWMV;
        } else {
            compMode = get_mode(i);
        }
        if (use_intrabc) {
            ASSERT(0);
            /*
            PredMv[ 0 ] = RefStackMv[ 0 ][ 0 ];
            if ( PredMv[ 0 ][ 0 ] == 0 && PredMv[ 0 ][ 1 ] == 0 ) {
                PredMv[ 0 ] = RefStackMv[ 1 ][ 0 ]
            }
            if ( PredMv[ 0 ][ 0 ] == 0 && PredMv[ 0 ][ 1 ] == 0 ) {
                sbSize = use_128x128_superblock ? BLOCK_128X128 : BLOCK_64X64
                sbSize4 = Num_4x4_Blocks_High[ sbSize ]
                if ( MiRow - sbSize4 < MiRowStart ) {
                    PredMv[ 0 ][ 0 ] = 0
                    PredMv[ 0 ][ 1 ] = -(sbSize4 * MI_SIZE + INTRABC_DELAY_PIXELS) * 8
                } else {
                    PredMv[ 0 ][ 0 ] = -(sbSize4 * MI_SIZE * 8)
                    PredMv[ 0 ][ 1 ] = 0
                }
            }*/
        } else if (compMode == GLOBALMV) {
            PredMv[i] = find.GlobalMvs[i];
        } else {
            uint8_t pos = (compMode == NEARESTMV) ? 0 : RefMvIdx;
            if (compMode == NEWMV && find.getNumMvFound() <= 1)
                pos = 0;
            PredMv[i] = find.RefStackMv[pos][i];
        }
        if (compMode == NEWMV) {
            read_mv(PredMv, i);
        } else {
            m_mv[i] = PredMv[i];
        }
    }
}

void Block::intra_block_mode_info()
{
    RefFrame[0] = INTRA_FRAME;
    RefFrame[1] = NONE_FRAME;
    YMode = m_entropy.readYMode(MiSize);
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
    if (MiSize >= BLOCK_8X8 && Block_Width[MiSize] <= 64 && Block_Height[MiSize] <= 64 && m_frame.allow_screen_content_tools) {
        ASSERT(0);
    }
    filter_intra_mode_info();
}

void Block::inter_frame_mode_info()
{
    use_intrabc = false;
    LeftRefFrame[0] = AvailL ? m_frame.RefFrames[MiRow][MiCol - 1][0] : INTRA_FRAME;
    AboveRefFrame[0] = AvailU ? m_frame.RefFrames[MiRow - 1][MiCol][0] : INTRA_FRAME;
    LeftRefFrame[1] = AvailL ? m_frame.RefFrames[MiRow][MiCol - 1][1] : NONE_FRAME;
    AboveRefFrame[1] = AvailU ? m_frame.RefFrames[MiRow - 1][MiCol][1] : NONE_FRAME;
    LeftIntra = LeftRefFrame[0] <= INTRA_FRAME;
    AboveIntra = AboveRefFrame[0] <= INTRA_FRAME;
    LeftSingle = LeftRefFrame[1] <= INTRA_FRAME;
    AboveSingle = AboveRefFrame[1] <= INTRA_FRAME;
    skip = false;

    inter_segment_id(true);
    read_skip_mode();
    if (skip_mode)
        skip = true;
    else
        skip = read_skip();
    if (!m_frame.m_segmentation.SegIdPreSkip) {
        inter_segment_id(false);
    }
    Lossless = m_frame.LosslessArray[segment_id];
    read_cdef();
    bool ReadDeltas = m_frame.m_deltaQ.delta_q_present;
    read_delta_qindex(ReadDeltas);
    read_delta_lf(ReadDeltas);
    read_is_inter();
    if (is_inter)
        inter_block_mode_info();
    else
        intra_block_mode_info();
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
    if (row == MiRow) {
        if (!AvailU) {
            return 64;
        } else if (m_frame.Skips[row - 1][col] && m_frame.IsInters[row - 1][col]) {
            return Block_Width[m_frame.MiSizes[row - 1][col]];
        }
    }
    return Tx_Width[m_frame.InterTxSizes[row - 1][col]];
}

int Block::get_left_tx_height(uint32_t row, uint32_t col)
{
    if (col == MiCol) {
        if (!AvailL) {
            return 64;
        } else if (m_frame.Skips[row][col - 1] && m_frame.IsInters[row][col - 1]) {
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

uint8_t Block::getTxfmSplitCtx(uint32_t row, uint32_t col, TX_SIZE txSz)
{
    int above = get_above_tx_width(row, col) < Tx_Width[txSz];
    int left = get_left_tx_height(row, col) < Tx_Height[txSz];
    int size = std::min(64, std::max(Block_Width[MiSize], Block_Height[MiSize]));
    int maxTxSz = find_tx_size(size, size);
    TX_SIZE txSzSqrUp = Tx_Size_Sqr_Up[txSz];

    uint8_t ctx = (txSzSqrUp != maxTxSz) * 3 + (TX_SIZES - 1 - maxTxSz) * 6 + above + left;
    return ctx;
}

void Block::read_var_tx_size(uint32_t row, uint32_t col, TX_SIZE txSz, int depth)
{
    if (row >= m_frame.MiRows || col >= m_frame.MiCols)
        return;
    bool txfm_split;
    if (txSz == TX_4X4 || depth == MAX_VARTX_DEPTH) {
        txfm_split = false;
    } else {
        txfm_split = m_entropy.readTxfmSplit(getTxfmSplitCtx(row, col, txSz));
    }
    int w4 = Tx_Width[txSz] / MI_SIZE;
    int h4 = Tx_Height[txSz] / MI_SIZE;
    if (txfm_split) {
        TX_SIZE subTxSz = Split_Tx_Size[txSz];
        int stepW = Tx_Width[subTxSz] / MI_SIZE;
        int stepH = Tx_Height[subTxSz] / MI_SIZE;
        for (int i = 0; i < h4; i += stepH)
            for (int j = 0; j < w4; j += stepW)
                read_var_tx_size(row + i, col + j, subTxSz, depth + 1);
    } else {
        for (int i = 0; i < h4; i++)
            for (int j = 0; j < w4; j++)
                m_frame.InterTxSizes[row + i][col + j] = txSz;
        TxSize = txSz;
    }
}

void Block::read_block_tx_size()
{
    if (m_frame.TxMode == TX_MODE_SELECT && MiSize > BLOCK_4X4 && is_inter && !skip && !m_frame.CodedLossless) {
        TX_SIZE maxTxSz = Max_Tx_Size_Rect[MiSize];
        int txW4 = Tx_Width[maxTxSz] / MI_SIZE;
        int txH4 = Tx_Height[maxTxSz] / MI_SIZE;
        for (uint32_t row = MiRow; row < MiRow + bh4; row += txH4)
            for (uint32_t col = MiCol; col < MiCol + bw4; col += txW4)
                read_var_tx_size(row, col, maxTxSz, 0);
    } else {
        read_tx_size(!skip || !is_inter);
        for (int row = MiRow; row < MiRow + bh4; row++) {
            for (int col = MiCol; col < MiCol + bw4; col++)
                m_frame.InterTxSizes[row][col] = TxSize;
        }
    }
}

bool Block::decode(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore)
{
    compute_prediction(frame, frameStore);
    for (auto& t : m_transformBlocks) {
        if (!t->decode(frame))
            return false;
    }
    return true;
}

bool Block::has_nearmv() const
{
    return (YMode == NEARMV
        || YMode == NEAR_NEARMV
        || YMode == NEAR_NEWMV
        || YMode == NEW_NEARMV);
}

}
