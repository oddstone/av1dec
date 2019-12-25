#include "Block.h"
#include "Av1Common.h"
#include "Av1Parser.h"
#include "Av1Tile.h"
#include "SymbolDecoder.h"
#include "TransformBlock.h"
#include "log.h"

#include <limits>

namespace Yami {
namespace Av1 {
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

    BLOCK_SIZE Block::get_plane_residual_size(int subsize, int plane)
    {
        return m_sequence.get_plane_residual_size(subsize, plane);
    }

    int16_t Block::get_q_idx()
    {
        return m_frame.m_segmentation.segmentation_enabled ? m_frame.get_qindex(true, segment_id) : m_frame.m_quant.base_q_idx;
    }

    class Block::PredictInter {
    public:
        PredictInter(const Block& block, YuvFrame& yuv);
        void predict_inter(int plane, int x, int y, int w, int h, int candRow, int candCol);

    private:
        uint8_t getUseWarp(int x, int y, int refFrame);
        void motionVectorScaling(uint8_t refIdx, int plane, int x, int y, const Mv& mv);
        void blockInterPrediction(int refList, int w, int h, int candRow, int candCol);

        const Block& m_block;
        const FrameHeader& m_frame;
        const SequenceHeader& m_sequence;
        YuvFrame& m_yuv;
        bool isCompound;
        int InterRound0;
        int InterRound1;
        int InterPostRound;
        bool LocalValid = false;
        bool globaValid = false;

        int startX;
        int startY;
        int stepX;
        int stepY;

        std::vector<std::vector<uint8_t>> preds[2];
    };

    Block::PredictInter::PredictInter(const Block& block, YuvFrame& yuv)
        : m_block(block)
        , m_frame(block.m_frame)
        , m_sequence(block.m_sequence)
        , m_yuv(yuv)
    {
    }

    uint8_t Block::PredictInter::getUseWarp(int w, int h, int refFrame)
    {
        if (w < 8 || h < 8)
            return false;
        if (m_frame.force_integer_mv)
            return false;
        if (m_block.motion_mode == LOCALWARP && LocalValid)
            return true;
        if ((m_block.YMode == GLOBALMV || m_block.YMode == GLOBAL_GLOBALMV)
            && m_frame.GmType[refFrame] > TRANSLATION
            && !m_frame.is_scaled(refFrame)
            && globaValid)
            return 2;
        return 0;
    }

    void Block::PredictInter::motionVectorScaling(uint8_t refIdx, int plane, int x, int y, const Mv& mv)
    {
        uint32_t xScale, yScale;
        m_frame.getScale(refIdx, xScale, yScale);

        const static uint8_t halfSample = (1 << (SUBPEL_BITS - 1));
        uint8_t subX = plane ? m_sequence.subsampling_x : 0;
        uint8_t subY = plane ? m_sequence.subsampling_y : 0;
        uint32_t origX = ((x << SUBPEL_BITS) + ((2 * mv.mv[1]) >> subX) + halfSample);
        uint32_t origY = ((y << SUBPEL_BITS) + ((2 * mv.mv[0]) >> subY) + halfSample);
        uint32_t baseX = (origX * xScale - (halfSample << REF_SCALE_SHIFT));
        uint32_t baseY = (origY * yScale - (halfSample << REF_SCALE_SHIFT));

        const static uint8_t off = ((1 << (SCALE_SUBPEL_BITS - SUBPEL_BITS)) / 2);

        startX = (ROUND2SIGNED(baseX, REF_SCALE_SHIFT + SUBPEL_BITS - SCALE_SUBPEL_BITS) + off);
        startY = (ROUND2SIGNED(baseY, REF_SCALE_SHIFT + SUBPEL_BITS - SCALE_SUBPEL_BITS) + off);
        stepX = ROUND2SIGNED(xScale, REF_SCALE_SHIFT - SCALE_SUBPEL_BITS);
        stepY = ROUND2SIGNED(yScale, REF_SCALE_SHIFT - SCALE_SUBPEL_BITS);
    }
    void Block::PredictInter::blockInterPrediction(int refList, int w, int h, int candRow, int candCol)
    {
        std::vector<std::vector<uint8_t>>& pred = preds[refList];
        pred.assign(h, std::vector<uint8_t>(w));


    }
    void Block::PredictInter::predict_inter(int plane, int x, int y, int w, int h, int candRow, int candCol)
    {
        isCompound = m_frame.RefFrames[candRow][candCol][1] > INTRA_FRAME;
        roundingVariablesDerivation(isCompound, m_sequence.BitDepth, InterRound0, InterRound1, InterPostRound);
        if (!plane && m_block.motion_mode == LOCALWARP) {
            ASSERT(0);
        }
        int refList = 0;
        int refFrame = m_frame.RefFrames[candRow][candCol][refList];
        if ((m_block.YMode == GLOBALMV || m_block.YMode == GLOBAL_GLOBALMV) && m_frame.GmType[refFrame] > TRANSLATION) {
            ASSERT(0);
        }
        uint8_t useWarp = getUseWarp(w, h, refFrame);
        Mv mv = m_frame.Mvs[candRow][candCol][refList];
        uint8_t refIdx;
        if (!m_block.use_intrabc) {
            refIdx = m_frame.ref_frame_idx[refFrame - LAST_FRAME];
        } else {
            ASSERT(0);
        }
        motionVectorScaling(refIdx, plane, x, y, mv);

        if (m_block.use_intrabc) {
            ASSERT(0);
        }
        if (useWarp) {
            ASSERT(0);
        } else {
            blockInterPrediction(refList, w, h, candRow, candCol);
        }
    }

    void Block::compute_prediction(std::shared_ptr<YuvFrame>& frame)
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

    class Block::FindMvStack {
    public:
        FindMvStack(Block& block);

        void find_mv_stack();
        uint8_t getCompoundModeCtx() const;
        uint8_t getNewMvCtx() const;
        uint8_t getZeroMvCtx() const;
        uint8_t getRefMvCtx() const;
        int getNumMvFound() const;
        uint8_t getDrlModeCtx(uint8_t idx) const;
        Mv RefStackMv[MAX_REF_MV_STACK_SIZE][2];
        Mv GlobalMvs[2];

    private:
        int16_t clamp_mv_row(int16_t mvec, int border);
        int16_t clamp_mv_col(int16_t mvec, int border);
        void clampMv();
        void generateRefAndNewMvContext(uint32_t CloseMatches, int numNew, uint32_t TotalMatches);
        void generateDrlCtxStack();
        void sort(int start, int end);
        void swap_stack(int i, int j);
        static bool has_newmv(PREDICTION_MODE mode);
        void searchStack(uint32_t mvRow, uint32_t mvCol, int candList, uint32_t weight);
        void searchCompoundStack(uint32_t mvRow, uint32_t mvCol, uint32_t weight);
        void add_ref_mv_candidate(uint32_t mvRow, uint32_t mvCol, uint32_t weight);
        void scanRow(int deltaRow);
        void scanCol(int deltaCol);
        void scanPoint(int deltaRow, int deltaCol);
        void lower_mv_precision(Mv& mv);
        void setupGlobalMV(uint8_t refList);

        void temporalScan();
        void extraSearch();
        void add_tpl_ref_mv(int deltaRow, int deltaCol);
        void add_extra_mv_candidate(int mvRow, int mvCol);

        const Block& m_block;
        const FrameHeader& m_frame;
        const Tile& m_tile;

        uint32_t WeightStack[MAX_REF_MV_STACK_SIZE];
        int NumMvFound = 0;
        int NewMvCount = 0;

        const uint32_t MiCol;
        const uint32_t MiRow;
        const uint32_t bw4;
        const uint32_t bh4;
        const bool isCompound;
        bool FoundMatch;
        std::vector<uint8_t> DrlCtxStack;
        uint8_t NewMvContext;
        uint8_t RefMvContext;
        uint8_t ZeroMvContext;
        std::vector<Mv> RefIdMvs[2];
        std::vector<Mv> RefDiffMvs[2];
    };

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
        }
    }
    int Wedge_Bits[BLOCK_SIZES_ALL] = {
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
        ASSERT(0);
        /*
        find_warp_samples();
        if ( force_integer_mv || NumSamples == 0 ||
            !allow_warped_motion || is_scaled( RefFrame[0] ) ) {
            use_obmc S()
            motion_mode = use_obmc ? OBMC : SIMPLE_TRANSLATION;
        } else {
            motion_mode
        }*/
    }

    void Block::read_interintra_mode(bool isCompound)
    {
        if (!skip_mode && m_sequence.enable_interintra_compound && !isCompound && MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32) {
            interintra = m_entropy.readInterIntra(MiSize);
            if (interintra) {
                INTERINTRA_MODE interintra_mode = m_entropy.readInterIntraMode(MiSize);
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
        compute_prediction(frame);
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

    void Block::FindMvStack::add_tpl_ref_mv(int deltaRow, int deltaCol)
    {
        int mvRow = (MiRow + deltaRow) | 1;
        int mvCol = (MiCol + deltaCol) | 1;
        if (!m_tile.is_inside(mvRow, mvCol))
            return;
        int x8 = mvCol >> 1;
        int y8 = mvRow >> 1;

        if (deltaRow == 0 && deltaCol == 0) {
            ZeroMvContext = 1;
        }
        if (!isCompound) {
            Mv candMv = m_frame.MotionFieldMvs[m_block.RefFrame[0]][y8][x8];
            if (candMv.mv[0] == -1 << 15)
                return;
            lower_mv_precision(candMv);
            if (deltaRow == 0 && deltaCol == 0) {
                if (std::abs(candMv.mv[0] - GlobalMvs[0].mv[0]) >= 16 || std::abs(candMv.mv[1] - GlobalMvs[0].mv[1]) >= 16)
                    ZeroMvContext = 1;
                else
                    ZeroMvContext = 0;
            }
            int idx;
            for (idx = 0; idx < NumMvFound; idx++) {
                if (candMv.mv[0] == RefStackMv[idx][0].mv[0] && candMv.mv[1] == RefStackMv[idx][0].mv[1])
                    break;
            }
            if (idx < NumMvFound) {
                WeightStack[idx] += 2;
            } else if (NumMvFound < MAX_REF_MV_STACK_SIZE) {
                RefStackMv[NumMvFound][0] = candMv;
                WeightStack[NumMvFound] = 2;
                NumMvFound += 1;
            }
        } else {
            Mv candMv0 = m_frame.MotionFieldMvs[m_block.RefFrame[0]][y8][x8];
            if (candMv0.mv[0] == -1 << 15)
                return;
            Mv candMv1 = m_frame.MotionFieldMvs[m_block.RefFrame[1]][y8][x8];
            if (candMv1.mv[0] == -1 << 15)
                return;
            lower_mv_precision(candMv0);
            lower_mv_precision(candMv1);
            if (deltaRow == 0 && deltaCol == 0) {
                if (std::abs(candMv0.mv[0] - GlobalMvs[0].mv[0]) >= 16 || std::abs(candMv0.mv[1] - GlobalMvs[0].mv[1]) >= 16 || std::abs(candMv1.mv[0] - GlobalMvs[1].mv[0]) >= 16 || std::abs(candMv1.mv[1] - GlobalMvs[1].mv[1]) >= 16)
                    ZeroMvContext = 1;
                else
                    ZeroMvContext = 0;
            }
            int idx;
            for (idx = 0; idx < NumMvFound; idx++) {
                if (candMv0.mv[0] == RefStackMv[idx][0].mv[0]
                    && candMv0.mv[1] == RefStackMv[idx][0].mv[1]
                    && candMv1.mv[0] == RefStackMv[idx][1].mv[0]
                    && candMv1.mv[1] == RefStackMv[idx][1].mv[1])
                    break;
            }
            if (idx < NumMvFound) {
                WeightStack[idx] += 2;
            } else if (NumMvFound < MAX_REF_MV_STACK_SIZE) {
                RefStackMv[NumMvFound][0] = candMv0;
                RefStackMv[NumMvFound][1] = candMv1;
                WeightStack[NumMvFound] = 2;
                NumMvFound += 1;
            }
        }
    }

    void Block::FindMvStack::temporalScan()
    {
        int stepW4 = (bw4 >= 16) ? 4 : 2;
        int stepH4 = (bh4 >= 16) ? 4 : 2;
        for (int deltaRow = 0; deltaRow < std::min((int)bh4, 16); deltaRow += stepH4) {
            for (int deltaCol = 0; deltaCol < std::min((int)bw4, 16); deltaCol += stepW4) {
                add_tpl_ref_mv(deltaRow, deltaCol);
            }
        }
    }

    void Block::FindMvStack::add_extra_mv_candidate(int mvRow, int mvCol)
    {
        if (isCompound) {
            for (int candList = 0; candList < 2; candList++) {
                int candRef = m_frame.RefFrames[mvRow][mvCol][candList];
                if (candRef > INTRA_FRAME) {
                    for (int list = 0; list < 2; list++) {
                        Mv candMv = m_frame.Mvs[mvRow][mvCol][candList];
                        if (candRef == m_block.RefFrame[list] && RefIdMvs[list].size() < 2) {
                            RefIdMvs[list].push_back(candMv);
                        } else if (RefDiffMvs[list].size() < 2) {
                            if (m_frame.RefFrameSignBias[candRef] != m_frame.RefFrameSignBias[m_block.RefFrame[list]]) {
                                candMv.mv[0] *= -1;
                                candMv.mv[1] *= -1;
                            }
                            RefDiffMvs[list].push_back(candMv);
                        }
                    }
                }
            }
        } else {
            for (int candList = 0; candList < 2; candList++) {
                int candRef = m_frame.RefFrames[mvRow][mvCol][candList];
                if (candRef > INTRA_FRAME) {
                    Mv candMv = m_frame.Mvs[mvRow][mvCol][candList];
                    if (m_frame.RefFrameSignBias[candRef] != m_frame.RefFrameSignBias[m_block.RefFrame[0]]) {
                        candMv.mv[0] *= -1;
                        candMv.mv[1] *= -1;
                    }
                    int idx;
                    for (idx = 0; idx < NumMvFound; idx++) {
                        if (candMv == RefStackMv[idx][0])
                            break;
                    }
                    if (idx == NumMvFound) {
                        RefStackMv[idx][0] = candMv;
                        WeightStack[idx] = 2;
                        NumMvFound++;
                    }
                }
            }
        }
    }
    void Block::FindMvStack::extraSearch()
    {

        for (int list = 0; list < 2; list++) {
            RefIdMvs[list].clear();
            RefDiffMvs[list].clear();
        }
        int w4 = std::min(16, (int)bw4);
        int h4 = std::min(16, (int)bh4);
        w4 = std::min(w4, (int)(m_frame.MiCols - MiCol));
        h4 = std::min(h4, (int)(m_frame.MiRows - MiRow));
        int num4x4 = std::min(w4, h4);
        for (int pass = 0; pass < 2; pass++) {
            int idx = 0;
            while (idx < num4x4 && NumMvFound < 2) {
                int mvRow, mvCol;
                if (pass == 0) {
                    mvRow = MiRow - 1;
                    mvCol = MiCol + idx;
                } else {
                    mvRow = MiRow + idx;
                    mvCol = MiCol - 1;
                }
                if (!m_tile.is_inside(mvRow, mvCol))
                    break;
                add_extra_mv_candidate(mvRow, mvCol);
                if (pass == 0) {
                    idx += Num_4x4_Blocks_Wide[m_frame.MiSizes[mvRow][mvCol]];
                } else {
                    idx += Num_4x4_Blocks_High[m_frame.MiSizes[mvRow][mvCol]];
                }
            }
        }
        if (isCompound) {
            std::vector<Mv> combinedMvs[2];
            for (int list = 0; list < 2; list++) {
                for (size_t idx = 0; idx < RefIdMvs[list].size(); idx++) {
                    combinedMvs[list].push_back(RefIdMvs[list][idx]);
                }
                for (int idx = 0; idx < RefDiffMvs[list].size() && combinedMvs[list].size() < 2; idx++) {
                    combinedMvs[list].push_back(RefDiffMvs[list][idx]);
                }
                while (combinedMvs[list].size() < 2) {
                    combinedMvs[list].push_back(GlobalMvs[list]);
                }
            }
            if (NumMvFound == 1) {
                if (combinedMvs[0][0] == RefStackMv[0][0] && combinedMvs[1][0] == RefStackMv[0][1]) {
                    RefStackMv[NumMvFound][0] = combinedMvs[0][1];
                    RefStackMv[NumMvFound][1] = combinedMvs[1][1];
                } else {
                    RefStackMv[NumMvFound][0] = combinedMvs[0][0];
                    RefStackMv[NumMvFound][1] = combinedMvs[1][0];
                }
                WeightStack[NumMvFound] = 2;
                NumMvFound++;
            } else {
                for (int idx = 0; idx < 2; idx++) {
                    RefStackMv[NumMvFound][0] = combinedMvs[0][idx];
                    RefStackMv[NumMvFound][1] = combinedMvs[1][idx];
                    WeightStack[NumMvFound] = 2;
                    NumMvFound++;
                }
            }
        }
    }
    void Block::FindMvStack::find_mv_stack()
    {
        setupGlobalMV(0);
        if (isCompound)
            setupGlobalMV(1);
        scanRow(-1);
        bool foundAboveMatch = FoundMatch;

        FoundMatch = false;
        scanCol(-1);
        bool foundLeftMatch = FoundMatch;

        if (std::max(bw4, bh4) <= 16) {
            FoundMatch = false;
            scanPoint(-1, bw4);
            if (FoundMatch) {
                foundAboveMatch = true;
            }
        }
        uint32_t CloseMatches = foundAboveMatch + foundLeftMatch;
        int numNearest = NumMvFound;
        int numNew = NewMvCount;
        if (numNearest > 0) {
            for (int i = 0; i < numNearest; i++) {
                WeightStack[i] += REF_CAT_LEVEL;
            }
        }
        ZeroMvContext = 0;
        if (m_frame.use_ref_frame_mvs) {
            temporalScan();
        }
        FoundMatch = false;
        scanPoint(-1, -1);
        if (FoundMatch) {
            foundAboveMatch = true;
            FoundMatch = false;
        }
        scanRow(-3);
        if (FoundMatch) {
            foundAboveMatch = true;
            FoundMatch = false;
        }
        scanCol(-3);
        if (FoundMatch) {
            foundLeftMatch = true;
            FoundMatch = false;
        }
        if (bh4 > 1) {
            scanRow(-5);
            if (FoundMatch) {
                foundAboveMatch = true;
                FoundMatch = false;
            }
        }
        if (bw4 > 1) {
            scanCol(-5);
            if (FoundMatch) {
                foundLeftMatch = true;
                FoundMatch = false;
            }
        }
        uint32_t TotalMatches = foundAboveMatch + foundLeftMatch;
        sort(0, numNearest);
        sort(numNearest, NumMvFound);
        if (NumMvFound < 2) {
            extraSearch();
        }
        generateDrlCtxStack();
        generateRefAndNewMvContext(CloseMatches, numNew, TotalMatches);
        clampMv();
    }

    uint8_t Block::FindMvStack::getCompoundModeCtx() const
    {
        constexpr static uint8_t Compound_Mode_Ctx_Map[3][COMP_NEWMV_CTXS] = {
            { 0, 1, 1, 1, 1 },
            { 1, 2, 3, 4, 4 },
            { 4, 4, 5, 6, 7 }
        };
        return Compound_Mode_Ctx_Map[RefMvContext >> 1][std::min(NewMvContext, (uint8_t)(COMP_NEWMV_CTXS - 1))];
    }
    void Block::FindMvStack::setupGlobalMV(uint8_t refList)
    {
        Mv& mv = GlobalMvs[refList];
        GlobalMotionType typ;
        uint8_t ref = m_block.RefFrame[refList];
        if (ref != INTRA_FRAME)
            typ = IDENTITY;
        if (ref == INTRA_FRAME || typ == IDENTITY) {
            mv.mv[0] = mv.mv[1] = 0;
        } else if (typ == TRANSLATION) {
            mv.mv[0] = m_frame.gm_params[ref][0] >> (WARPEDMODEL_PREC_BITS - 3);
            mv.mv[1] = m_frame.gm_params[ref][1] >> (WARPEDMODEL_PREC_BITS - 3);
        } else {
            int x = MiCol * MI_SIZE + m_block.bw4 * 2 - 1;
            int y = MiRow * MI_SIZE + m_block.bh4 * 2 - 1;
            int xc = (m_frame.gm_params[ref][2] - (1 << WARPEDMODEL_PREC_BITS)) * x + m_frame.gm_params[ref][3] * y + m_frame.gm_params[ref][0];
            int yc = m_frame.gm_params[ref][4] * x + (m_frame.gm_params[ref][5] - (1 << WARPEDMODEL_PREC_BITS)) * y + m_frame.gm_params[ref][1];
            if (m_frame.allow_high_precision_mv) {
                mv.mv[0] = ROUND2SIGNED(yc, WARPEDMODEL_PREC_BITS - 3);
                mv.mv[1] = ROUND2SIGNED(xc, WARPEDMODEL_PREC_BITS - 3);
            } else {
                mv.mv[0] = ROUND2SIGNED(yc, WARPEDMODEL_PREC_BITS - 2) * 2;
                mv.mv[1] = ROUND2SIGNED(xc, WARPEDMODEL_PREC_BITS - 2) * 2;
            }
        }
        lower_mv_precision(mv);
    }
    void Block::FindMvStack::lower_mv_precision(Mv& mv)
    {
        int16_t* candMv = &mv.mv[0];
        for (int i = 0; i < 2; i++) {
            if (m_frame.force_integer_mv) {
                int a = std::abs(candMv[i]);
                int aInt = (a + 3) >> 3;
                if (candMv[i] > 0)
                    candMv[i] = aInt << 3;
                else
                    candMv[i] = -(aInt << 3);
            } else {
                if (candMv[i] & 1) {
                    if (candMv[i] > 0)
                        candMv[i]--;
                    else
                        candMv[i]++;
                }
            }
        }
    }
    uint8_t Block::FindMvStack::getNewMvCtx() const
    {
        return NewMvContext;
    }
    void Block::FindMvStack::scanCol(int deltaCol)
    {
        int deltaRow = 0;
        uint32_t end4 = std::min(std::min(bh4, m_frame.MiRows - m_block.MiRow), (uint32_t)16);
        bool useStep16 = (bh4 >= 16);
        if (std::abs(deltaCol)) {
            deltaRow = 1 - (MiRow & 1);
            deltaCol += MiCol & 1;
        }
        uint32_t i = 0;
        while (i < end4) {
            uint32_t mvRow = MiRow + deltaRow + i;
            uint32_t mvCol = MiCol + deltaCol;
            if (!m_tile.is_inside(mvRow, mvCol))
                break;
            int len = std::min((int)bh4, Num_4x4_Blocks_High[m_frame.MiSizes[mvRow][mvCol]]);
            if (std::abs(deltaCol) > 1)
                len = std::max(2, len);
            if (useStep16)
                len = std::max(4, len);
            uint32_t weight = len * 2;
            add_ref_mv_candidate(mvRow, mvCol, weight);
            i += len;
        }
    }
    uint8_t Block::FindMvStack::getZeroMvCtx() const
    {
        return ZeroMvContext;
    }
    void Block::FindMvStack::searchStack(uint32_t mvRow, uint32_t mvCol, int candList, uint32_t weight)
    {
        Mv candMv;
        PREDICTION_MODE candMode = m_frame.YModes[mvRow][mvCol];
        BLOCK_SIZE candSize = m_frame.MiSizes[mvRow][mvCol];
        bool large = std::min(Block_Width[candSize], Block_Height[candSize]) >= 8;
        if ((candMode == GLOBALMV || candMode == GLOBAL_GLOBALMV)
            && m_frame.GmType[m_block.RefFrame[0]] > TRANSLATION
            && large) {
            candMv = GlobalMvs[0];
        } else {
            candMv = m_frame.Mvs[mvRow][mvCol][candList];
        }
        lower_mv_precision(candMv);
        if (has_newmv(candMode))
            NewMvCount++;
        FoundMatch = true;
        int idx;
        for (idx = 0; idx < NumMvFound; idx++) {
            if (candMv == RefStackMv[idx][0]) {
                break;
            }
        }
        if (idx < NumMvFound) {
            WeightStack[idx] += weight;
        } else if (idx < MAX_REF_MV_STACK_SIZE) {
            RefStackMv[NumMvFound][0] = candMv;
            WeightStack[NumMvFound] = weight;
            NumMvFound++;
        }
    }
    int16_t Block::FindMvStack::clamp_mv_row(int16_t mvec, int border)
    {
        int mbToTopEdge = -(((int)MiRow * MI_SIZE) * 8);
        int mbToBottomEdge = (((int)m_frame.MiRows - bh4 - MiRow) * MI_SIZE) * 8;
        return CLIP3(mbToTopEdge - border, mbToBottomEdge + border, mvec);
    }
    int16_t Block::FindMvStack::clamp_mv_col(int16_t mvec, int border)
    {
        int mbToLeftEdge = -(int)((MiCol * MI_SIZE) * 8);
        int mbToRightEdge = ((m_frame.MiCols - bw4 - MiCol) * MI_SIZE) * 8;
        return CLIP3(mbToLeftEdge - border, mbToRightEdge + border, mvec);
    }
    uint8_t Block::FindMvStack::getRefMvCtx() const
    {
        return RefMvContext;
    }
    void Block::FindMvStack::clampMv()
    {
        for (int list = 0; list < 1 + isCompound; list++) {
            for (int idx = 0; idx < NumMvFound; idx++) {
                Mv refMv = RefStackMv[idx][list];
                refMv.mv[0] = clamp_mv_row(refMv.mv[0], MV_BORDER + bh4 * 4 * 8);
                refMv.mv[1] = clamp_mv_col(refMv.mv[1], MV_BORDER + bw4 * 4 * 8);
                RefStackMv[idx][list] = refMv;
            }
        }
    }
    void Block::FindMvStack::generateRefAndNewMvContext(uint32_t CloseMatches, int numNew, uint32_t TotalMatches)
    {
        if (CloseMatches == 0) {
            NewMvContext = std::min((int)TotalMatches, 1); // 0,1
            RefMvContext = TotalMatches;
        } else if (CloseMatches == 1) {
            NewMvContext = 3 - std::min(numNew, 1); // 2,3
            RefMvContext = 2 + TotalMatches;
        } else {
            NewMvContext = 5 - std::min(numNew, 1); // 4,5
            RefMvContext = 5;
        }
    }
    void Block::FindMvStack::scanPoint(int deltaRow, int deltaCol)
    {
        uint32_t mvRow = MiRow + deltaRow;
        uint32_t mvCol = MiCol + deltaCol;
        uint32_t weight = 4;
        if (m_tile.is_inside(mvRow, mvCol)
            && m_frame.RefFrames[mvRow][mvCol][0] != NONE_FRAME) {
            add_ref_mv_candidate(mvRow, mvCol, weight);
        }
    }
    void Block::FindMvStack::generateDrlCtxStack()
    {
        DrlCtxStack.resize(NumMvFound);
        for (int idx = 0; idx < NumMvFound; idx++) {
            uint8_t z = 0;
            if (idx + 1 < NumMvFound) {
                uint32_t w0 = WeightStack[idx];
                uint32_t w1 = WeightStack[idx + 1];
                if (w0 >= REF_CAT_LEVEL) {
                    if (w1 < REF_CAT_LEVEL) {
                        z = 1;
                    }
                } else {
                    z = 2;
                }
            }
            DrlCtxStack[idx] = z;
        }
    }
    void Block::FindMvStack::sort(int start, int end)
    {
        while (end > start) {
            int newEnd = start;
            for (int idx = start + 1; idx < end; idx++) {
                if (WeightStack[idx - 1] < WeightStack[idx]) {
                    swap_stack(idx - 1, idx);
                    newEnd = idx;
                }
            }
            end = newEnd;
        }
    }
    void Block::FindMvStack::swap_stack(int i, int j)
    {
        std::swap(WeightStack[i], WeightStack[j]);

        for (int list = 0; list < 1 + isCompound; list++) {
            std::swap(RefStackMv[i][list], RefStackMv[j][list]);
        }
    }
    bool Block::FindMvStack::has_newmv(PREDICTION_MODE mode)
    {
        return (mode == NEWMV
            || mode == NEW_NEWMV
            || mode == NEAR_NEWMV
            || mode == NEW_NEARMV
            || mode == NEAREST_NEWMV
            || mode == NEW_NEARESTMV);
    }
    void Block::FindMvStack::searchCompoundStack(uint32_t mvRow, uint32_t mvCol, uint32_t weight)
    {
        std::vector<Mv> candMvs = m_frame.Mvs[mvRow][mvCol];
        PREDICTION_MODE candMode = m_frame.YModes[mvRow][mvCol];
        BLOCK_SIZE candSize = m_frame.MiSizes[mvRow][mvCol];
        if (candMode == GLOBAL_GLOBALMV) {
            for (int i = 0; i <= 1; i++) {
                if (m_frame.GmType[m_block.RefFrame[i]] > TRANSLATION) {
                    candMvs[i] = GlobalMvs[i];
                }
            }
        }
        lower_mv_precision(candMvs[0]);
        lower_mv_precision(candMvs[1]);
        if (has_newmv(candMode))
            NewMvCount++;

        FoundMatch = true;
        int idx;
        for (idx = 0; idx < NumMvFound; idx++) {
            if (candMvs[0] == RefStackMv[idx][0]
                && candMvs[1] == RefStackMv[idx][1]) {
                break;
            }
        }
        if (idx < NumMvFound) {
            WeightStack[idx] += weight;
        } else if (idx < MAX_REF_MV_STACK_SIZE) {
            RefStackMv[NumMvFound][0] = candMvs[0];
            RefStackMv[NumMvFound][1] = candMvs[1];
            NumMvFound++;
        }
    }
    void Block::FindMvStack::add_ref_mv_candidate(uint32_t mvRow, uint32_t mvCol, uint32_t weight)
    {
        if (!m_frame.IsInters[mvRow][mvCol])
            return;
        if (!isCompound) {
            for (int candList = 0; candList <= 0; candList++) {
                if (m_frame.RefFrames[mvRow][mvCol][candList] == m_block.RefFrame[0]) {
                    searchStack(mvRow, mvCol, candList, weight);
                }
            }
        } else {
            if (m_frame.RefFrames[mvRow][mvCol][0] == m_block.RefFrame[0]
                || m_frame.RefFrames[mvRow][mvCol][1] == m_block.RefFrame[1]) {
                searchCompoundStack(mvRow, mvCol, weight);
            }
        }
    }
    void Block::FindMvStack::scanRow(int deltaRow)
    {
        int deltaCol = 0;
        uint32_t end4 = std::min(std::min(bw4, m_frame.MiCols - m_block.MiCol), (uint32_t)16);
        bool useStep16 = (bw4 >= 16);
        if (std::abs(deltaRow)) {
            deltaRow += MiRow & 1;
            deltaCol = 1 - (MiCol & 1);
        }
        uint32_t i = 0;
        while (i < end4) {
            uint32_t mvRow = MiRow + deltaRow;
            uint32_t mvCol = MiCol + deltaCol + i;
            if (!m_tile.is_inside(mvRow, mvCol))
                break;
            int len = std::min((int)bw4, Num_4x4_Blocks_Wide[m_frame.MiSizes[mvRow][mvCol]]);
            if (std::abs(deltaRow) > 1)
                len = std::max(2, len);
            if (useStep16)
                len = std::max(4, len);
            uint32_t weight = len * 2;
            add_ref_mv_candidate(mvRow, mvCol, weight);
            i += len;
        }
    }

    Block::FindMvStack::FindMvStack(Block& block)
        : m_block(block)
        , m_tile(block.m_tile)
        , m_frame(block.m_frame)
        , MiRow(block.MiRow)
        , MiCol(block.MiCol)
        , bw4(block.bw4)
        , bh4(block.bh4)
        , isCompound(block.RefFrame[1] > INTRA_FRAME)
    {
    }

    uint8_t Block::FindMvStack::getDrlModeCtx(uint8_t idx) const
    {
        return DrlCtxStack[idx];
    }

    int Block::FindMvStack::getNumMvFound() const
    {
        return NumMvFound;
    }

}
}
