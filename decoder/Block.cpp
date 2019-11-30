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
        bool isCompund = RefFrame[1] > INTRA_FRAME;

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
                if (((r + y) >= 288) || ((c + x) >= 1920 / 4))
                    printf("done");
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
        return ((refFrame >= BWDREF_FRAME) && (refFrame <= ALTREF_FRAME ));
    }
    uint8_t Block::getCompModeCtx()
    {
        uint8_t ctx;
        if ( AvailU && AvailL ) {
            if ( AboveSingle && LeftSingle )
                ctx = check_backward( AboveRefFrame[ 0 ] )
                    ^ check_backward( LeftRefFrame[ 0 ] );
            else if ( AboveSingle )
                ctx = 2 + ( check_backward( AboveRefFrame[ 0 ] ) || AboveIntra);
            else if ( LeftSingle )
                ctx = 2 + ( check_backward( LeftRefFrame[ 0 ] ) || LeftIntra);
            else
                ctx = 4;
        } else if ( AvailU ) {
            if ( AboveSingle )
                ctx = check_backward( AboveRefFrame[ 0 ] );
            else
                ctx = 3;
        } else if ( AvailL ) {
            if ( LeftSingle )
                ctx= check_backward( LeftRefFrame[ 0 ] );
            else
                ctx = 3;
        } else {
            ctx = 1;
        };
        return ctx;
    }
    static bool is_samedir_ref_pair(uint8_t ref0, uint8_t ref1) {
        return (ref0 >= BWDREF_FRAME) == (ref1 >= BWDREF_FRAME);
    }
    uint8_t Block::getCompReferenceTypeCtx()
    {
        uint8_t ctx;
        uint8_t above0 = AboveRefFrame[ 0 ];
        uint8_t above1 = AboveRefFrame[ 1 ];
        uint8_t left0 = LeftRefFrame[ 0 ];
        uint8_t left1 = LeftRefFrame[ 1 ];
        bool aboveCompInter = AvailU && !AboveIntra && !AboveSingle;
        bool leftCompInter = AvailL && !LeftIntra && !LeftSingle;
        bool aboveUniComp = aboveCompInter && is_samedir_ref_pair(above0, above1);
        bool leftUniComp = leftCompInter && is_samedir_ref_pair(left0, left1);
        if ( AvailU && !AboveIntra && AvailL && !LeftIntra ) {
            bool samedir = is_samedir_ref_pair(above0, left0);
            if ( !aboveCompInter && !leftCompInter ) {
                ctx = 1 + 2 * samedir;
            } else if ( !aboveCompInter ) {
                if ( !leftUniComp )
                    ctx = 1;
                else
                    ctx = 3 + samedir;
            } else if ( !leftCompInter ) {
                if ( !aboveUniComp )
                    ctx = 1;
                else
                    ctx = 3 + samedir;
            } else {
                if ( !aboveUniComp && !leftUniComp )
                    ctx = 0;
                else if ( !aboveUniComp || !leftUniComp )
                    ctx = 2;
                else
                    ctx = 3 + ((above0 == BWDREF_FRAME) == (left0 == BWDREF_FRAME));
            }
        } else if ( AvailU && AvailL ) {
            if ( aboveCompInter )
                ctx = 1 + 2 * aboveUniComp;
            else if ( leftCompInter )
                ctx = 1 + 2 * leftUniComp;
            else
                ctx = 2;
        } else if ( aboveCompInter ) {
            ctx = 4 * aboveUniComp;
        } else if ( leftCompInter ) {
            ctx = 4 * leftUniComp;
        } else {
            ctx = 2;
        }
        return ctx;
    }
    uint8_t Block::count_refs(uint8_t frameType) {
        uint8_t c = 0;
        if ( AvailU ) {
            if ( AboveRefFrame[ 0 ] == frameType ) c++;
            if ( AboveRefFrame[ 1 ] == frameType ) c++;
        }
        if ( AvailL ) {
            if ( LeftRefFrame[ 0 ] == frameType ) c++;
            if ( LeftRefFrame[ 1 ] == frameType ) c++;
        }
        return c;
    }

    uint8_t Block::ref_count_ctx(uint8_t counts0, uint8_t counts1)
    {
        if ( counts0 < counts1 )
            return 0;
        else if ( counts0 == counts1 )
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
        uint8_t last2Count = count_refs( LAST2_FRAME );
        uint8_t last3GoldCount = count_refs( LAST3_FRAME ) + count_refs( GOLDEN_FRAME );
        uint8_t ctx = ref_count_ctx( last2Count, last3GoldCount );
        return ctx;
    }

    uint8_t Block::getUniCompRefP2Ctx()
    {
        return getCompRefP2Ctx();
    }

    uint8_t Block::getCompRefCtx()
    {
        uint8_t last12Count = count_refs( LAST_FRAME ) + count_refs( LAST2_FRAME );
        uint8_t last3GoldCount = count_refs( LAST3_FRAME ) + count_refs( GOLDEN_FRAME );
        uint8_t ctx = ref_count_ctx( last12Count, last3GoldCount );
        return ctx;
    }

    uint8_t Block::getCompRefP1Ctx()
    {
        uint8_t lastCount = count_refs( LAST_FRAME );
        uint8_t last2Count = count_refs( LAST2_FRAME );
        uint8_t ctx = ref_count_ctx( lastCount, last2Count );
        return ctx;
    }

    uint8_t Block::getCompRefP2Ctx()
    {
        uint8_t last3Count = count_refs( LAST3_FRAME );
        uint8_t goldCount = count_refs( GOLDEN_FRAME );
        uint8_t ctx = ref_count_ctx( last3Count, goldCount);
        return ctx;
    }

    uint8_t Block::getCompBwdRefCtx()
    {
        uint8_t brfarf2Count = count_refs( BWDREF_FRAME ) + count_refs( ALTREF2_FRAME );
        uint8_t arfCount = count_refs( ALTREF_FRAME );
        uint8_t ctx = ref_count_ctx( brfarf2Count, arfCount );
        return ctx;
    }

    uint8_t Block::getCompBwdRefP1Ctx()
    {
        uint8_t brfCount = count_refs( BWDREF_FRAME );
        uint8_t arf2Count = count_refs( ALTREF2_FRAME );
        uint8_t ctx = ref_count_ctx( brfCount, arf2Count );
        return ctx;
    }

    void Block::readCompReference()
    {
        COMP_REFERENCE_TYPE comp_ref_type = m_entropy.readCompReferenceType(getCompReferenceTypeCtx());
        if ( comp_ref_type == UNIDIR_COMP_REFERENCE ) {
            bool uni_comp_ref = m_entropy.readUniCompRef(getUniCompRefCtx());
            if ( uni_comp_ref ) {
                RefFrame[0] = BWDREF_FRAME;
                RefFrame[1] = ALTREF_FRAME;
            } else {
                bool uni_comp_ref_p1 = m_entropy.readUniCompRefP1(getUniCompRefP1Ctx());
                if ( uni_comp_ref_p1 ) {
                    bool uni_comp_ref_p2 = m_entropy.readUniCompRefP2(getUniCompRefP2Ctx());
                    if ( uni_comp_ref_p2 ) {
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
            if (!comp_ref ) {
                bool comp_ref_p1 = m_entropy.readCompRefP1(getCompRefP1Ctx());
                RefFrame[ 0 ] = comp_ref_p1 ? LAST2_FRAME : LAST_FRAME;
            } else {
                bool comp_ref_p2 = m_entropy.readCompRefP2(getCompRefP2Ctx());
                RefFrame[ 0 ] = comp_ref_p2 ? GOLDEN_FRAME : LAST3_FRAME;
            }
            bool comp_bwdref =  m_entropy.readCompBwdRef(getCompBwdRefCtx());
            if (!comp_bwdref) {
                bool comp_bwdref_p1 = m_entropy.readCompBwdRefP1(getCompBwdRefP1Ctx());
                RefFrame[ 1 ] = comp_bwdref_p1 ? ALTREF2_FRAME : BWDREF_FRAME;
            } else {
                RefFrame[ 1 ] = ALTREF_FRAME;
            }
        }

    }

    uint8_t Block::getSingleRefP1Ctx()
    {
        uint8_t fwdCount, bwdCount;
        fwdCount = count_refs(LAST_FRAME );
        fwdCount += count_refs(LAST2_FRAME );
        fwdCount += count_refs(LAST3_FRAME );
        fwdCount += count_refs(GOLDEN_FRAME );
        bwdCount = count_refs(BWDREF_FRAME );
        bwdCount += count_refs(ALTREF2_FRAME);
        bwdCount += count_refs(ALTREF_FRAME);
        uint8_t ctx = ref_count_ctx( fwdCount, bwdCount );
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
                RefFrame[ 0 ] = ALTREF_FRAME;
            }
        } else {
            bool single_ref_p3 = m_entropy.readSingleRef(getSingleRefP3Ctx(), 3);
            if (single_ref_p3) {
                bool single_ref_p5 = m_entropy.readSingleRef(getSingleRefP5Ctx(), 5);
                RefFrame[ 0 ] = single_ref_p5 ? GOLDEN_FRAME : LAST3_FRAME;
            } else {
                bool single_ref_p4 = m_entropy.readSingleRef(getSingleRefP4Ctx(), 4);
                RefFrame[ 0 ] = single_ref_p4 ? LAST2_FRAME : LAST_FRAME;
            }
        }
        RefFrame[ 1 ] = NONE_FRAME;
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
            if (m_frame.reference_select && (std::min( bw4, bh4 ) >= 2 ) ) {
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
        FindMvStack(Block &block)
            : m_block(block)
            , m_tile(block.m_tile)
            , m_frame(block.m_frame)
            , MiRow(block.MiRow)
            , MiCol(block.MiCol)
        {


        }
        void find_mv_stack(bool isCompound)
        {
            setupGlobalMV(0);
            if (isCompound)
                setupGlobalMV(1);
            FoundMatch = false;


        }
    private:
        static bool has_newmv(PREDICTION_MODE mode )
        {
            return (mode == NEWMV
                || mode == NEW_NEWMV
                || mode == NEAR_NEWMV
                || mode == NEW_NEARMV
                || mode == NEAREST_NEWMV
                || mode == NEW_NEARESTMV);
        }
        void searchStack(uint32_t mvRow, uint32_t mvCol, int candList, uint32_t weight)
        {
            Mv candMv;
            PREDICTION_MODE candMode = m_frame.YModes[mvRow][mvCol];
            BLOCK_SIZE candSize = m_frame.MiSizes[mvRow][mvCol];
            bool large = std::min( Block_Width[ candSize ],Block_Height[ candSize ] ) >= 8;
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
            } else if (idx < MAX_REF_MV_STACK_SIZE){
                RefStackMv[NumMvFound][0] = candMv;
                WeightStack[NumMvFound] = weight;
                NumMvFound++;
            }
        }
        void searchCompoundStack(uint32_t mvRow, uint32_t mvCol, uint32_t weight)
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
        void add_ref_mv_candidate(uint32_t mvRow, uint32_t mvCol, bool isCompound, uint32_t weight)
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
        void scanRow(int deltaRow, bool isCompound)
        {
            int deltaCol = 0;
            uint32_t bw4 = m_block.bw4;
            uint32_t end4 = std::min(std::min(bw4, m_frame.MiCols - m_block.MiCol ), (uint32_t)16);
            bool useStep16 = (bw4 >= 16);
            if (std::abs(deltaRow)) {
                deltaRow += MiRow & 1;
                deltaCol = 1 - (MiCol & 1);
            }
            uint32_t i = 0;
            while ( i < end4 ) {
                uint32_t mvRow = MiRow + deltaRow;
                uint32_t mvCol = MiCol + deltaCol + i;
                if ( !m_tile.is_inside(mvRow,mvCol) )
                    break;
                int len = std::min((int)bw4, Num_4x4_Blocks_Wide[ m_frame.MiSizes[ mvRow ][ mvCol ] ]);
                if ( std::abs(deltaRow) > 1 )
                    len = std::max(2, len);
                if ( useStep16 )
                    len =std::max(4, len);
                uint32_t weight = len * 2;
                add_ref_mv_candidate( mvRow, mvCol, isCompound, weight);
                i += len;
            }


        }
        void scanCol(int deltaCol, bool isCompound)
        {
            //scanLine(0, deltaCol, isCompound);
        }
        void lower_mv_precision(Mv& mv)
        {
            int16_t* candMv = &mv.mv[0];
            for (int i = 0; i < 2; i++) {
                if (m_frame.force_integer_mv ) {
                    int a = std::abs( candMv[ i ] );
                    int aInt = (a + 3) >> 3;
                    if ( candMv[ i ] > 0 )
                        candMv[ i ] = aInt << 3;
                    else
                        candMv[ i ] = -( aInt << 3 );
                } else {
                    if ( candMv[ i ] & 1 ) {
                        if ( candMv[ i ] > 0 )
                            candMv[ i ]--;
                        else
                            candMv[ i ]++;
                    }
                }
            }

        }
        void setupGlobalMV(uint8_t refList)
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
                int xc = (m_frame.gm_params[ref][2] - (1 << WARPEDMODEL_PREC_BITS)) * x +
                    m_frame.gm_params[ref][3] * y +
                    m_frame.gm_params[ref][0];
                int yc = m_frame.gm_params[ref][4] * x +
                    (m_frame.gm_params[ref][5] - (1 << WARPEDMODEL_PREC_BITS)) * y +
                    m_frame.gm_params[ref][1];
                if (m_frame.allow_high_precision_mv ) {
                    mv.mv[0] = ROUND2SIGNED(yc, WARPEDMODEL_PREC_BITS - 3);
                    mv.mv[1] = ROUND2SIGNED(xc, WARPEDMODEL_PREC_BITS - 3);
                } else {
                    mv.mv[0] = ROUND2SIGNED(yc, WARPEDMODEL_PREC_BITS - 2) * 2;
                    mv.mv[1] = ROUND2SIGNED(xc, WARPEDMODEL_PREC_BITS - 2) * 2;
                }
            }
            lower_mv_precision(mv);
        }
        Block &m_block;
        FrameHeader &m_frame;
        Tile &m_tile;
        Mv RefStackMv[MAX_REF_MV_STACK_SIZE][2];
        uint32_t WeightStack[MAX_REF_MV_STACK_SIZE];
        int NumMvFound = 0;
        int NewMvCount = 0;
        Mv GlobalMvs[2];
        uint32_t MiCol;
        uint32_t MiRow;
        bool FoundMatch;
    };

    void Block::find_mv_stack(bool isCompound)
    {
        FindMvStack find(*this);
        find.find_mv_stack(isCompound);
    }

    void Block::inter_block_mode_info()
    {
        PaletteSizeY = 0;
        PaletteSizeUV = 0;
        read_ref_frames();
        bool isCompound = RefFrame[ 1 ] > INTRA_FRAME;
        find_mv_stack(isCompound);
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
        for (auto& t : m_transformBlocks) {
            if (!t->decode(frame))
                return false;
        }
        return true;
    }
}
}
