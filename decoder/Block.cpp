#include "Block.h"
#include "Av1Common.h"
#include "Av1Parser.h"
#include "Av1Tile.h"
#include "SymbolDecoder.h"
#include "TransformBlock.h"
#include "VideoFrame.h"
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

    class Block::PredictInter {
    public:
        PredictInter(Block& block, YuvFrame& yuv, const FrameStore& frameStore);
        void predict_inter(int plane, int x, int y, uint32_t w, uint32_t h, int candRow, int candCol);

    private:
        uint8_t getUseWarp(int x, int y, int refFrame);
        void motionVectorScaling(uint8_t refIdx, int plane, int x, int y, const Mv& mv);
        int getFilterIdx(int size, int candRow, int candCol, int dir);
        void blockInterPrediction(uint8_t refIdx, int refList, int plane, uint32_t w, uint32_t h, int candRow, int candCol);
        void blockWarp(int useWarp, int plane, uint8_t refIdx, int refList, int x, int y, int i8, int j8, int w, int h);
        const Block& m_block;
        const FrameHeader& m_frame;
        const SequenceHeader& m_sequence;
        LocalWarp& m_localWarp;
        YuvFrame& m_yuv;
        const FrameStore& m_frameStore;
        bool isCompound;
        int InterRound0;
        int InterRound1;
        int InterPostRound;
        bool globaValid = false;

        int startX;
        int startY;
        int xStep;
        int yStep;
        std::vector<std::vector<uint8_t>> preds[2];
    };

    Block::PredictInter::PredictInter(Block& block, YuvFrame& yuv, const FrameStore& frameStore)
        : m_localWarp(block.m_localWarp)
        , m_block(block)
        , m_frame(block.m_frame)
        , m_sequence(block.m_sequence)
        , m_yuv(yuv)
        , m_frameStore(frameStore)
    {
    }

    uint8_t Block::PredictInter::getUseWarp(int w, int h, int refFrame)
    {
        if (w < 8 || h < 8)
            return false;
        if (m_frame.force_integer_mv)
            return false;
        if (m_block.motion_mode == LOCALWARP && m_localWarp.LocalValid)
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

        const static int halfSample = (1 << (SUBPEL_BITS - 1));
        uint8_t subX = plane ? m_sequence.subsampling_x : 0;
        uint8_t subY = plane ? m_sequence.subsampling_y : 0;
        int origX = ((x << SUBPEL_BITS) + ((2 * mv.mv[1]) >> subX) + halfSample);
        int origY = ((y << SUBPEL_BITS) + ((2 * mv.mv[0]) >> subY) + halfSample);
        int baseX = (origX * xScale - (halfSample << REF_SCALE_SHIFT));
        int baseY = (origY * yScale - (halfSample << REF_SCALE_SHIFT));

        const static int off = ((1 << (SCALE_SUBPEL_BITS - SUBPEL_BITS)) / 2);

        startX = (ROUND2SIGNED(baseX, REF_SCALE_SHIFT + SUBPEL_BITS - SCALE_SUBPEL_BITS) + off);
        startY = (ROUND2SIGNED(baseY, REF_SCALE_SHIFT + SUBPEL_BITS - SCALE_SUBPEL_BITS) + off);
        xStep = ROUND2SIGNED(xScale, REF_SCALE_SHIFT - SCALE_SUBPEL_BITS);
        yStep = ROUND2SIGNED(yScale, REF_SCALE_SHIFT - SCALE_SUBPEL_BITS);
    }

    inline int Block::PredictInter::getFilterIdx(int size, int candRow, int candCol, int dir)
    {
        InterpFilter interpFilter = m_frame.InterpFilters[candRow][candCol][dir];
        int idx = (int)interpFilter;
        if (size <= 4) {
            if (interpFilter == EIGHTTAP || interpFilter == EIGHTTAP_SHARP) {
                idx = 4;
            } else if (interpFilter == EIGHTTAP_SMOOTH) {
                idx = 5;
            }
        }
        return idx;
    }

    const static int Subpel_Filters[6][16][8] = {
        { { 0, 0, 0, 128, 0, 0, 0, 0 },
            { 0, 2, -6, 126, 8, -2, 0, 0 },
            { 0, 2, -10, 122, 18, -4, 0, 0 },
            { 0, 2, -12, 116, 28, -8, 2, 0 },
            { 0, 2, -14, 110, 38, -10, 2, 0 },
            { 0, 2, -14, 102, 48, -12, 2, 0 },
            { 0, 2, -16, 94, 58, -12, 2, 0 },
            { 0, 2, -14, 84, 66, -12, 2, 0 },
            { 0, 2, -14, 76, 76, -14, 2, 0 },
            { 0, 2, -12, 66, 84, -14, 2, 0 },
            { 0, 2, -12, 58, 94, -16, 2, 0 },
            { 0, 2, -12, 48, 102, -14, 2, 0 },
            { 0, 2, -10, 38, 110, -14, 2, 0 },
            { 0, 2, -8, 28, 116, -12, 2, 0 },
            { 0, 0, -4, 18, 122, -10, 2, 0 },
            { 0, 0, -2, 8, 126, -6, 2, 0 } },
        { { 0, 0, 0, 128, 0, 0, 0, 0 },
            { 0, 2, 28, 62, 34, 2, 0, 0 },
            { 0, 0, 26, 62, 36, 4, 0, 0 },
            { 0, 0, 22, 62, 40, 4, 0, 0 },
            { 0, 0, 20, 60, 42, 6, 0, 0 },
            { 0, 0, 18, 58, 44, 8, 0, 0 },
            { 0, 0, 16, 56, 46, 10, 0, 0 },
            { 0, -2, 16, 54, 48, 12, 0, 0 },
            { 0, -2, 14, 52, 52, 14, -2, 0 },
            { 0, 0, 12, 48, 54, 16, -2, 0 },
            { 0, 0, 10, 46, 56, 16, 0, 0 },
            { 0, 0, 8, 44, 58, 18, 0, 0 },
            { 0, 0, 6, 42, 60, 20, 0, 0 },
            { 0, 0, 4, 40, 62, 22, 0, 0 },
            { 0, 0, 4, 36, 62, 26, 0, 0 },
            { 0, 0, 2, 34, 62, 28, 2, 0 } },
        { { 0, 0, 0, 128, 0, 0, 0, 0 },
            { -2, 2, -6, 126, 8, -2, 2, 0 },
            { -2, 6, -12, 124, 16, -6, 4, -2 },
            { -2, 8, -18, 120, 26, -10, 6, -2 },
            { -4, 10, -22, 116, 38, -14, 6, -2 },
            { -4, 10, -22, 108, 48, -18, 8, -2 },
            { -4, 10, -24, 100, 60, -20, 8, -2 },
            { -4, 10, -24, 90, 70, -22, 10, -2 },
            { -4, 12, -24, 80, 80, -24, 12, -4 },
            { -2, 10, -22, 70, 90, -24, 10, -4 },
            { -2, 8, -20, 60, 100, -24, 10, -4 },
            { -2, 8, -18, 48, 108, -22, 10, -4 },
            { -2, 6, -14, 38, 116, -22, 10, -4 },
            { -2, 6, -10, 26, 120, -18, 8, -2 },
            { -2, 4, -6, 16, 124, -12, 6, -2 },
            { 0, 2, -2, 8, 126, -6, 2, -2 } },
        { { 0, 0, 0, 128, 0, 0, 0, 0 },
            { 0, 0, 0, 120, 8, 0, 0, 0 },
            { 0, 0, 0, 112, 16, 0, 0, 0 },
            { 0, 0, 0, 104, 24, 0, 0, 0 },
            { 0, 0, 0, 96, 32, 0, 0, 0 },
            { 0, 0, 0, 88, 40, 0, 0, 0 },
            { 0, 0, 0, 80, 48, 0, 0, 0 },
            { 0, 0, 0, 72, 56, 0, 0, 0 },
            { 0, 0, 0, 64, 64, 0, 0, 0 },
            { 0, 0, 0, 56, 72, 0, 0, 0 },
            { 0, 0, 0, 48, 80, 0, 0, 0 },
            { 0, 0, 0, 40, 88, 0, 0, 0 },
            { 0, 0, 0, 32, 96, 0, 0, 0 },
            { 0, 0, 0, 24, 104, 0, 0, 0 },
            { 0, 0, 0, 16, 112, 0, 0, 0 },
            { 0, 0, 0, 8, 120, 0, 0, 0 } },
        { { 0, 0, 0, 128, 0, 0, 0, 0 },
            { 0, 0, -4, 126, 8, -2, 0, 0 },
            { 0, 0, -8, 122, 18, -4, 0, 0 },
            { 0, 0, -10, 116, 28, -6, 0, 0 },
            { 0, 0, -12, 110, 38, -8, 0, 0 },
            { 0, 0, -12, 102, 48, -10, 0, 0 },
            { 0, 0, -14, 94, 58, -10, 0, 0 },
            { 0, 0, -12, 84, 66, -10, 0, 0 },
            { 0, 0, -12, 76, 76, -12, 0, 0 },
            { 0, 0, -10, 66, 84, -12, 0, 0 },
            { 0, 0, -10, 58, 94, -14, 0, 0 },
            { 0, 0, -10, 48, 102, -12, 0, 0 },
            { 0, 0, -8, 38, 110, -12, 0, 0 },
            { 0, 0, -6, 28, 116, -10, 0, 0 },
            { 0, 0, -4, 18, 122, -8, 0, 0 },
            { 0, 0, -2, 8, 126, -4, 0, 0 } },
        { { 0, 0, 0, 128, 0, 0, 0, 0 },
            { 0, 0, 30, 62, 34, 2, 0, 0 },
            { 0, 0, 26, 62, 36, 4, 0, 0 },
            { 0, 0, 22, 62, 40, 4, 0, 0 },
            { 0, 0, 20, 60, 42, 6, 0, 0 },
            { 0, 0, 18, 58, 44, 8, 0, 0 },
            { 0, 0, 16, 56, 46, 10, 0, 0 },
            { 0, 0, 14, 54, 48, 12, 0, 0 },
            { 0, 0, 12, 52, 52, 12, 0, 0 },
            { 0, 0, 12, 48, 54, 14, 0, 0 },
            { 0, 0, 10, 46, 56, 16, 0, 0 },
            { 0, 0, 8, 44, 58, 18, 0, 0 },
            { 0, 0, 6, 42, 60, 20, 0, 0 },
            { 0, 0, 4, 40, 62, 22, 0, 0 },
            { 0, 0, 4, 36, 62, 26, 0, 0 },
            { 0, 0, 2, 34, 62, 30, 0, 0 } }
    };
    void Block::PredictInter::blockInterPrediction(uint8_t refIdx, int refList, int plane, uint32_t w, uint32_t h, int candRow, int candCol)
    {
        std::vector<std::vector<uint8_t>>& pred = preds[refList];
        pred.assign(h, std::vector<uint8_t>(w));
        YuvFrame& ref = (refIdx == NONE_FRAME ? m_yuv : *m_frameStore[refIdx]);

        int lastX, lastY;
        if (refIdx == NONE_FRAME) {
            ASSERT(0);
        } else {
            int subX = plane ? m_sequence.subsampling_x : 0;
            int subY = plane ? m_sequence.subsampling_y : 0;
            auto& rinfo = m_frame.m_refInfo.m_refs[refIdx];
            lastX = ((rinfo.RefUpscaledWidth + subX) >> subX) - 1;
            lastY = ((rinfo.RefFrameHeight + subY) >> subY) - 1;
        }
        const int intermediateHeight = (((h - 1) * yStep + (1 << SCALE_SUBPEL_BITS) - 1) >> SCALE_SUBPEL_BITS) + 8;
        std::vector<std::vector<int>> intermediate(intermediateHeight, std::vector<int>(w));
        int filterIdx = getFilterIdx(w, candRow, candCol, 1);

        for (int r = 0; r < intermediateHeight; r++) {
            for (int c = 0; c < w; c++) {
                int s = 0;
                int p = startX + xStep * c;
                for (int t = 0; t < 8; t++) {
                    uint8_t pixel = ref.getPixel(plane, CLIP3(0, lastX, (p >> 10) + t - 3), CLIP3(0, lastY, (startY >> 10) + r - 3));
                    s += Subpel_Filters[filterIdx][(p >> 6) & SUBPEL_MASK][t] * pixel;
                }

                intermediate[r][c] = ROUND2(s, InterRound0);
            }
        }

        filterIdx = getFilterIdx(h, candRow, candCol, 0);
        for (int r = 0; r < h; r++) {
            for (int c = 0; c < w; c++) {
                int s = 0;
                int p = (startY & 1023) + yStep * r;
                for (int t = 0; t < 8; t++)
                    s += Subpel_Filters[filterIdx][(p >> 6) & SUBPEL_MASK][t] * intermediate[(p >> 10) + t][c];
                pred[r][c] = ROUND2(s, InterRound1);
            }
        }
    }

    static const int WARPEDDIFF_PREC_BITS = 10;
    static const int WARPEDPIXEL_PREC_SHIFTS = 1 << 6;

    static const int Warped_Filters[WARPEDPIXEL_PREC_SHIFTS * 3 + 1][8] = {
        { 0, 0, 127, 1, 0, 0, 0, 0 }, { 0, -1, 127, 2, 0, 0, 0, 0 },
        { 1, -3, 127, 4, -1, 0, 0, 0 }, { 1, -4, 126, 6, -2, 1, 0, 0 },
        { 1, -5, 126, 8, -3, 1, 0, 0 }, { 1, -6, 125, 11, -4, 1, 0, 0 },
        { 1, -7, 124, 13, -4, 1, 0, 0 }, { 2, -8, 123, 15, -5, 1, 0, 0 },
        { 2, -9, 122, 18, -6, 1, 0, 0 }, { 2, -10, 121, 20, -6, 1, 0, 0 },
        { 2, -11, 120, 22, -7, 2, 0, 0 }, { 2, -12, 119, 25, -8, 2, 0, 0 },
        { 3, -13, 117, 27, -8, 2, 0, 0 }, { 3, -13, 116, 29, -9, 2, 0, 0 },
        { 3, -14, 114, 32, -10, 3, 0, 0 }, { 3, -15, 113, 35, -10, 2, 0, 0 },
        { 3, -15, 111, 37, -11, 3, 0, 0 }, { 3, -16, 109, 40, -11, 3, 0, 0 },
        { 3, -16, 108, 42, -12, 3, 0, 0 }, { 4, -17, 106, 45, -13, 3, 0, 0 },
        { 4, -17, 104, 47, -13, 3, 0, 0 }, { 4, -17, 102, 50, -14, 3, 0, 0 },
        { 4, -17, 100, 52, -14, 3, 0, 0 }, { 4, -18, 98, 55, -15, 4, 0, 0 },
        { 4, -18, 96, 58, -15, 3, 0, 0 }, { 4, -18, 94, 60, -16, 4, 0, 0 },
        { 4, -18, 91, 63, -16, 4, 0, 0 }, { 4, -18, 89, 65, -16, 4, 0, 0 },
        { 4, -18, 87, 68, -17, 4, 0, 0 }, { 4, -18, 85, 70, -17, 4, 0, 0 },
        { 4, -18, 82, 73, -17, 4, 0, 0 }, { 4, -18, 80, 75, -17, 4, 0, 0 },
        { 4, -18, 78, 78, -18, 4, 0, 0 }, { 4, -17, 75, 80, -18, 4, 0, 0 },
        { 4, -17, 73, 82, -18, 4, 0, 0 }, { 4, -17, 70, 85, -18, 4, 0, 0 },
        { 4, -17, 68, 87, -18, 4, 0, 0 }, { 4, -16, 65, 89, -18, 4, 0, 0 },
        { 4, -16, 63, 91, -18, 4, 0, 0 }, { 4, -16, 60, 94, -18, 4, 0, 0 },
        { 3, -15, 58, 96, -18, 4, 0, 0 }, { 4, -15, 55, 98, -18, 4, 0, 0 },
        { 3, -14, 52, 100, -17, 4, 0, 0 }, { 3, -14, 50, 102, -17, 4, 0, 0 },
        { 3, -13, 47, 104, -17, 4, 0, 0 }, { 3, -13, 45, 106, -17, 4, 0, 0 },
        { 3, -12, 42, 108, -16, 3, 0, 0 }, { 3, -11, 40, 109, -16, 3, 0, 0 },
        { 3, -11, 37, 111, -15, 3, 0, 0 }, { 2, -10, 35, 113, -15, 3, 0, 0 },
        { 3, -10, 32, 114, -14, 3, 0, 0 }, { 2, -9, 29, 116, -13, 3, 0, 0 },
        { 2, -8, 27, 117, -13, 3, 0, 0 }, { 2, -8, 25, 119, -12, 2, 0, 0 },
        { 2, -7, 22, 120, -11, 2, 0, 0 }, { 1, -6, 20, 121, -10, 2, 0, 0 },
        { 1, -6, 18, 122, -9, 2, 0, 0 }, { 1, -5, 15, 123, -8, 2, 0, 0 },
        { 1, -4, 13, 124, -7, 1, 0, 0 }, { 1, -4, 11, 125, -6, 1, 0, 0 },
        { 1, -3, 8, 126, -5, 1, 0, 0 }, { 1, -2, 6, 126, -4, 1, 0, 0 },
        { 0, -1, 4, 127, -3, 1, 0, 0 }, { 0, 0, 2, 127, -1, 0, 0, 0 },
        { 0, 0, 0, 127, 1, 0, 0, 0 }, { 0, 0, -1, 127, 2, 0, 0, 0 },
        { 0, 1, -3, 127, 4, -2, 1, 0 }, { 0, 1, -5, 127, 6, -2, 1, 0 },
        { 0, 2, -6, 126, 8, -3, 1, 0 }, { -1, 2, -7, 126, 11, -4, 2, -1 },
        { -1, 3, -8, 125, 13, -5, 2, -1 }, { -1, 3, -10, 124, 16, -6, 3, -1 },
        { -1, 4, -11, 123, 18, -7, 3, -1 }, { -1, 4, -12, 122, 20, -7, 3, -1 },
        { -1, 4, -13, 121, 23, -8, 3, -1 }, { -2, 5, -14, 120, 25, -9, 4, -1 },
        { -1, 5, -15, 119, 27, -10, 4, -1 }, { -1, 5, -16, 118, 30, -11, 4, -1 },
        { -2, 6, -17, 116, 33, -12, 5, -1 }, { -2, 6, -17, 114, 35, -12, 5, -1 },
        { -2, 6, -18, 113, 38, -13, 5, -1 }, { -2, 7, -19, 111, 41, -14, 6, -2 },
        { -2, 7, -19, 110, 43, -15, 6, -2 }, { -2, 7, -20, 108, 46, -15, 6, -2 },
        { -2, 7, -20, 106, 49, -16, 6, -2 }, { -2, 7, -21, 104, 51, -16, 7, -2 },
        { -2, 7, -21, 102, 54, -17, 7, -2 }, { -2, 8, -21, 100, 56, -18, 7, -2 },
        { -2, 8, -22, 98, 59, -18, 7, -2 }, { -2, 8, -22, 96, 62, -19, 7, -2 },
        { -2, 8, -22, 94, 64, -19, 7, -2 }, { -2, 8, -22, 91, 67, -20, 8, -2 },
        { -2, 8, -22, 89, 69, -20, 8, -2 }, { -2, 8, -22, 87, 72, -21, 8, -2 },
        { -2, 8, -21, 84, 74, -21, 8, -2 }, { -2, 8, -22, 82, 77, -21, 8, -2 },
        { -2, 8, -21, 79, 79, -21, 8, -2 }, { -2, 8, -21, 77, 82, -22, 8, -2 },
        { -2, 8, -21, 74, 84, -21, 8, -2 }, { -2, 8, -21, 72, 87, -22, 8, -2 },
        { -2, 8, -20, 69, 89, -22, 8, -2 }, { -2, 8, -20, 67, 91, -22, 8, -2 },
        { -2, 7, -19, 64, 94, -22, 8, -2 }, { -2, 7, -19, 62, 96, -22, 8, -2 },
        { -2, 7, -18, 59, 98, -22, 8, -2 }, { -2, 7, -18, 56, 100, -21, 8, -2 },
        { -2, 7, -17, 54, 102, -21, 7, -2 }, { -2, 7, -16, 51, 104, -21, 7, -2 },
        { -2, 6, -16, 49, 106, -20, 7, -2 }, { -2, 6, -15, 46, 108, -20, 7, -2 },
        { -2, 6, -15, 43, 110, -19, 7, -2 }, { -2, 6, -14, 41, 111, -19, 7, -2 },
        { -1, 5, -13, 38, 113, -18, 6, -2 }, { -1, 5, -12, 35, 114, -17, 6, -2 },
        { -1, 5, -12, 33, 116, -17, 6, -2 }, { -1, 4, -11, 30, 118, -16, 5, -1 },
        { -1, 4, -10, 27, 119, -15, 5, -1 }, { -1, 4, -9, 25, 120, -14, 5, -2 },
        { -1, 3, -8, 23, 121, -13, 4, -1 }, { -1, 3, -7, 20, 122, -12, 4, -1 },
        { -1, 3, -7, 18, 123, -11, 4, -1 }, { -1, 3, -6, 16, 124, -10, 3, -1 },
        { -1, 2, -5, 13, 125, -8, 3, -1 }, { -1, 2, -4, 11, 126, -7, 2, -1 },
        { 0, 1, -3, 8, 126, -6, 2, 0 }, { 0, 1, -2, 6, 127, -5, 1, 0 },
        { 0, 1, -2, 4, 127, -3, 1, 0 }, { 0, 0, 0, 2, 127, -1, 0, 0 },
        { 0, 0, 0, 1, 127, 0, 0, 0 }, { 0, 0, 0, -1, 127, 2, 0, 0 },
        { 0, 0, 1, -3, 127, 4, -1, 0 }, { 0, 0, 1, -4, 126, 6, -2, 1 },
        { 0, 0, 1, -5, 126, 8, -3, 1 }, { 0, 0, 1, -6, 125, 11, -4, 1 },
        { 0, 0, 1, -7, 124, 13, -4, 1 }, { 0, 0, 2, -8, 123, 15, -5, 1 },
        { 0, 0, 2, -9, 122, 18, -6, 1 }, { 0, 0, 2, -10, 121, 20, -6, 1 },
        { 0, 0, 2, -11, 120, 22, -7, 2 }, { 0, 0, 2, -12, 119, 25, -8, 2 },
        { 0, 0, 3, -13, 117, 27, -8, 2 }, { 0, 0, 3, -13, 116, 29, -9, 2 },
        { 0, 0, 3, -14, 114, 32, -10, 3 }, { 0, 0, 3, -15, 113, 35, -10, 2 },
        { 0, 0, 3, -15, 111, 37, -11, 3 }, { 0, 0, 3, -16, 109, 40, -11, 3 },
        { 0, 0, 3, -16, 108, 42, -12, 3 }, { 0, 0, 4, -17, 106, 45, -13, 3 },
        { 0, 0, 4, -17, 104, 47, -13, 3 }, { 0, 0, 4, -17, 102, 50, -14, 3 },
        { 0, 0, 4, -17, 100, 52, -14, 3 }, { 0, 0, 4, -18, 98, 55, -15, 4 },
        { 0, 0, 4, -18, 96, 58, -15, 3 }, { 0, 0, 4, -18, 94, 60, -16, 4 },
        { 0, 0, 4, -18, 91, 63, -16, 4 }, { 0, 0, 4, -18, 89, 65, -16, 4 },
        { 0, 0, 4, -18, 87, 68, -17, 4 }, { 0, 0, 4, -18, 85, 70, -17, 4 },
        { 0, 0, 4, -18, 82, 73, -17, 4 }, { 0, 0, 4, -18, 80, 75, -17, 4 },
        { 0, 0, 4, -18, 78, 78, -18, 4 }, { 0, 0, 4, -17, 75, 80, -18, 4 },
        { 0, 0, 4, -17, 73, 82, -18, 4 }, { 0, 0, 4, -17, 70, 85, -18, 4 },
        { 0, 0, 4, -17, 68, 87, -18, 4 }, { 0, 0, 4, -16, 65, 89, -18, 4 },
        { 0, 0, 4, -16, 63, 91, -18, 4 }, { 0, 0, 4, -16, 60, 94, -18, 4 },
        { 0, 0, 3, -15, 58, 96, -18, 4 }, { 0, 0, 4, -15, 55, 98, -18, 4 },
        { 0, 0, 3, -14, 52, 100, -17, 4 }, { 0, 0, 3, -14, 50, 102, -17, 4 },
        { 0, 0, 3, -13, 47, 104, -17, 4 }, { 0, 0, 3, -13, 45, 106, -17, 4 },
        { 0, 0, 3, -12, 42, 108, -16, 3 }, { 0, 0, 3, -11, 40, 109, -16, 3 },
        { 0, 0, 3, -11, 37, 111, -15, 3 }, { 0, 0, 2, -10, 35, 113, -15, 3 },
        { 0, 0, 3, -10, 32, 114, -14, 3 }, { 0, 0, 2, -9, 29, 116, -13, 3 },
        { 0, 0, 2, -8, 27, 117, -13, 3 }, { 0, 0, 2, -8, 25, 119, -12, 2 },
        { 0, 0, 2, -7, 22, 120, -11, 2 }, { 0, 0, 1, -6, 20, 121, -10, 2 },
        { 0, 0, 1, -6, 18, 122, -9, 2 }, { 0, 0, 1, -5, 15, 123, -8, 2 },
        { 0, 0, 1, -4, 13, 124, -7, 1 }, { 0, 0, 1, -4, 11, 125, -6, 1 },
        { 0, 0, 1, -3, 8, 126, -5, 1 }, { 0, 0, 1, -2, 6, 126, -4, 1 },
        { 0, 0, 0, -1, 4, 127, -3, 1 }, { 0, 0, 0, 0, 2, 127, -1, 0 },
        { 0, 0, 0, 0, 2, 127, -1, 0 }
    };

    void Block::PredictInter::blockWarp(int useWarp, int plane, uint8_t refIdx, int refList, int x, int y, int i8, int j8, int w, int h)
    {
        YuvFrame& ref = *m_frameStore[refIdx];
        std::vector<std::vector<uint8_t>>& pred = preds[refList];
        int subX = plane ? m_sequence.subsampling_x : 0;
        int subY = plane ? m_sequence.subsampling_y : 0;
        auto& rinfo = m_frame.m_refInfo.m_refs[refIdx];
        int lastX = ((rinfo.RefUpscaledWidth + subX) >> subX) - 1;
        int lastY = ((rinfo.RefFrameHeight + subY) >> subY) - 1;
        int srcX = (x + j8 * 8 + 4) << subX;
        int srcY = (y + i8 * 8 + 4) << subY;
        const int* warpParams = useWarp == 1 ? m_localWarp.LocalWarpParams : m_frame.gm_params[m_block.RefFrame[refList]];
        int dstX = warpParams[2] * srcX + warpParams[3] * srcY + warpParams[0];
        int dstY = warpParams[4] * srcX + warpParams[5] * srcY + warpParams[1];
        int alpha, beta, gamma, delta;
        bool warpValid = m_localWarp.setupShear(warpParams, alpha, beta, gamma, delta);

        int intermediate[16][8];

        int x4 = dstX >> subX;
        int y4 = dstY >> subY;
        int ix4 = x4 >> WARPEDMODEL_PREC_BITS;
        int sx4 = x4 & ((1 << WARPEDMODEL_PREC_BITS) - 1);
        int iy4 = y4 >> WARPEDMODEL_PREC_BITS;
        int sy4 = y4 & ((1 << WARPEDMODEL_PREC_BITS) - 1);
        for (int i1 = -7; i1 < 8; i1++) {
            for (int i2 = -4; i2 < 4; i2++) {
                int sx = sx4 + alpha * i2 + beta * i1;
                int offs = ROUND2(sx, WARPEDDIFF_PREC_BITS) + WARPEDPIXEL_PREC_SHIFTS;
                int s = 0;
                for (int i3 = 0; i3 < 8; i3++) {
                    uint8_t pixel = ref.getPixel(plane, CLIP3(0, lastX, ix4 + i2 - 3 + i3), CLIP3(0, lastY, iy4 + i1));
                    s += Warped_Filters[offs][i3] * pixel;
                }
                intermediate[(i1 + 7)][(i2 + 4)] = ROUND2(s, InterRound0);
            }
        }
        for (int i1 = -4; i1 < std::min(4, h - i8 * 8 - 4); i1++) {
            for (int i2 = -4; i2 < std::min(4, w - j8 * 8 - 4); i2++) {
                int sy = sy4 + gamma * i2 + delta * i1;
                int offs = ROUND2(sy, WARPEDDIFF_PREC_BITS) + WARPEDPIXEL_PREC_SHIFTS;
                int s = 0;
                for (int i3 = 0; i3 < 8; i3++) {
                    s += Warped_Filters[offs][i3] * intermediate[(i1 + i3 + 4)][(i2 + 4)];
                }
                pred[i8 * 8 + i1 + 4][j8 * 8 + i2 + 4] = ROUND2(s, InterRound1);
            }
        }
    }
    void Block::PredictInter::predict_inter(int plane, int x, int y, uint32_t w, uint32_t h, int candRow, int candCol)
    {
        isCompound = m_frame.RefFrames[candRow][candCol][1] > INTRA_FRAME;
        if (isCompound) {
            ASSERT(0);
        }
        roundingVariablesDerivation(isCompound, m_sequence.BitDepth, InterRound0, InterRound1, InterPostRound);
        if (!plane && m_block.motion_mode == LOCALWARP) {
            m_localWarp.warpEstimation();
            m_localWarp.setupShear();
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
            std::vector<std::vector<uint8_t>>& pred = preds[refList];
            pred.assign(h, std::vector<uint8_t>(w));
            for (int i8 = 0; i8 <= ((h - 1) >> 3); i8++) {
                for (int j8 = 0; j8 <= ((w - 1) >> 3); j8++) {
                    blockWarp(useWarp, plane, refIdx, refList, x, y, i8, j8, w, h);
                }
            }

        } else {
            blockInterPrediction(refIdx, refList, plane, w, h, candRow, candCol);
        }
        if (!isCompound) {
            for (int i = 0; i < h; i++) {
                for (int j = 0; j < w; j++) {
                    m_yuv.setPixel(plane, x + j, y + i, preds[0][i][j]);
                }
            }
        } else {
            ASSERT(0);
        }
    }

    void Block::compute_prediction(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore)
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
                uint32_t predW = bw >> subX;
                uint32_t predH = bh >> subY;
                bool someUseIntra = false;
                for (uint32_t r = 0; r < (bh4 << subY); r++) {
                    for (uint32_t c = 0; c < (bw4 << subX); c++) {
                        if (m_frame.RefFrames[candRow + r][candCol + c][0] == INTRA_FRAME)
                            someUseIntra = true;
                    }
                }
                if (someUseIntra) {
                    predW = bw;
                    predH = bh;
                    candRow = MiRow;
                    candCol = MiCol;
                }
                uint32_t r = 0;
                for (uint32_t y = 0; y < bh; y += predH) {
                    uint32_t c = 0;
                    for (uint32_t x = 0; x < bw; x += predW) {
                        PredictInter inter(*this, *frame, frameStore);
                        inter.predict_inter(plane, baseX + x, baseY + y, predW, predH, candRow + r, candCol + c);
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
        } else {
            for (int dir = 0; dir < 2; dir++)
                interp_filter[dir] = m_frame.interpolation_filter;
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

    void Block::LocalWarp::resolveDivisor(int d, int& divShift, int& divFactor) const
    {
        static const int DIV_LUT_BITS = 8;
        static const int DIV_LUT_PREC_BITS = 14;
        int n = FloorLog2(std::abs(d));
        int e = std::abs(d) - (1 << n);
        int f = n > DIV_LUT_BITS ? ROUND2(e, n - DIV_LUT_BITS) : (e << (DIV_LUT_BITS - n));
        divShift = (n + DIV_LUT_PREC_BITS);
        divFactor = d < 0 ? -Div_Lut[f] : Div_Lut[f];
    }
    inline int ls_product(int a, int b)
    {
        return ((a * b) >> 2) + (a + b);
    }

    static const int WARPEDMODEL_NONDIAGAFFINE_CLAMP = 1 << 13;

    inline int nondiag(int v, int divFactor, int divShift)
    {
        return CLIP3(-WARPEDMODEL_NONDIAGAFFINE_CLAMP + 1,
            WARPEDMODEL_NONDIAGAFFINE_CLAMP - 1,
            ROUND2SIGNED(v * divFactor, divShift));
    }
    inline int diag(int v, int divFactor, int divShift)
    {
        return CLIP3((1 << WARPEDMODEL_PREC_BITS) - WARPEDMODEL_NONDIAGAFFINE_CLAMP + 1,
            (1 << WARPEDMODEL_PREC_BITS) + WARPEDMODEL_NONDIAGAFFINE_CLAMP - 1,
            ROUND2SIGNED(v * divFactor, divShift));
    }
    void Block::LocalWarp::warpEstimation()
    {
        int A[2][2];
        int Bx[2];
        int By[2];
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
        int det = A[0][0] * A[1][1] - A[0][1] * A[0][1];
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
        LocalWarpParams[2] = diag(A[1][1] * Bx[0] - A[0][1] * Bx[1], divFactor, divShift );
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
        int v = (warpParams[4] << WARPEDMODEL_PREC_BITS);
        int gamma0 = CLIP3(-32768, 32767, ROUND2SIGNED(v * divFactor, divShift));
        int w = (warpParams[3] * warpParams[4]);
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
        } else {
            for (int idx = NumMvFound; idx < 2; idx++) {
                RefStackMv[idx][0] = GlobalMvs[0];
            }
        }
    }
    void Block::FindMvStack::find_mv_stack()
    {
        setupGlobalMV(0);
        if (isCompound)
            setupGlobalMV(1);
        FoundMatch = false;
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
        if (std::abs(deltaCol) > 1) {
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
        //lower_mv_precision(candMv);
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
        if (std::abs(deltaRow) > 1) {
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
