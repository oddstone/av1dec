
#include "InterPredict.h"
#include "Parser.h"
#include "VideoFrame.h"
#include "log.h"

namespace YamiAv1 {

Block::InterPredict::InterPredict(Block& block, YuvFrame& yuv, const FrameStore& frameStore)
    : m_localWarp(block.m_localWarp)
    , m_block(block)
    , m_frame(block.m_frame)
    , m_sequence(block.m_sequence)
    , m_yuv(yuv)
    , m_frameStore(frameStore)
{
}

uint8_t Block::InterPredict::getUseWarp(int w, int h, int refFrame)
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

void Block::InterPredict::motionVectorScaling(uint8_t refIdx, int plane, int x, int y, const Mv& mv)
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

inline int Block::InterPredict::getFilterIdx(int size, int candRow, int candCol, int dir)
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
void Block::InterPredict::blockInterPrediction(uint8_t refIdx, int refList, int plane, uint32_t w, uint32_t h, int candRow, int candCol)
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

void Block::InterPredict::blockWarp(int useWarp, int plane, uint8_t refIdx, int refList, int x, int y, int i8, int j8, int w, int h)
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

void Block::InterPredict::IntraVariantMask(std::vector<std::vector<uint8_t>>& Mask, int w, int h) const
{
    int Ii_Weights_1d[MAX_SB_SIZE] = {
        60, 58, 56, 54, 52, 50, 48, 47, 45, 44, 42, 41, 39, 38, 37, 35, 34, 33, 32,
        31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 22, 21, 20, 19, 19, 18, 18, 17, 16,
        16, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 9, 8,
        8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4,
        4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    int sizeScale = MAX_SB_SIZE / std::max(h, w);
    Mask.assign(h, std::vector<uint8_t>(w));
    for (int i = 0; i < h; i++) {
        INTERINTRA_MODE interintra_mode = m_block.interintra_mode;
        for (int j = 0; j < w; j++) {
            if (interintra_mode == II_V_PRED) {
                Mask[i][j] = Ii_Weights_1d[i * sizeScale];
            } else if (interintra_mode == II_H_PRED) {
                Mask[i][j] = Ii_Weights_1d[j * sizeScale];
            } else if (interintra_mode == II_SMOOTH_PRED) {
                Mask[i][j] = Ii_Weights_1d[std::min(i, j) * sizeScale];
            } else {
                Mask[i][j] = 32;
            }
        }
    }
}

void Block::InterPredict::maskBlend(const std::vector<std::vector<uint8_t>>& Mask, int plane, int dstX, int dstY, int w, int h)
{
    uint8_t subX = plane ? m_sequence.subsampling_x : 0;
    uint8_t subY = plane ? m_sequence.subsampling_y : 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int m;
            if ((!subX && !subY) || (m_block.interintra && !m_block.wedge_interintra)) {
                m = Mask[y][x];
            } else if (subX && !subY) {
                m = ROUND2(Mask[y][2 * x] + Mask[y][2 * x + 1], 1);
            } else if (!subX && subY) {
                m = ROUND2(Mask[2 * y][x] + Mask[2 * y + 1][x], 1);
            } else {
                m = ROUND2(Mask[2 * y][2 * x] + Mask[2 * y][2 * x + 1] + Mask[2 * y + 1][2 * x] + Mask[2 * y + 1][2 * x + 1], 2);
            }
            if (m_block.interintra) {
                int pred0 = CLIP1(ROUND2(preds[0][y][x], InterPostRound));
                int pred1 = m_yuv.getPixel(plane, dstX + x, dstY + y);
                m_yuv.setPixel(plane, dstX + x, dstY + y, ROUND2(m * pred1 + (64 - m) * pred0, 6));
            } else {
                int pred0 = preds[0][y][x];
                int pred1 = preds[1][y][x];
                m_yuv.setPixel(plane, dstX + x, dstY + y, CLIP1(ROUND2(m * pred0 + (64 - m) * pred1, 6 + InterPostRound)));
            }
        }
    }

}

void Block::InterPredict::predict_inter(int plane, int x, int y, uint32_t w, uint32_t h, int candRow, int candCol)
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
    COMPOUND_TYPE compound_type = m_block.compound_type;
    std::vector<std::vector<uint8_t>> Mask;
    if (compound_type == COMPOUND_WEDGE && plane == 0) {
        //ASSERT(0);
    } else if (compound_type == COMPOUND_INTRA) {
        IntraVariantMask(Mask, w, h);
    } else if (compound_type == COMPOUND_DIFFWTD && plane == 0) {
        ASSERT(0);
    }
    if (compound_type == COMPOUND_DISTANCE) {
        ASSERT(0);
    }

    bool IsInterIntra = (m_block.is_inter && m_block.RefFrame[1] == INTRA_FRAME);
    if (!isCompound && !IsInterIntra) {
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                m_yuv.setPixel(plane, x + j, y + i, preds[0][i][j]);
            }
        }
    } else if (compound_type == COMPOUND_AVERAGE) {
        ASSERT(0);
    } else if (compound_type == COMPOUND_DISTANCE) {
        ASSERT(0);
    } else if (compound_type == COMPOUND_WEDGE) {
    }
    else {
        maskBlend(Mask, plane, x, y, w, h);
    }
    /*if (m_block.motion_mode == OBMC_CAUSAL) {
        ASSERT(0);
    }*/
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
