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

#include "InterPredict.h"
#include "Parser.h"
#include "VideoFrame.h"
#include "log.h"

namespace YamiAv1 {

Block::InterPredict::InterPredict(Block& block, int p, YuvFrame& yuv, const FrameStore& frameStore, std::vector<std::vector<uint8_t>>& mask)
    : m_localWarp(block.m_localWarp)
    , m_block(block)
    , m_frame(block.m_frame)
    , m_sequence(block.m_sequence)
    , plane(p)
    , subX(plane ? m_sequence.subsampling_x : 0)
    , subY(plane ? m_sequence.subsampling_y : 0)
    , MiRow(m_block.MiRow)
    , MiCol(m_block.MiCol)
    , m_yuv(yuv)
    , m_frameStore(frameStore)
    , Mask(mask)
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

void Block::InterPredict::motionVectorScaling(int8_t refIdx, int x, int y, const Mv& mv)
{
    uint32_t xScale, yScale;
    m_frame.getScale(refIdx, xScale, yScale);

    const static int halfSample = (1 << (SUBPEL_BITS - 1));
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

struct Tap {
    int start;
    int end;
};

Tap Subpel_Filter_Taps[6][16] = {
    {
        { 3, 4 },
        { 1, 6 },
        { 1, 6 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 1, 7 },
        { 2, 7 },
        { 2, 7 },
    },
    {
        { 3, 4 },
        { 1, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 1, 6 },
        { 1, 7 },
        { 2, 7 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 7 },
    },
    {
        { 3, 4 },
        { 0, 7 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 0, 8 },
        { 1, 8 },
    },
    {
        { 3, 4 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
        { 3, 5 },
    },
    {
        { 3, 4 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
    },
    {
        { 3, 4 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
        { 2, 6 },
    },
};

void Block::InterPredict::blockInterPrediction(int8_t refIdx, int refList, int w, int h, int candRow, int candCol)
{
    std::vector<std::vector<int16_t>>& pred = preds[refList];
    pred.assign(h, std::vector<int16_t>(w));
    YuvFrame& ref = (refIdx == NONE_FRAME ? m_yuv : *m_frameStore[refIdx]);

    int lastX, lastY;
    auto& rinfo = m_frame.m_refInfo.m_refs[refIdx];
    lastX = ((rinfo.RefUpscaledWidth + subX) >> subX) - 1;
    lastY = ((rinfo.RefFrameHeight + subY) >> subY) - 1;
    const int intermediateHeight = (((h - 1) * yStep + (1 << SCALE_SUBPEL_BITS) - 1) >> SCALE_SUBPEL_BITS) + 8;
    std::vector<std::vector<int>> intermediate(intermediateHeight, std::vector<int>(w));
    int filterIdx = getFilterIdx(w, candRow, candCol, 1);

    for (int r = 0; r < intermediateHeight; r++) {
        for (int c = 0; c < w; c++) {
            int p = startX + xStep * c;
            const int* filter = Subpel_Filters[filterIdx][(p >> 6) & SUBPEL_MASK];
            const Tap& tap = Subpel_Filter_Taps[filterIdx][(p >> 6) & SUBPEL_MASK];

            int x = (p >> 10);
            int y = (startY >> 10) + r;

            int s = 0;
            for (int t = tap.start; t < tap.end; t++) {
                uint8_t pixel = ref.getPixel(plane, CLIP3(0, lastX, x + t - 3), CLIP3(0, lastY, y - 3));
                s += filter[t] * pixel;
            }
            intermediate[r][c] = ROUND2(s, InterRound0);
        }
    }

    filterIdx = getFilterIdx(h, candRow, candCol, 0);
    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            int p = (startY & 1023) + yStep * r;
            const int* filter = Subpel_Filters[filterIdx][(p >> 6) & SUBPEL_MASK];
            const Tap& tap = Subpel_Filter_Taps[filterIdx][(p >> 6) & SUBPEL_MASK];

            int y = p >> 10;

            int s = 0;
            for (int t = tap.start; t < tap.end; t++)
                s += filter[t] * intermediate[y + t][c];
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

void Block::InterPredict::blockWarp(int useWarp, uint8_t refIdx, int refList, int x, int y, int i8, int j8, int w, int h)
{
    YuvFrame& ref = *m_frameStore[refIdx];
    std::vector<std::vector<int16_t>>& pred = preds[refList];
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

void Block::InterPredict::intraModeVariantMask(int w, int h)
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

void Block::InterPredict::maskBlend(int dstX, int dstY, int w, int h)
{
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
                m_yuv.setPixel(plane, dstX + x, dstY + y, CLIP1(ROUND2(m * pred1 + (64 - m) * pred0, 6)));
            } else {
                int pred0 = preds[0][y][x];
                int pred1 = preds[1][y][x];
                m_yuv.setPixel(plane, dstX + x, dstY + y, CLIP1(ROUND2(m * pred0 + (64 - m) * pred1, 6 + InterPostRound)));
            }
        }
    }
}

void Block::InterPredict::predict_overlap(int pass, int candRow, int candCol, int x4, int y4, int predW, int predH, const uint8_t* mask)
{
    Mv mv = m_frame.Mvs[candRow][candCol][0];
    int8_t refIdx = m_frame.ref_frame_idx[m_frame.RefFrames[candRow][candCol][0] - LAST_FRAME];
    int predX = (x4 * 4) >> subX;
    int predY = (y4 * 4) >> subY;
    motionVectorScaling(refIdx, predX, predY, mv);
    blockInterPrediction(refIdx, 0, predW, predH, candRow, candCol);
    for (int i = 0; i < predH; i++) {
        for (int j = 0; j < predW; j++) {
            int m = pass ? mask[j] : mask[i];
            uint8_t pixel = m_yuv.getPixel(plane, predX + j, predY + i);
            pixel = CLIP1(ROUND2(m * pixel + (64 - m) * CLIP1(preds[0][i][j]), 6));
            m_yuv.setPixel(plane, predX + j, predY + i, pixel);
        }
    }
}

const uint8_t* get_obmc_mask(int length)
{
    const static uint8_t Obmc_Mask_2[2] = { 45, 64 };
    const static uint8_t Obmc_Mask_4[4] = { 39, 50, 59, 64 };
    const static uint8_t Obmc_Mask_8[8] = { 36, 42, 48, 53, 57, 61, 64, 64 };
    const static uint8_t Obmc_Mask_16[16] = {
        34, 37, 40, 43, 46, 49, 52, 54,
        56, 58, 60, 61, 64, 64, 64, 64
    };
    const static uint8_t Obmc_Mask_32[32] = {
        33, 35, 36, 38, 40, 41, 43, 44,
        45, 47, 48, 50, 51, 52, 53, 55,
        56, 57, 58, 59, 60, 60, 61, 62,
        64, 64, 64, 64, 64, 64, 64, 64
    };
    if (length == 2) {
        return Obmc_Mask_2;
    } else if (length == 4) {
        return Obmc_Mask_4;
    } else if (length == 8) {
        return Obmc_Mask_8;
    } else if (length == 16) {
        return Obmc_Mask_16;
    } else {
        return Obmc_Mask_32;
    }
}

void Block::InterPredict::overlappedMotionCompensation(int w, int h)
{
    BLOCK_SIZE MiSize = m_block.MiSize;
    if (m_block.AvailU) {
        if (m_sequence.get_plane_residual_size(MiSize, plane) >= BLOCK_8X8) {
            int pass = 0;
            int w4 = Num_4x4_Blocks_Wide[MiSize];
            int x4 = MiCol;
            int y4 = MiRow;
            int nCount = 0;
            int nLimit = std::min(4, (int)Mi_Width_Log2[MiSize]);
            while (nCount < nLimit && x4 < std::min(m_frame.MiCols, MiCol + w4)) {
                int candRow = MiRow - 1;
                int candCol = x4 | 1;
                BLOCK_SIZE candSz = m_frame.MiSizes[candRow][candCol];
                int step4 = CLIP3(2, 16, Num_4x4_Blocks_Wide[candSz]);
                if (m_frame.RefFrames[candRow][candCol][0] > INTRA_FRAME) {
                    nCount += 1;
                    int predW = std::min(w, (step4 * MI_SIZE) >> subX);
                    int predH = std::min(h >> 1, 32 >> subY);
                    const uint8_t* mask = get_obmc_mask(predH);
                    predict_overlap(pass, candRow, candCol, x4, y4, predW, predH, mask);
                }
                x4 += step4;
            }
        }
    }
    if (m_block.AvailL) {
        int pass = 1;
        int h4 = Num_4x4_Blocks_High[MiSize];
        int x4 = MiCol;
        int y4 = MiRow;
        int nCount = 0;
        int nLimit = std::min(4, (int)Mi_Height_Log2[MiSize]);
        while (nCount < nLimit && y4 < std::min(m_frame.MiRows, MiRow + h4)) {
            int candCol = MiCol - 1;
            int candRow = y4 | 1;
            BLOCK_SIZE candSz = m_frame.MiSizes[candRow][candCol];
            int step4 = CLIP3(2, 16, Num_4x4_Blocks_High[candSz]);
            if (m_frame.RefFrames[candRow][candCol][0] > INTRA_FRAME) {
                nCount += 1;
                int predW = std::min(w >> 1, 32 >> subX);
                int predH = std::min(h, (step4 * MI_SIZE) >> subY);
                const uint8_t* mask = get_obmc_mask(predW);
                predict_overlap(pass, candRow, candCol, x4, y4, predW, predH, mask);
            }
            y4 += step4;
        }
    }
}

const int MASK_MASTER_SIZE = 64;
static const int Wedge_Master_Oblique_Odd[MASK_MASTER_SIZE] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 6, 18,
    37, 53, 60, 63, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
};

static const int Wedge_Master_Oblique_Even[MASK_MASTER_SIZE] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4, 11, 27,
    46, 58, 62, 63, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
};

static int Wedge_Master_Vertical[MASK_MASTER_SIZE] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 7, 21,
    43, 57, 62, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
};

enum {
    WEDGE_HORIZONTAL,
    WEDGE_VERTICAL,
    WEDGE_OBLIQUE27,
    WEDGE_OBLIQUE63,
    WEDGE_OBLIQUE117,
    WEDGE_OBLIQUE153,
    WEDGE_DIRS,
};

static const int WEDGE_TYPES = 16;

int WedgeMasks[BLOCK_SIZES_ALL][2][WEDGE_TYPES][MASK_MASTER_SIZE][MASK_MASTER_SIZE];
int MasterMask[WEDGE_DIRS][MASK_MASTER_SIZE][MASK_MASTER_SIZE];

static int Wedge_Bits[BLOCK_SIZES_ALL] = {
    0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 0,
    0, 0, 0, 0, 0, 0, 0, 4, 4, 0, 0
};

int Wedge_Codebook[3][16][3] = {
    {
        { WEDGE_OBLIQUE27, 4, 4 },
        { WEDGE_OBLIQUE63, 4, 4 },
        { WEDGE_OBLIQUE117, 4, 4 },
        { WEDGE_OBLIQUE153, 4, 4 },
        { WEDGE_HORIZONTAL, 4, 2 },
        { WEDGE_HORIZONTAL, 4, 4 },
        { WEDGE_HORIZONTAL, 4, 6 },
        { WEDGE_VERTICAL, 4, 4 },
        { WEDGE_OBLIQUE27, 4, 2 },
        { WEDGE_OBLIQUE27, 4, 6 },
        { WEDGE_OBLIQUE153, 4, 2 },
        { WEDGE_OBLIQUE153, 4, 6 },
        { WEDGE_OBLIQUE63, 2, 4 },
        { WEDGE_OBLIQUE63, 6, 4 },
        { WEDGE_OBLIQUE117, 2, 4 },
        { WEDGE_OBLIQUE117, 6, 4 },
    },
    {
        { WEDGE_OBLIQUE27, 4, 4 },
        { WEDGE_OBLIQUE63, 4, 4 },
        { WEDGE_OBLIQUE117, 4, 4 },
        { WEDGE_OBLIQUE153, 4, 4 },
        { WEDGE_VERTICAL, 2, 4 },
        { WEDGE_VERTICAL, 4, 4 },
        { WEDGE_VERTICAL, 6, 4 },
        { WEDGE_HORIZONTAL, 4, 4 },
        { WEDGE_OBLIQUE27, 4, 2 },
        { WEDGE_OBLIQUE27, 4, 6 },
        { WEDGE_OBLIQUE153, 4, 2 },
        { WEDGE_OBLIQUE153, 4, 6 },
        { WEDGE_OBLIQUE63, 2, 4 },
        { WEDGE_OBLIQUE63, 6, 4 },
        { WEDGE_OBLIQUE117, 2, 4 },
        { WEDGE_OBLIQUE117, 6, 4 },
    },
    {
        { WEDGE_OBLIQUE27, 4, 4 },
        { WEDGE_OBLIQUE63, 4, 4 },
        { WEDGE_OBLIQUE117, 4, 4 },
        { WEDGE_OBLIQUE153, 4, 4 },
        { WEDGE_HORIZONTAL, 4, 2 },
        { WEDGE_HORIZONTAL, 4, 6 },
        { WEDGE_VERTICAL, 2, 4 },
        { WEDGE_VERTICAL, 6, 4 },
        { WEDGE_OBLIQUE27, 4, 2 },
        { WEDGE_OBLIQUE27, 4, 6 },
        { WEDGE_OBLIQUE153, 4, 2 },
        { WEDGE_OBLIQUE153, 4, 6 },
        { WEDGE_OBLIQUE63, 2, 4 },
        { WEDGE_OBLIQUE63, 6, 4 },
        { WEDGE_OBLIQUE117, 2, 4 },
        { WEDGE_OBLIQUE117, 6, 4 },
    }
};

int block_shape(int bsize)
{
    int w4 = Num_4x4_Blocks_Wide[bsize];
    int h4 = Num_4x4_Blocks_High[bsize];
    if (h4 > w4)
        return 0;
    else if (h4 < w4)
        return 1;
    else
        return 2;
}

int get_wedge_direction(int bsize, int index)
{
    return Wedge_Codebook[block_shape(bsize)][index][0];
}
int get_wedge_xoff(int bsize, int index)
{
    return Wedge_Codebook[block_shape(bsize)][index][1];
}
int get_wedge_yoff(int bsize, int index)
{
    return Wedge_Codebook[block_shape(bsize)][index][2];
}

void initialise_wedge_mask_table()
{
    static bool inited = false;
    if (inited)
        return;
    int w = MASK_MASTER_SIZE;
    int h = MASK_MASTER_SIZE;
    for (int j = 0; j < w; j++) {
        int shift = MASK_MASTER_SIZE / 4;
        for (int i = 0; i < h; i += 2) {
            MasterMask[WEDGE_OBLIQUE63][i][j] = Wedge_Master_Oblique_Even[CLIP3(0, MASK_MASTER_SIZE - 1, j - shift)];
            shift -= 1;
            MasterMask[WEDGE_OBLIQUE63][i + 1][j] = Wedge_Master_Oblique_Odd[CLIP3(0, MASK_MASTER_SIZE - 1, j - shift)];
            MasterMask[WEDGE_VERTICAL][i][j] = Wedge_Master_Vertical[j];
            MasterMask[WEDGE_VERTICAL][i + 1][j] = Wedge_Master_Vertical[j];
        }
    }
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int msk = MasterMask[WEDGE_OBLIQUE63][i][j];
            MasterMask[WEDGE_OBLIQUE27][j][i] = msk;
            MasterMask[WEDGE_OBLIQUE117][i][w - 1 - j] = 64 - msk;
            MasterMask[WEDGE_OBLIQUE153][w - 1 - j][i] = 64 - msk;
            MasterMask[WEDGE_HORIZONTAL][j][i] = MasterMask[WEDGE_VERTICAL][i][j];
        }
    }
    for (int bsize = BLOCK_8X8; bsize < BLOCK_SIZES_ALL; bsize++) {
        if (Wedge_Bits[bsize] > 0) {
            w = Block_Width[bsize];
            h = Block_Height[bsize];
            for (int wedge = 0; wedge < WEDGE_TYPES; wedge++) {
                int dir = get_wedge_direction((BLOCK_SIZE)bsize, wedge);
                int xoff = MASK_MASTER_SIZE / 2 - ((get_wedge_xoff(bsize, wedge) * w) >> 3);
                int yoff = MASK_MASTER_SIZE / 2 - ((get_wedge_yoff(bsize, wedge) * h) >> 3);
                int sum = 0;
                for (int i = 0; i < w; i++)
                    sum += MasterMask[dir][yoff][xoff + i];
                for (int i = 1; i < h; i++)
                    sum += MasterMask[dir][yoff + i][xoff];
                int avg = (sum + (w + h - 1) / 2) / (w + h - 1);
                int flipSign = (avg < 32);
                for (int i = 0; i < h; i++) {
                    for (int j = 0; j < w; j++) {
                        WedgeMasks[bsize][flipSign][wedge][i][j] = MasterMask[dir][yoff + i][xoff + j];
                        WedgeMasks[bsize][!flipSign][wedge][i][j] = 64 - MasterMask[dir][yoff + i][xoff + j];
                    }
                }
            }
        }
    }
    inited = true;
}

void Block::InterPredict::wedgeMask(int w, int h)
{
    initialise_wedge_mask_table();
    w <<= subX;
    h <<= subY;
    Mask.assign(h, std::vector<uint8_t>(w));
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            Mask[i][j] = WedgeMasks[m_block.MiSize][m_block.wedge_sign][m_block.wedge_index][i][j];
        }
    }
}

void Block::InterPredict::differenceWeightMask(int w, int h)
{
    Mask.assign(h, std::vector<uint8_t>(w));
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int16_t diff = std::abs(preds[0][i][j] - preds[1][i][j]);
            diff = ROUND2(diff, (m_sequence.BitDepth - 8) + InterPostRound);
            uint8_t m = CLIP3(0, 64, 38 + diff / 16);
            if (m_block.mask_type)
                Mask[i][j] = 64 - m;
            else
                Mask[i][j] = m;
        }
    }
}

void Block::InterPredict::getDistanceWeights(int candRow, int candCol, int& FwdWeight, int& BckWeight)
{
    const static uint8_t Quant_Dist_Weight[4][2] = {
        { 2, 3 },
        { 2, 5 },
        { 2, 7 },
        { 1, MAX_FRAME_DISTANCE }
    };
    const static uint8_t Quant_Dist_Lookup[4][2] = {
        { 9, 7 },
        { 11, 5 },
        { 12, 4 },
        { 13, 3 },
    };
    int refList = 0;
    int dist[2];
    for (refList = 0; refList < 2; refList++) {
        uint8_t ref = m_frame.RefFrames[candRow][candCol][refList];
        int d = std::abs(m_frame.get_relative_dist(ref));
        dist[refList] = CLIP3(0, MAX_FRAME_DISTANCE, d);
    }
    int d0 = dist[1];
    int d1 = dist[0];
    int order = d0 <= d1;
    if (d0 == 0 || d1 == 0) {
        FwdWeight = Quant_Dist_Lookup[3][order];
        BckWeight = Quant_Dist_Lookup[3][1 - order];
    } else {
        int i;
        for (i = 0; i < 3; i++) {
            int c0 = Quant_Dist_Weight[i][order];
            int c1 = Quant_Dist_Weight[i][1 - order];
            if (order) {
                if (d0 * c0 > d1 * c1)
                    break;
            } else {
                if (d0 * c0 < d1 * c1)
                    break;
            }
        }
        FwdWeight = Quant_Dist_Lookup[i][order];
        BckWeight = Quant_Dist_Lookup[i][1 - order];
    }
}

void Block::InterPredict::predict_inter(int x, int y, int w, int h, int candRow, int candCol)
{
    isCompound = m_frame.RefFrames[candRow][candCol][1] > INTRA_FRAME;
    roundingVariablesDerivation(isCompound, m_sequence.BitDepth, InterRound0, InterRound1, InterPostRound);
    if (!plane && m_block.motion_mode == LOCALWARP) {
        m_localWarp.warpEstimation();
        m_localWarp.setupShear();
    }
    int refList = 0;
    for (refList = 0; refList < 1 + isCompound; refList++) {
        int refFrame = m_frame.RefFrames[candRow][candCol][refList];
        if ((m_block.YMode == GLOBALMV || m_block.YMode == GLOBAL_GLOBALMV) && m_frame.GmType[refFrame] > TRANSLATION) {
            int alpha, beta, gamma, delta;
            globaValid = m_localWarp.setupShear(m_frame.gm_params[refFrame], alpha, beta, gamma, delta);
        }
        uint8_t useWarp = getUseWarp(w, h, refFrame);
        Mv mv = m_frame.Mvs[candRow][candCol][refList];
        int8_t refIdx;
        if (!m_block.use_intrabc) {
            refIdx = m_frame.ref_frame_idx[refFrame - LAST_FRAME];
        } else {
            refIdx = NONE_FRAME;
            auto& ref = m_frame.m_refInfo.m_refs[NONE_FRAME];
            ref.RefFrameWidth = m_frame.FrameWidth;
            ref.RefFrameHeight = m_frame.FrameHeight;
            ref.RefUpscaledWidth = m_frame.UpscaledWidth;
        }
        motionVectorScaling(refIdx, x, y, mv);

        if (m_block.use_intrabc) {
            auto& ref = m_frame.m_refInfo.m_refs[NONE_FRAME];
            ref.RefFrameWidth = m_frame.MiCols * MI_SIZE;
            ref.RefFrameHeight = m_frame.MiRows * MI_SIZE;
            ref.RefUpscaledWidth = m_frame.MiCols * MI_SIZE;
        }
        if (useWarp) {
            std::vector<std::vector<int16_t>>& pred = preds[refList];
            pred.assign(h, std::vector<int16_t>(w));
            for (int i8 = 0; i8 <= ((h - 1) >> 3); i8++) {
                for (int j8 = 0; j8 <= ((w - 1) >> 3); j8++) {
                    blockWarp(useWarp, refIdx, refList, x, y, i8, j8, w, h);
                }
            }
        } else {
            blockInterPrediction(refIdx, refList, w, h, candRow, candCol);
        }
    }
    COMPOUND_TYPE compound_type = m_block.compound_type;
    if (compound_type == COMPOUND_WEDGE) {
        wedgeMask(w, h);
    } else if (compound_type == COMPOUND_INTRA) {
        intraModeVariantMask(w, h);
    } else if (compound_type == COMPOUND_DIFFWTD && plane == 0) {
        differenceWeightMask(w, h);
    }
    int FwdWeight, BckWeight;
    if (compound_type == COMPOUND_DISTANCE) {
        getDistanceWeights(candRow, candCol, FwdWeight, BckWeight);
    }

    bool IsInterIntra = (m_block.is_inter && m_block.RefFrame[1] == INTRA_FRAME);
    if (!isCompound && !IsInterIntra) {
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                m_yuv.setPixel(plane, x + j, y + i, CLIP1(preds[0][i][j]));
            }
        }
    } else if (compound_type == COMPOUND_AVERAGE) {
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                m_yuv.setPixel(plane, x + j, y + i, CLIP1(ROUND2(preds[0][i][j] + preds[1][i][j], 1 + InterPostRound)));
            }
        }
    } else if (compound_type == COMPOUND_DISTANCE) {
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                m_yuv.setPixel(plane, x + j, y + i, CLIP1(ROUND2(FwdWeight * preds[0][i][j] + BckWeight * preds[1][i][j], 4 + InterPostRound)));
            }
        }
    } else {
        //if (compound_type != COMPOUND_WEDGE || plane == 0)
        maskBlend(x, y, w, h);
    }
    if (m_block.motion_mode == OBMC_CAUSAL) {
        overlappedMotionCompensation(w, h);
    }
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

bool Block::FindMvStack::check_sb_border(int deltaRow, int deltaCol) const
{
    int row = (MiRow & 15) + deltaRow;
    int col = (MiCol & 15) + deltaCol;

    return (row >= 0 && row < 16 && col >= 0 && col < 16);
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
    bool allowExtension = ((bh4 >= Num_4x4_Blocks_High[BLOCK_8X8])
        && (bh4 < Num_4x4_Blocks_High[BLOCK_64X64])
        && (bw4 >= Num_4x4_Blocks_Wide[BLOCK_8X8])
        && (bw4 < Num_4x4_Blocks_Wide[BLOCK_64X64]));
    if (allowExtension) {
        const int tplSamplePos[3][2] = {
            { bh4, -2 }, { bh4, bw4 }, { bh4 - 2, bw4 }
        };

        for (int i = 0; i < 3; i++) {
            int deltaRow = tplSamplePos[i][0];
            int deltaCol = tplSamplePos[i][1];
            if (check_sb_border(deltaRow, deltaCol)) {
                add_tpl_ref_mv(deltaRow, deltaCol);
            }
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
        typ = m_frame.GmType[ref];
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
    if (m_frame.allow_high_precision_mv)
        return;
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
    int end4 = std::min(std::min(bh4, m_frame.MiRows - m_block.MiRow), 16);
    bool useStep16 = (bh4 >= 16);
    if (std::abs(deltaCol) > 1) {
        deltaRow = 1 - (MiRow & 1);
        deltaCol += MiCol & 1;
    }
    uint32_t i = 0;
    while (i < end4) {
        int mvRow = MiRow + deltaRow + i;
        int mvCol = MiCol + deltaCol;
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
void Block::FindMvStack::searchStack(int mvRow, int mvCol, int candList, uint32_t weight)
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
    int mvRow = MiRow + deltaRow;
    int mvCol = MiCol + deltaCol;
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
void Block::FindMvStack::searchCompoundStack(int mvRow, int mvCol, uint32_t weight)
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
        WeightStack[NumMvFound] = weight;
        NumMvFound++;
    }
}
void Block::FindMvStack::add_ref_mv_candidate(int mvRow, int mvCol, uint32_t weight)
{
    if (!m_frame.IsInters[mvRow][mvCol])
        return;
    if (!isCompound) {
        for (int candList = 0; candList <= 1; candList++) {
            if (m_frame.RefFrames[mvRow][mvCol][candList] == m_block.RefFrame[0]) {
                searchStack(mvRow, mvCol, candList, weight);
            }
        }
    } else {
        if (m_frame.RefFrames[mvRow][mvCol][0] == m_block.RefFrame[0]
            && m_frame.RefFrames[mvRow][mvCol][1] == m_block.RefFrame[1]) {
            searchCompoundStack(mvRow, mvCol, weight);
        }
    }
}
void Block::FindMvStack::scanRow(int deltaRow)
{
    int deltaCol = 0;
    int end4 = std::min(std::min(bw4, m_frame.MiCols - m_block.MiCol), 16);
    bool useStep16 = (bw4 >= 16);
    if (std::abs(deltaRow) > 1) {
        deltaRow += MiRow & 1;
        deltaCol = 1 - (MiCol & 1);
    }
    uint32_t i = 0;
    while (i < end4) {
        int mvRow = MiRow + deltaRow;
        int mvCol = MiCol + deltaCol + i;
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
