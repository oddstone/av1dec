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

#include "IntraPredict.h"
#include "Parser.h"
#include "VideoFrame.h"
#include "log.h"

#include <numeric>

namespace YamiAv1 {

Block::IntraPredict::IntraPredict(const Block& block, const std::shared_ptr<YuvFrame>& yuv,
    int p, int startX, int startY, int log2w, int log2h,
    std::vector<std::vector<uint8_t>>& pred)
    : m_block(block)
    , m_tile(block.m_tile)
    , m_frame(block.m_frame)
    , m_sequence(block.m_sequence)
    , plane(p)
    , x(startX)
    , y(startY)
    , log2W(log2w)
    , log2H(log2h)
    , w(1 << log2w)
    , h(1 << log2h)
    , m_yuv(yuv)
    , m_pred(pred)
{
}

static const int INTRA_FILTER_SCALE_BITS = 4;

static const int INTRA_FILTER_MODES = 5;

int Intra_Filter_Taps[INTRA_FILTER_MODES][8][7] = {
    {
        { -6, 10, 0, 0, 0, 12, 0 },
        { -5, 2, 10, 0, 0, 9, 0 },
        { -3, 1, 1, 10, 0, 7, 0 },
        { -3, 1, 1, 2, 10, 5, 0 },
        { -4, 6, 0, 0, 0, 2, 12 },
        { -3, 2, 6, 0, 0, 2, 9 },
        { -3, 2, 2, 6, 0, 2, 7 },
        { -3, 1, 2, 2, 6, 3, 5 },
    },
    {
        { -10, 16, 0, 0, 0, 10, 0 },
        { -6, 0, 16, 0, 0, 6, 0 },
        { -4, 0, 0, 16, 0, 4, 0 },
        { -2, 0, 0, 0, 16, 2, 0 },
        { -10, 16, 0, 0, 0, 0, 10 },
        { -6, 0, 16, 0, 0, 0, 6 },
        { -4, 0, 0, 16, 0, 0, 4 },
        { -2, 0, 0, 0, 16, 0, 2 },
    },
    {
        { -8, 8, 0, 0, 0, 16, 0 },
        { -8, 0, 8, 0, 0, 16, 0 },
        { -8, 0, 0, 8, 0, 16, 0 },
        { -8, 0, 0, 0, 8, 16, 0 },
        { -4, 4, 0, 0, 0, 0, 16 },
        { -4, 0, 4, 0, 0, 0, 16 },
        { -4, 0, 0, 4, 0, 0, 16 },
        { -4, 0, 0, 0, 4, 0, 16 },
    },
    {
        { -2, 8, 0, 0, 0, 10, 0 },
        { -1, 3, 8, 0, 0, 6, 0 },
        { -1, 2, 3, 8, 0, 4, 0 },
        { 0, 1, 2, 3, 8, 2, 0 },
        { -1, 4, 0, 0, 0, 3, 10 },
        { -1, 3, 4, 0, 0, 4, 6 },
        { -1, 2, 3, 4, 0, 4, 4 },
        { -1, 2, 2, 3, 4, 3, 3 },
    },
    {
        { -12, 14, 0, 0, 0, 14, 0 },
        { -10, 0, 14, 0, 0, 12, 0 },
        { -9, 0, 0, 14, 0, 11, 0 },
        { -8, 0, 0, 0, 14, 10, 0 },
        { -10, 12, 0, 0, 0, 0, 14 },
        { -9, 1, 12, 0, 0, 0, 12 },
        { -8, 0, 0, 12, 0, 1, 11 },
        { -7, 0, 0, 1, 12, 1, 9 },
    }
};

void Block::IntraPredict::recursiveIntraPrediction(const uint8_t* AboveRow, const uint8_t* LeftCol,
    const std::shared_ptr<YuvFrame>& frame)
{
    const int w4 = w >> 2;
    const int h2 = h >> 1;

    for (int i2 = 0; i2 < h2; i2++) {
        for (int j4 = 0; j4 < w4; j4++) {
            uint8_t p[7];
            for (int i = 0; i < 5; i++) {
                if (!i2) {
                    p[i] = AboveRow[(j4 << 2) + i - 1];
                } else if (!j4 && !i) {
                    p[i] = LeftCol[(i2 << 1) - 1];
                } else {
                    p[i] = m_pred[(i2 << 1) - 1][(j4 << 2) + i - 1];
                }
            }
            for (int i = 5; i < 7; i++) {
                if (!j4) {
                    p[i] = LeftCol[(i2 << 1) + i - 5];
                } else {
                    p[i] = m_pred[(i2 << 1) + i - 5][(j4 << 2) - 1];
                }
            }
            for (int i1 = 0; i1 < 2; i1++) {
                for (int j1 = 0; j1 < 4; j1++) {
                    int pr = 0;
                    for (int i = 0; i < 7; i++) {
                        pr += Intra_Filter_Taps[m_block.filter_intra_mode][(i1 << 2) + j1][i] * p[i];
                    }
                    uint8_t p = CLIP1(ROUND2SIGNED(pr, INTRA_FILTER_SCALE_BITS));
                    m_pred[(i2 << 1) + i1][(j4 << 2) + j1] = p;
                }
            }
        }
    }
}

void Block::IntraPredict::paethPredict(const uint8_t* AboveRow, const uint8_t* LeftCol)
{

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int base = AboveRow[j] + LeftCol[i] - AboveRow[-1];
            int pLeft = std::abs(base - LeftCol[i]);
            int pTop = std::abs(base - AboveRow[j]);
            int pTopLeft = std::abs(base - AboveRow[-1]);
            uint8_t p;
            if (pLeft <= pTop && pLeft <= pTopLeft) {
                p = LeftCol[i];
            } else if (pTop <= pTopLeft) {
                p = AboveRow[j];
            } else {
                p = AboveRow[-1];
            }
            m_pred[i][j] = p;
        }
    }
}

static const int Mode_To_Angle[INTRA_MODES] = { 0, 90, 180, 45, 135, 113, 157, 203, 67, 0, 0, 0, 0 };

static const int Dr_Intra_Derivative[90] = {
    0, 0, 0, 1023, 0, 0, 547, 0, 0, 372, 0, 0, 0, 0,
    273, 0, 0, 215, 0, 0, 178, 0, 0, 151, 0, 0, 132, 0, 0,
    116, 0, 0, 102, 0, 0, 0, 90, 0, 0, 80, 0, 0, 71, 0, 0,
    64, 0, 0, 57, 0, 0, 51, 0, 0, 45, 0, 0, 0, 40, 0, 0,
    35, 0, 0, 31, 0, 0, 27, 0, 0, 23, 0, 0, 19, 0, 0,
    15, 0, 0, 0, 0, 11, 0, 0, 7, 0, 0, 3, 0, 0
};

static int getDx(int pAngle)
{
    if (pAngle < 90)
        return Dr_Intra_Derivative[pAngle];
    else if (pAngle > 90 && pAngle < 180)
        return Dr_Intra_Derivative[180 - pAngle];
    ASSERT(0);
    return 0;
}

static int getDy(int pAngle)
{
    if (pAngle > 90 && pAngle < 180)
        return Dr_Intra_Derivative[pAngle - 90];
    else if (pAngle > 180)
        return Dr_Intra_Derivative[270 - pAngle];
    ASSERT(0);
    return 0;
}

static void filterCorner(uint8_t* LeftCol, uint8_t* AboveRow)
{
    uint8_t s = ROUND2(LeftCol[0] * 5 + AboveRow[-1] * 6 + AboveRow[0] * 5, 4);
    LeftCol[-1] = s;
    AboveRow[-1] = s;
}

bool Block::IntraPredict::getLeftSmooth() const
{
    bool avail = !plane ? m_block.AvailL : m_block.AvailLChroma;
    bool leftSmooth = false;
    if (avail) {
        int r = m_block.MiRow;
        int c = m_block.MiCol - 1;
        if (plane > 0) {
            if (m_sequence.subsampling_x && (m_block.MiCol & 1))
                c--;
            if (m_sequence.subsampling_y && !(m_block.MiRow & 1))
                r++;
        }
        leftSmooth = getSmooth(r, c);
    }
    return leftSmooth;
}

bool Block::IntraPredict::getAboveSmooth() const
{
    bool avail = !plane ? m_block.AvailU : m_block.AvailUChroma;
    bool aboveSmooth = false;
    if (avail) {
        int r = m_block.MiRow - 1;
        int c = m_block.MiCol;
        if (plane > 0) {
            if (m_sequence.subsampling_x && !(m_block.MiCol & 1))
                c++;
            if (m_sequence.subsampling_y && (m_block.MiRow & 1))
                r--;
        }
        aboveSmooth = getSmooth(r, c);
    }
    return aboveSmooth;
}

bool Block::IntraPredict::getSmooth(int r, int c) const
{
    const ModeInfoBlock& info = m_block.getModeInfo(r, c);

    int mode;
    if (!plane) {
        mode = info.YMode;
    } else {
        if (info.RefFrames[0] > INTRA_FRAME)
            return 0;
        mode = info.UVMode;
    }
    return (mode == SMOOTH_PRED || mode == SMOOTH_V_PRED || mode == SMOOTH_H_PRED);
}

bool Block::IntraPredict::get_filter_type(bool haveLeft, bool haveAbove) const
{
    bool aboveSmooth = getAboveSmooth();
    bool leftSmooth = getLeftSmooth();
    return aboveSmooth || leftSmooth;
}

uint8_t getIntraEdgeFilterStrength(int w, int h, bool filterType, int delta)
{
    int d = std::abs(delta);
    int blkWh = w + h;
    uint8_t strength = 0;
    if (!filterType) {
        if (blkWh <= 8) {
            if (d >= 56)
                strength = 1;
        } else if (blkWh <= 12) {
            if (d >= 40)
                strength = 1;
        } else if (blkWh <= 16) {
            if (d >= 40)
                strength = 1;
        } else if (blkWh <= 24) {
            if (d >= 8)
                strength = 1;
            if (d >= 16)
                strength = 2;
            if (d >= 32)
                strength = 3;
        } else if (blkWh <= 32) {
            strength = 1;
            if (d >= 4)
                strength = 2;
            if (d >= 32)
                strength = 3;
        } else {
            strength = 3;
        }
    } else {
        if (blkWh <= 8) {
            if (d >= 40)
                strength = 1;
            if (d >= 64)
                strength = 2;
        } else if (blkWh <= 16) {
            if (d >= 20)
                strength = 1;
            if (d >= 48)
                strength = 2;
        } else if (blkWh <= 24) {
            if (d >= 4)
                strength = 3;
        } else {
            strength = 3;
        }
    }
    return strength;
}

const static int INTRA_EDGE_KERNELS = 3;
const static int INTRA_EDGE_TAPS = 5;

uint8_t Intra_Edge_Kernel[INTRA_EDGE_KERNELS][INTRA_EDGE_TAPS] = {
    { 0, 4, 8, 4, 0 },
    { 0, 5, 6, 5, 0 },
    { 2, 4, 4, 4, 2 }
};

static void intraEdgeFilter(int sz, int strength, uint8_t* array)
{
    if (!strength)
        return;
    std::vector<uint8_t> edge(array - 1, array - 1 + sz);
    for (int i = 1; i < sz; i++) {
        int s = 0;
        for (int j = 0; j < INTRA_EDGE_TAPS; j++) {
            int k = CLIP3(0, sz - 1, i - 2 + j);
            s += Intra_Edge_Kernel[strength - 1][j] * edge[k];
        }
        array[i - 1] = (s + 8) >> 4;
    }
}

static int getIntraEdgeUpsample(int w, int h, int filterType, int delta)
{
    int blkWh = w + h;
    int d = std::abs(delta);
    int useUpsample;
    if (d <= 0 || d >= 40) {
        useUpsample = 0;
    } else if (filterType == 0) {
        useUpsample = (blkWh <= 16);
    } else {
        useUpsample = (blkWh <= 8);
    }
    return useUpsample;
}

uint8_t* Block::IntraPredict::intraEdgeUpsample(const uint8_t* edge, int numPx, std::vector<uint8_t>& upsampled) const
{
    std::vector<uint8_t> dup;
    dup.reserve(numPx + 3);
    dup.push_back(edge[-1]);
    dup.insert(dup.end(), edge - 1, edge + numPx);
    dup.push_back(edge[numPx - 1]);

    upsampled.resize(2 * numPx + 1);
    uint8_t* buf = &upsampled[2];
    buf[-2] = dup[0];
    for (int i = 0; i < numPx; i++) {
        int s = -dup[i] + (9 * dup[i + 1]) + (9 * dup[i + 2]) - dup[i + 3];
        buf[2 * i - 1] = CLIP1(ROUND2(s, 4));
        buf[2 * i] = dup[i + 2];
    }
    return buf;
}

void Block::IntraPredict::directionalIntraPredict(
    bool haveAbove, bool haveLeft, uint8_t* AboveRow, uint8_t* LeftCol,
    int mode)
{
    int subX = (plane > 0) ? m_sequence.subsampling_x : 0;
    int subY = (plane > 0) ? m_sequence.subsampling_y : 0;
    int maxX = (m_frame.MiCols * MI_SIZE) >> subX;
    int maxY = (m_frame.MiRows * MI_SIZE) >> subY;
    int angleDelta = plane == 0 ? m_block.AngleDeltaY : m_block.AngleDeltaUV;
    int pAngle = Mode_To_Angle[mode] + angleDelta * ANGLE_STEP;
    int upsampleAbove = 0;
    int upsampleLeft = 0;
    std::vector<uint8_t> upsampledAbove;
    std::vector<uint8_t> upsampledLeft;
    if (m_sequence.enable_intra_edge_filter) {
        if (pAngle != 90 && pAngle != 180) {
            if (pAngle > 90 && pAngle < 180 && (w + h) >= 24) {
                filterCorner(LeftCol, AboveRow);
            }
            bool filterType = get_filter_type(haveLeft, haveAbove);
            if (haveAbove) {
                int strength = getIntraEdgeFilterStrength(w, h, filterType, pAngle - 90);
                int numPx = std::min(w, (maxX - x + 1)) + (pAngle < 90 ? h : 0) + 1;
                intraEdgeFilter(numPx, strength, AboveRow);
            }
            if (haveLeft) {
                int strength = getIntraEdgeFilterStrength(w, h, filterType, pAngle - 180);
                int numPx = std::min(h, (maxY - y + 1)) + (pAngle > 180 ? w : 0) + 1;
                intraEdgeFilter(numPx, strength, LeftCol);
            }
            upsampleAbove = getIntraEdgeUpsample(w, h, filterType, pAngle - 90);
            int numPx = (w + (pAngle < 90 ? h : 0));
            if (upsampleAbove) {
                AboveRow = intraEdgeUpsample(AboveRow, numPx, upsampledAbove);
            }
            upsampleLeft = getIntraEdgeUpsample(w, h, filterType, pAngle - 180);
            numPx = (h + (pAngle > 180 ? w : 0));
            if (upsampleLeft) {
                LeftCol = intraEdgeUpsample(LeftCol, numPx, upsampledLeft);
            }
        }
    }
    if (pAngle < 90) {
        int dx = getDx(pAngle);
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                int idx = (i + 1) * dx;
                int base = (idx >> (6 - upsampleAbove)) + (j << upsampleAbove);
                int shift = ((idx << upsampleAbove) >> 1) & 0x1F;
                int maxBaseX = (w + h - 1) << upsampleAbove;
                uint8_t p;
                if (base < maxBaseX) {
                    p = ROUND2(AboveRow[base] * (32 - shift) + AboveRow[base + 1] * shift, 5);
                } else {
                    p = AboveRow[maxBaseX];
                }
                m_pred[i][j] = p;
            }
        }
    } else if (pAngle > 90 && pAngle < 180) {
        int dx = getDx(pAngle);
        int dy = getDy(pAngle);
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                int idx = (j << 6) - (i + 1) * dx;
                int base = idx >> (6 - upsampleAbove);
                int shift;
                uint8_t p;
                if (base >= -(1 << upsampleAbove)) {
                    shift = ((idx << upsampleAbove) >> 1) & 0x1F;
                    p = ROUND2(AboveRow[base] * (32 - shift) + AboveRow[base + 1] * shift, 5);
                } else {
                    idx = (i << 6) - (j + 1) * dy;
                    base = idx >> (6 - upsampleLeft);
                    shift = ((idx << upsampleLeft) >> 1) & 0x1F;
                    p = ROUND2(LeftCol[base] * (32 - shift) + LeftCol[base + 1] * shift, 5);
                }
                m_pred[i][j] = p;
            }
        }
    } else if (pAngle > 180) {
        int dy = getDy(pAngle);
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                int idx = (j + 1) * dy;
                int base = (idx >> (6 - upsampleLeft)) + (i << upsampleLeft);
                int shift = ((idx << upsampleLeft) >> 1) & 0x1F;
                uint8_t p = ROUND2(LeftCol[base] * (32 - shift) + LeftCol[base + 1] * shift, 5);
                m_pred[i][j] = p;
            }
        }
    } else if (pAngle == 90) {
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                m_pred[i][j] = AboveRow[j];
            }
        }
    } else if (pAngle == 180) {
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                m_pred[i][j] = LeftCol[i];
            }
        }
    }
}

void Block::IntraPredict::dcPredict(bool haveAbove, bool haveLeft,
    const uint8_t* AboveRow, const uint8_t* LeftCol)
{
    uint8_t avg;
    int sum = 0;
    if (haveLeft && haveAbove) {
        sum = std::accumulate(LeftCol, LeftCol + h, sum);
        sum = std::accumulate(AboveRow, AboveRow + w, sum);
        avg = (uint8_t)((sum + ((w + h) >> 1)) / (w + h));
    } else if (haveLeft) {
        sum = std::accumulate(LeftCol, LeftCol + h, sum);
        avg = CLIP1((sum + (h >> 1)) >> log2H);
    } else if (haveAbove) {
        sum = std::accumulate(AboveRow, AboveRow + w, sum);
        avg = CLIP1((sum + (w >> 1)) >> log2W);
    } else {
        avg = (1 << (m_sequence.BitDepth - 1));
    }
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            m_pred[i][j] = avg;
        }
    }
}

static const int Sm_Weights_Tx_4x4[4] = { 255, 149, 85, 64 };
static const int Sm_Weights_Tx_8x8[8] = { 255, 197, 146, 105, 73, 50, 37, 32 };
static const int Sm_Weights_Tx_16x16[16] = { 255, 225, 196, 170, 145, 123, 102, 84, 68, 54, 43, 33, 26, 20, 17, 16 };
static const int Sm_Weights_Tx_32x32[32] = { 255, 240, 225, 210, 196, 182, 169, 157, 145, 133, 122, 111, 101, 92, 83, 74,
    66, 59, 52, 45, 39, 34, 29, 25, 21, 17, 14, 12, 10, 9, 8, 8 };
static const int Sm_Weights_Tx_64x64[64] = { 255, 248, 240, 233, 225, 218, 210, 203, 196, 189, 182, 176, 169, 163, 156, 150,
    144, 138, 133, 127, 121, 116, 111, 106, 101, 96, 91, 86, 82, 77, 73,
    69, 65, 61, 57, 54, 50, 47, 44, 41, 38, 35, 32, 29, 27, 25, 22, 20, 18, 16, 15,
    13, 12, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4 };

const int* getSmWeights(int log)
{
    const int(*weights[]) = { Sm_Weights_Tx_4x4, Sm_Weights_Tx_8x8, Sm_Weights_Tx_16x16, Sm_Weights_Tx_32x32, Sm_Weights_Tx_64x64 };
    return weights[log - 2];
}

void Block::IntraPredict::smoothPredict(const uint8_t* AboveRow, const uint8_t* LeftCol)
{
    const int* smWeightsX = getSmWeights(log2W);
    const int* smWeightsY = getSmWeights(log2H);
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int smoothPred = smWeightsY[i] * AboveRow[j]
                + (256 - smWeightsY[i]) * LeftCol[h - 1]
                + smWeightsX[j] * LeftCol[i]
                + (256 - smWeightsX[j]) * AboveRow[w - 1];
            m_pred[i][j] = ROUND2(smoothPred, 9);
        }
    }
}

void Block::IntraPredict::smoothVPredict(const uint8_t* AboveRow, const uint8_t* LeftCol)
{
    const int* smWeights = getSmWeights(log2H);
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int smoothPred = smWeights[i] * AboveRow[j] + (256 - smWeights[i]) * LeftCol[h - 1];
            m_pred[i][j] = ROUND2(smoothPred, 8);
        }
    }
}

void Block::IntraPredict::smoothHPredict(const uint8_t* AboveRow, const uint8_t* LeftCol)
{
    const int* smWeights = getSmWeights(log2W);
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int smoothPred = smWeights[j] * LeftCol[i] + (256 - smWeights[j]) * AboveRow[w - 1];
            m_pred[i][j] = ROUND2(smoothPred, 8);
        }
    }
}

void Block::IntraPredict::predict_intra(int haveLeft, int haveAbove, bool haveAboveRight, bool haveBelowLeft,
    int mode)
{
    int subX = (plane > 0) ? m_sequence.subsampling_x : 0;
    int subY = (plane > 0) ? m_sequence.subsampling_x : 0;
    int maxX = ((m_frame.MiCols * MI_SIZE) >> subX) - 1;
    int maxY = ((m_frame.MiRows * MI_SIZE) >> subY) - 1;

    uint8_t* AboveRow;
    uint8_t* LeftCol;
    std::vector<uint8_t> aboveRow;
    aboveRow.resize(w + h + 1);
    AboveRow = &aboveRow[1];

    std::vector<uint8_t> leftCol;
    leftCol.resize(w + h + 1);
    LeftCol = &leftCol[1];

    uint8_t median = (1 << (m_sequence.BitDepth - 1));
    if (!haveAbove && haveLeft) {
        std::fill(aboveRow.begin() + 1, aboveRow.end(), m_yuv->getPixel(plane, x - 1, y));
    } else if (!haveAbove && !haveLeft) {
        std::fill(aboveRow.begin() + 1, aboveRow.end(), median - 1);
    } else {
        int aboveLimit = std::min(maxX, x + (haveAboveRight ? 2 * w : w) - 1);
        for (int i = 0; i < w + h; i++) {
            AboveRow[i] = m_yuv->getPixel(plane, std::min(aboveLimit, x + i), y - 1);
        }
    }
    if (!haveLeft && haveAbove) {
        std::fill(leftCol.begin() + 1, leftCol.end(), m_yuv->getPixel(plane, x, y - 1));
    } else if (!haveAbove && !haveLeft) {
        std::fill(leftCol.begin() + 1, leftCol.end(), median + 1);
    } else {
        int leftLimit = std::min(maxY, y + (haveBelowLeft ? 2 * h : h) - 1);
        for (int i = 0; i < w + h; i++) {
            LeftCol[i] = m_yuv->getPixel(plane, x - 1, std::min(leftLimit, y + i));
        }
    }
    if (haveAbove && haveLeft) {
        AboveRow[-1] = m_yuv->getPixel(plane, x - 1, y - 1);
    } else if (haveAbove) {
        AboveRow[-1] = m_yuv->getPixel(plane, x, y - 1);
    } else if (haveLeft) {
        AboveRow[-1] = m_yuv->getPixel(plane, x - 1, y);
    } else {
        AboveRow[-1] = median;
    }
    LeftCol[-1] = AboveRow[-1];
    m_pred.assign(h, std::vector<uint8_t>(w));
    if (plane == 0 && m_block.use_filter_intra) {
        recursiveIntraPrediction(AboveRow, LeftCol, m_yuv);
    } else if (is_directional_mode(mode)) {
        directionalIntraPredict(haveAbove, haveLeft, AboveRow, LeftCol, mode);
    } else if (mode == PAETH_PRED) {
        paethPredict(AboveRow, LeftCol);
    } else if (mode == DC_PRED) {
        dcPredict(haveAbove, haveLeft, AboveRow, LeftCol);
    } else if (mode == SMOOTH_PRED) {
        smoothPredict(AboveRow, LeftCol);
    } else if (mode == SMOOTH_V_PRED) {
        smoothVPredict(AboveRow, LeftCol);
    } else if (mode == SMOOTH_H_PRED) {
        smoothHPredict(AboveRow, LeftCol);
    } else {
        ASSERT(0 && "not PAETH_PRED");
    }
}

void Block::IntraPredict::predict_chroma_from_luma(TX_SIZE txSz)
{
    int subX = m_sequence.subsampling_x;
    int subY = m_sequence.subsampling_y;
    int startX = x;
    int startY = y;

    int8_t alpha = plane == 1 ? m_block.CflAlphaU : m_block.CflAlphaV;
    int lumaAvg = 0;
    std::vector<std::vector<int>> L(h, std::vector<int>(w));
    for (int i = 0; i < h; i++) {
        int lumaY = (startY + i) << subY;
        lumaY = std::min(lumaY, m_block.MaxLumaH - (1 << subY));
        for (int j = 0; j < w; j++) {
            int lumaX = (startX + j) << subX;
            lumaX = std::min(lumaX, m_block.MaxLumaW - (1 << subX));
            int t = 0;
            for (int dy = 0; dy <= subY; dy += 1) {
                for (int dx = 0; dx <= subX; dx += 1) {
                    t += m_yuv->getPixel(0, lumaX + dx, lumaY + dy);
                }
            }
            int v = t << (3 - subX - subY);
            L[i][j] = v;
            lumaAvg += v;
        }
    }
    lumaAvg = ROUND2(lumaAvg, log2W + log2H);
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            uint8_t dc = m_pred[i][j];
            int scaledLuma = ROUND2SIGNED(alpha * (L[i][j] - lumaAvg), 6);
            m_pred[i][j] = CLIP1(dc + scaledLuma);
        }
    }
}

}
