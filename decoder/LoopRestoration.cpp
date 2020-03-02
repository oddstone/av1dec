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

#include "LoopRestoration.h"
#include "Parser.h"
#include "log.h"

namespace YamiAv1 {

static int count_units_in_frame(int unitSize, int frameSize)
{
    return std::max((frameSize + (unitSize >> 1)) / unitSize, 1);
}

LoopRestoration::LoopRestoration(const ConstFramePtr& frame,
    const std::shared_ptr<YuvFrame>& upscaledCdefFrame,
    const std::shared_ptr<YuvFrame>& upscaledCurrFrame)
    : m_frame(frame)
    , m_sequence(*m_frame->m_sequence)
    , m_loopRestoration(m_frame->m_loopRestoration)
    , UpscaledCdefFrame(upscaledCdefFrame)
    , UpscaledCurrFrame(upscaledCurrFrame)
{
}

struct LoopRestoration::PlaneInfo {
    int subX = 0;
    int subY = 0;
    int unitSize;
    int unitRows;
    int unitCols;
    int plane;
    int PlaneEndX;
    int PlaneEndY;
    PlaneInfo(LoopRestoration& lr, int p)
        : plane(p)
    {
        if (plane) {
            subX = lr.m_sequence.subsampling_x;
            subY = lr.m_sequence.subsampling_y;
        }
        unitSize = lr.m_loopRestoration.LoopRestorationSize[plane];
        unitRows = count_units_in_frame(unitSize, ROUND2(lr.m_frame->FrameHeight, subY));
        unitCols = count_units_in_frame(unitSize, ROUND2(lr.m_frame->UpscaledWidth, subX));
        PlaneEndX = ROUND2(lr.m_frame->UpscaledWidth, subX);
        PlaneEndY = ROUND2(lr.m_frame->FrameHeight, subY);
    }
};

struct LoopRestoration::UnitInfo {
    int x;
    int y;
    int w;
    int h;
    int row;
    int col;
    RestorationType type;
    UnitInfo(const PlaneInfo& plane, int unitRow, int unitCol, RestorationType rType)
        : type(rType)
    {
        const int lastRow = plane.unitRows - 1;
        const int lastCol = plane.unitCols - 1;
        const int unitSize = plane.unitSize;
        row = unitRow;
        col = unitCol;
        x = unitCol * unitSize;
        y = unitRow * unitSize;
        if (y)
            y -= 8 >> plane.subY;
        w = (unitCol == lastCol) ? plane.PlaneEndX - x : unitSize;
        if (unitRow == lastRow) {
            h = plane.PlaneEndY - y;
        } else {
            h = unitSize;
            if (!y) {
                //first row is 8 pixels short
                h -= 8 >> plane.subY;
            }
        }
    }
};

struct LoopRestoration::StripeInfo {
    //strip start and end
    int start;
    int end;

    StripeInfo(const PlaneInfo& plane, const UnitInfo& unit)
        : m_plane(plane)
        , m_unit(unit)
        , m_unitEnd(m_unit.y + m_unit.h)
    {
        int lumaY = unit.y << m_plane.subY;
        int stripeNum = (lumaY + 8) / 64;
        start = (-8 + stripeNum * 64) >> m_plane.subY;
        end = start + (64 >> m_plane.subY);
    }
    bool next()
    {
        if (end >= m_unitEnd)
            return false;
        start = end;
        end += (64 >> m_plane.subY);
        return true;
    }

private:
    const PlaneInfo& m_plane;
    const UnitInfo& m_unit;
    const int m_unitEnd;
};

void LoopRestoration::forEachStripe(std::shared_ptr<YuvFrame>& LrFrame, const PlaneInfo& plane, const UnitInfo& unit)
{
    StripeInfo stripe(plane, unit);
    do {
        forEachBlock(LrFrame, plane, unit, stripe);
    } while (stripe.next());
}

void LoopRestoration::forEachBlock(std::shared_ptr<YuvFrame>& LrFrame,
    const PlaneInfo& plane, const UnitInfo& unit, const StripeInfo& stripe)
{
    const int x = unit.x;
    const int y = std::max(stripe.start, unit.y);
    int width = unit.w;
    int height = std::min(stripe.end, unit.y + unit.h) - y;
    if (unit.type == RESTORE_WIENER) {
        wienerFilter(LrFrame, plane.plane, unit, x, y, width, height, stripe);
    } else if (unit.type == RESTORE_SGRPROJ) {
        selfGuidedFilter(LrFrame, plane.plane, unit, x, y, width, height, stripe);
    }
}

void LoopRestoration::forEachUnit(std::shared_ptr<YuvFrame>& LrFrame, const PlaneInfo& plane)
{
    for (int unitRow = 0; unitRow < plane.unitRows; unitRow++) {
        for (int unitCol = 0; unitCol < plane.unitCols; unitCol++) {
            RestorationType rType = m_frame->m_loopRestoration.LrType[plane.plane][unitRow][unitCol];
            if (rType != RESTORE_NONE) {
                UnitInfo unit(plane, unitRow, unitCol, rType);
                forEachStripe(LrFrame, plane, unit);
            }
        }
    }
}

void LoopRestoration::forEachPlane(std::shared_ptr<YuvFrame>& LrFrame)
{
    for (int p = 0; p < m_sequence.NumPlanes; p++) {
        PlaneInfo info(*this, p);
        if (m_loopRestoration.FrameRestorationType[p] != RESTORE_NONE) {
#if 1
            forEachUnit(LrFrame, info);
#else
            for (int y = 0; y < m_frame->FrameHeight; y += MI_SIZE) {
                for (int x = 0; x < m_frame->UpscaledWidth; x += MI_SIZE) {
                    int row = y >> MI_SIZE_LOG2;
                    int col = x >> MI_SIZE_LOG2;
                    loop_restore_block(LrFrame, p, row, col);
                }
            }
#endif
        }
    }
}

std::shared_ptr<YuvFrame> LoopRestoration::filter()
{
    if (!m_frame->m_loopRestoration.UsesLr) {
        return UpscaledCdefFrame;
    }
    static const int borders = 3;
    UpscaledCdefFrame->extendBorder(borders);
    UpscaledCurrFrame->extendBorder(borders);
    for (int p = 0; p < m_sequence.NumPlanes; p++) {
        int subX = 0;
        int subY = 0;
        if (p) {
            subX = m_sequence.subsampling_x;
            subY = m_sequence.subsampling_y;
        }
        unitSize[p] = m_loopRestoration.LoopRestorationSize[p];
        unitRows[p] = count_units_in_frame(unitSize[p], ROUND2(m_frame->FrameHeight, subY));
        unitCols[p] = count_units_in_frame(unitSize[p], ROUND2(m_frame->UpscaledWidth, subX));

        PlaneEndX[p] = ROUND2(m_frame->UpscaledWidth, subX) - 1;
        PlaneEndY[p] = ROUND2(m_frame->FrameHeight, subY) - 1;
    }
    roundingVariablesDerivation(0, m_sequence.BitDepth,
        InterRound0, InterRound1, InterPostRound);

    std::shared_ptr<YuvFrame> LrFrame = YuvFrame::create(UpscaledCdefFrame);
    forEachPlane(LrFrame);
    return LrFrame;
}

std::vector<int> getFilter(const std::vector<int8_t>& coeff)
{
    std::vector<int> filter(FILTER_BITS);
    filter[3] = 128;
    for (int i = 0; i < 3; i++) {
        int c = coeff[i];
        filter[i] = c;
        filter[6 - i] = c;
        filter[3] -= 2 * c;
    }
    return std::move(filter);
}

uint8_t LoopRestoration::get_source_sample(int plane, int x, int y,
    const StripeInfo& stripe)
{
    if (y < stripe.start) {
        y = std::max(stripe.start - 2, y);
        return UpscaledCurrFrame->getPixel(plane, x, y);
    } else if (y >= stripe.end) {
        y = std::min(stripe.end + 1, y);
        return UpscaledCurrFrame->getPixel(plane, x, y);
    } else {
        return UpscaledCdefFrame->getPixel(plane, x, y);
    }
}
void LoopRestoration::wienerFilter(const std::shared_ptr<YuvFrame>& LrFrame,
    int plane, const UnitInfo& unit, int x, int y, int w, int h, const StripeInfo& stripe)
{
    int BitDepth = m_sequence.BitDepth;
    std::vector<int> vfilter = getFilter(m_loopRestoration.LrWiener[plane][unit.row][unit.col][0]);
    std::vector<int> hfilter = getFilter(m_loopRestoration.LrWiener[plane][unit.row][unit.col][1]);
    std::vector<std::vector<int>> intermediate;
    intermediate.assign(h + 6, std::vector<int>(w));
    int offset = (1 << (BitDepth + FILTER_BITS - InterRound0 - 1));
    int limit = (1 << (BitDepth + 1 + FILTER_BITS - InterRound0)) - 1;
    for (int r = 0; r < h + 6; r++) {
        for (int c = 0; c < w; c++) {
            int s = 0;
            for (int t = 0; t < 7; t++) {
                s += hfilter[t] * get_source_sample(plane, x + c + t - 3, y + r - 3, stripe);
            }
            int v = ROUND2(s, InterRound0);
            intermediate[r][c] = CLIP3(-offset, limit - offset, v);
        }
    }
    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            int s = 0;
            for (int t = 0; t < 7; t++) {
                s += vfilter[t] * intermediate[r + t][c];
            }
            int v = ROUND2(s, InterRound1);
            LrFrame->setPixel(plane, x + c, y + r, CLIP1(v));
        }
    }
}

inline int& getValue(int* sum, int y, int x)
{
    return sum[(LoopRestoration::REST_BORDER + y) * LoopRestoration::MAX_REST_WIDTH + LoopRestoration::REST_BORDER + x];
}

void boxsum1(int* sum, int w, int h)
{
    for (int y = -2; y < h + 2; y++) {
        int a;
        int b = getValue(sum, y, -2);
        int c = getValue(sum, y, -1);
        for (int x = -1; x < w + 1; x++) {
            a = b;
            b = c;
            c = getValue(sum, y, x + 1);
            getValue(sum, y, x) = a + b + c;
        }
    }
    for (int x = -2; x < w + 2; x++) {
        int a;
        int b = getValue(sum, -2, x);
        int c = getValue(sum, -1, x);
        for (int y = -1; y < h + 1; y++) {
            a = b;
            b = c;
            c = getValue(sum, y + 1, x);
            getValue(sum, y, x) = a + b + c;
        }
    }
}

void boxsum2(int* sum, int w, int h)
{
    for (int y = -3; y < h + 3; y++) {
        int a;
        int b = getValue(sum, y, -3);
        int c = getValue(sum, y, -2);
        int d = getValue(sum, y, -1);
        int e = getValue(sum, y, 0);

        for (int x = -1; x < w + 1; x++) {
            a = b;
            b = c;
            c = d;
            d = e;
            e = getValue(sum, y, x + 2);
            getValue(sum, y, x) = a + b + c + d + e;
        }
    }
    for (int x = -3; x < w + 3; x++) {
        int a;
        int b = getValue(sum, -3, x);
        int c = getValue(sum, -2, x);
        int d = getValue(sum, -1, x);
        int e = getValue(sum, 0, x);
        for (int y = -1; y < h + 1; y++) {
            a = b;
            b = c;
            c = d;
            d = e;
            e = getValue(sum, y + 2, x);
            getValue(sum, y, x) = a + b + c + d + e;
        }
    }
}

void boxsum(int* sum, int w, int h, int r)
{
    if (r == 1)
        boxsum1(sum, w, h);
    else
        boxsum2(sum, w, h);
}

std::vector<std::vector<int>> LoopRestoration::boxFilter(int plane, int x, int y, int w, int h, uint8_t set, const StripeInfo& stripe, int pass, int r)
{
    uint8_t BitDepth = m_sequence.BitDepth;

    std::vector<std::vector<int>> F(h, std::vector<int>(w));
    int eps = Sgr_Params[set][pass * 2 + 1];
    std::vector<std::vector<int>> A(h + 2, std::vector<int>(w + 2));
    std::vector<std::vector<int>> B(h + 2, std::vector<int>(w + 2));

    int n = (2 * r + 1) * (2 * r + 1);
    int n2e = n * n * eps;
    int s = (((1 << SGRPROJ_MTABLE_BITS) + n2e / 2) / n2e);
    for (int i = -1; i < h + 1; i++) {
        for (int j = -1; j < w + 1; j++) {
#if 0
            int a = 0;
            int b = 0;
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    int c = get_source_sample(plane, x + j + dx, y + i + dy, stripe);
                    a += c * c;
                    b += c;
                }
            }
            ASSERT(sumA[REST_BORDER + i][REST_BORDER + j] == a);
            ASSERT(sumB[REST_BORDER + i][REST_BORDER + j] == b);
#else
            int a = getValue(&sumA[0][0], i, j);
            int b = getValue(&sumB[0][0], i, j);
#endif
            a = ROUND2(a, 2 * (BitDepth - 8));
            int d = ROUND2(b, BitDepth - 8);
            int p = std::max(0, a * n - d * d);
            int z = ROUND2(p * s, SGRPROJ_MTABLE_BITS);
            int a2;
            if (z >= 255)
                a2 = 256;
            else if (z == 0)
                a2 = 1;
            else
                a2 = ((z << SGRPROJ_SGR_BITS) + (z / 2)) / (z + 1);
            int oneOverN = ((1 << SGRPROJ_RECIP_BITS) + (n / 2)) / n;
            int b2 = ((1 << SGRPROJ_SGR_BITS) - a2) * b * oneOverN;
            A[i + 1][j + 1] = a2;
            B[i + 1][j + 1] = ROUND2(b2, SGRPROJ_RECIP_BITS);
        }
    }

    for (int i = 0; i < h; i++) {
        int shift = 5;
        if (pass == 0 && (i & 1)) {
            shift = 4;
        }
        for (int j = 0; j < w; j++) {
            int a = 0;
            int b = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int weight;
                    if (pass == 0) {
                        if ((i + dy) & 1) {
                            weight = (dx == 0) ? 6 : 5;
                        } else {
                            weight = 0;
                        }
                    } else {
                        weight = (dx == 0 || dy == 0) ? 4 : 3;
                    }
                    a += weight * A[i + dy + 1][j + dx + 1];
                    b += weight * B[i + dy + 1][j + dx + 1];
                }
            }
            int v = a * UpscaledCdefFrame->getPixel(plane, x + j, y + i) + b;
            F[i][j] = ROUND2(v, SGRPROJ_SGR_BITS + shift - SGRPROJ_RST_BITS);
        }
    }
    return std::move(F);
}

void LoopRestoration::computIntermedia(int plane, int x, int y, int w, int h, int r, const StripeInfo& stripe)
{
    for (int dy = -(r + 1); dy < h + r + 1; dy++) {
        for (int dx = -(r + 1); dx < w + r + 1; dx++) {
            int c = get_source_sample(plane, x + dx, y + dy, stripe);
            getValue(&sumA[0][0], dy, dx) = c * c;
            getValue(&sumB[0][0], dy, dx) = c;
        }
    }
    boxsum(&sumA[0][0], w, h, r);
    boxsum(&sumB[0][0], w, h, r);
}
void LoopRestoration::selfGuidedFilter(const std::shared_ptr<YuvFrame>& LrFrame,
    int plane, const UnitInfo& unit, int x, int y, int w, int h, const StripeInfo& stripe)
{
    uint8_t set = m_loopRestoration.LrSgrSet[plane][unit.row][unit.col];
    std::vector<std::vector<int>> flt0;
    std::vector<std::vector<int>> flt1;
    int r0 = Sgr_Params[set][0];
    if (r0) {
        computIntermedia(plane, x, y, w, h, r0, stripe);
        flt0 = boxFilter(plane, x, y, w, h, set, stripe, 0, r0);
    }
    int r1 = Sgr_Params[set][2];
    if (r1 && (r0 != r1) ) {
        computIntermedia(plane, x, y, w, h, r1, stripe);
        flt1 = boxFilter(plane, x, y, w, h, set, stripe, 1, r1);
    }
    int w0 = m_loopRestoration.LrSgrXqd[plane][unit.row][unit.col][0];
    int w1 = m_loopRestoration.LrSgrXqd[plane][unit.row][unit.col][1];
    int w2 = (1 << SGRPROJ_PRJ_BITS) - w0 - w1;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int u = UpscaledCdefFrame->getPixel(plane, x + j, y + i) << SGRPROJ_RST_BITS;
            int v = w1 * u;
            if (r0)
                v += w0 * flt0[i][j];
            else
                v += w0 * u;
            if (r1)
                v += w2 * flt1[i][j];
            else
                v += w2 * u;
            int s = ROUND2(v, SGRPROJ_RST_BITS + SGRPROJ_PRJ_BITS);
            LrFrame->setPixel(plane, x + j, y + i, CLIP1(s));
        }
    }
}

}
