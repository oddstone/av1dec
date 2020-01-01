#include "Cdef.h"
#include "Av1Parser.h"
#include <limits>

namespace YamiAv1 {

using namespace Yami;
Cdef::Cdef(const ConstFramePtr& frame)
    : m_frame(frame)
    , m_sequence(*m_frame->m_sequence)
    , m_cdef(m_frame->m_cdef)
{
}

std::shared_ptr<YuvFrame> Cdef::filter(const std::shared_ptr<YuvFrame>& frame)
{
    std::shared_ptr<YuvFrame> cdef = YuvFrame::create(frame);
    if (!cdef)
        return cdef;
    int step4 = Num_4x4_Blocks_Wide[BLOCK_8X8];
    int cdefSize4 = Num_4x4_Blocks_Wide[BLOCK_64X64];
    int cdefMask4 = ~(cdefSize4 - 1);
    for (int r = 0; r < m_frame->MiRows; r += step4) {
        for (int c = 0; c < m_frame->MiCols; c += step4) {
            int baseR = r & cdefMask4;
            int baseC = c & cdefMask4;
            int idx = m_cdef.cdef_idx[baseR][baseC];
            cdef_block(cdef, frame, r, c, idx);
        }
    }
    return cdef;
}

const int Cdef_Uv_Dir[2][2][8] = {
    { { 0, 1, 2, 3, 4, 5, 6, 7 },
        { 1, 2, 2, 2, 3, 4, 6, 0 } },
    { { 7, 0, 2, 4, 5, 6, 6, 6 },
        { 0, 1, 2, 3, 4, 5, 6, 7 } }
};

void Cdef::cdef_block(const std::shared_ptr<YuvFrame>& cdef,
    const std::shared_ptr<YuvFrame>& frame,
    int r, int c, int idx)
{
    if (idx == -1)
        return;
    int coeffShift = m_sequence.BitDepth - 8;
    bool skip = (m_frame->Skips[r][c] && m_frame->Skips[r + 1][c]
        && m_frame->Skips[r][c + 1] && m_frame->Skips[r + 1][c + 1]);
    if (skip)
        return;

    int yDir, var;
    cdefDirection(frame, r, c, yDir, var);
    int priStr = m_cdef.cdef_y_pri_strength[idx] << coeffShift;
    int secStr = m_cdef.cdef_y_sec_strength[idx] << coeffShift;
    int dir = (priStr == 0) ? 0 : yDir;
    int varStr = (var >> 6) ? std::min(FloorLog2(var >> 6), 12) : 0;
    priStr = (var ? (priStr * (4 + varStr) + 8) >> 4 : 0);
    int damping = m_cdef.CdefDamping + coeffShift;
    cdefFilter(cdef, frame, 0, r, c, priStr, secStr, damping, dir);
    if (m_sequence.NumPlanes) {
        priStr = m_cdef.cdef_uv_pri_strength[idx] << coeffShift;
        secStr = m_cdef.cdef_uv_sec_strength[idx] << coeffShift;
        dir = (priStr == 0) ? 0 : Cdef_Uv_Dir[m_sequence.subsampling_x][m_sequence.subsampling_y][yDir];
        damping = m_cdef.CdefDamping + coeffShift - 1;
        cdefFilter(cdef, frame, 1, r, c, priStr, secStr, damping, dir);
        cdefFilter(cdef, frame, 2, r, c, priStr, secStr, damping, dir);
    }
}

const static int Cdef_Pri_Taps[2][2] = {
    { 4, 2 }, { 3, 3 }
};

const static int Cdef_Sec_Taps[2][2] = {
    { 2, 1 }, { 2, 1 }
};

static int constrain(int diff, int threshold, int damping)
{
    if (!threshold)
        return 0;
    int dampingAdj = std::max(0, damping - FloorLog2(threshold));
    int sign = (diff < 0) ? -1 : 1;
    return sign * CLIP3(0, std::abs(diff), threshold - (std::abs(diff) >> dampingAdj));
}

const static int Cdef_Directions[8][2][2] = {
    { { -1, 1 }, { -2, 2 } },
    { { 0, 1 }, { -1, 2 } },
    { { 0, 1 }, { 0, 2 } },
    { { 0, 1 }, { 1, 2 } },
    { { 1, 1 }, { 2, 2 } },
    { { 1, 0 }, { 2, 1 } },
    { { 1, 0 }, { 2, 0 } },
    { { 1, 0 }, { 2, -1 } }
};

bool Cdef::is_inside_filter_region(int candidateR, int candidateC)
{
    int colStart = 0;
    int colEnd = m_frame->MiCols;
    int rowStart = 0;
    int rowEnd = m_frame->MiRows;
    return (candidateC >= colStart && candidateC < colEnd && candidateR >= rowStart && candidateR < rowEnd);
}

uint8_t Cdef::cdef_get_at(const std::shared_ptr<YuvFrame>& frame,
    int plane, int x0, int y0, int i, int j, int dir, int k,
    int sign, int subX, int subY, bool& CdefAvailable)
{

    int y = y0 + i + sign * Cdef_Directions[dir][k][0];
    int x = x0 + j + sign * Cdef_Directions[dir][k][1];
    int candidateR = (y << subY) >> MI_SIZE_LOG2;
    int candidateC = (x << subX) >> MI_SIZE_LOG2;
    if (is_inside_filter_region(candidateR, candidateC)) {
        CdefAvailable = true;
        return frame->getPixel(plane, x, y);
    } else {
        CdefAvailable = false;
        return 0;
    }
}

void Cdef::cdefFilter(const std::shared_ptr<YuvFrame>& cdef,
    const std::shared_ptr<YuvFrame>& frame,
    int plane, int r, int c, int priStr, int secStr, int damping, int dir)
{
    int coeffShift = m_sequence.BitDepth - 8;
    int subX = (plane > 0) ? m_sequence.subsampling_x : 0;
    int subY = (plane > 0) ? m_sequence.subsampling_y : 0;
    int x0 = (c * MI_SIZE) >> subX;
    int y0 = (r * MI_SIZE) >> subY;
    int w = 8 >> subX;
    int h = 8 >> subY;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int sum = 0;
            uint8_t x = frame->getPixel(plane, x0 + j, y0 + i);
            uint8_t max = x;
            uint8_t min = x;
            for (int k = 0; k < 2; k++) {
                for (int sign = -1; sign <= 1; sign += 2) {
                    bool CdefAvailable;
                    uint8_t p = cdef_get_at(frame, plane, x0, y0, i, j, dir, k, sign, subX, subY, CdefAvailable);
                    if (CdefAvailable) {
                        sum += Cdef_Pri_Taps[(priStr >> coeffShift) & 1][k] * constrain(p - x, priStr, damping);
                        max = std::max(p, max);
                        min = std::min(p, min);
                    }
                    for (int dirOff = -2; dirOff <= 2; dirOff += 4) {
                        uint8_t s = cdef_get_at(frame, plane, x0, y0, i, j, (dir + dirOff) & 7, k, sign, subX, subY, CdefAvailable);
                        if (CdefAvailable) {
                            sum += Cdef_Sec_Taps[(priStr >> coeffShift) & 1][k] * constrain(s - x, secStr, damping);
                            max = std::max(s, max);
                            min = std::min(s, min);
                        }
                    }
                }
            }
            uint8_t pixel = CLIP3(min, max, x + ((8 + sum - (sum < 0)) >> 4));
            cdef->setPixel(plane, x0 + j, y0 + i, pixel);
        }
    }
}

static const int Div_Table[9] = {
    0, 840, 420, 280, 210, 168, 140, 120, 105
};
void Cdef::cdefDirection(const std::shared_ptr<YuvFrame>& frame,
    int r, int c, int& yDir, int& var)
{
    int cost[8], partial[8][15];
    for (int i = 0; i < 8; i++) {
        cost[i] = 0;
        for (int j = 0; j < 15; j++) {
            partial[i][j] = 0;
        }
    }
    int bestCost = 0;
    yDir = 0;
    int x0 = c << MI_SIZE_LOG2;
    int y0 = r << MI_SIZE_LOG2;
    uint8_t BitDepth = m_sequence.BitDepth;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            int x = (frame->getPixel(0, x0 + j, y0 + i) >> (BitDepth - 8)) - 128;
            partial[0][i + j] += x;
            partial[1][i + j / 2] += x;
            partial[2][i] += x;
            partial[3][3 + i - j / 2] += x;
            partial[4][7 + i - j] += x;
            partial[5][3 - i / 2 + j] += x;
            partial[6][j] += x;
            partial[7][i / 2 + j] += x;
        }
    }
    for (int i = 0; i < 8; i++) {
        cost[2] += partial[2][i] * partial[2][i];
        cost[6] += partial[6][i] * partial[6][i];
    }
    cost[2] *= Div_Table[8];
    cost[6] *= Div_Table[8];
    for (int i = 0; i < 7; i++) {
        cost[0] += (partial[0][i] * partial[0][i] + partial[0][14 - i] * partial[0][14 - i]) * Div_Table[i + 1];
        cost[4] += (partial[4][i] * partial[4][i] + partial[4][14 - i] * partial[4][14 - i]) * Div_Table[i + 1];
    }
    cost[0] += partial[0][7] * partial[0][7] * Div_Table[8];
    cost[4] += partial[4][7] * partial[4][7] * Div_Table[8];
    for (int i = 1; i < 8; i += 2) {
        for (int j = 0; j < 4 + 1; j++) {
            cost[i] += partial[i][3 + j] * partial[i][3 + j];
        }
        cost[i] *= Div_Table[8];
        for (int j = 0; j < 4 - 1; j++) {
            cost[i] += (partial[i][j] * partial[i][j]
                           + partial[i][10 - j] * partial[i][10 - j])
                * Div_Table[2 * j + 2];
        }
    }
    for (int i = 0; i < 8; i++) {
        if (cost[i] > bestCost) {
            bestCost = cost[i];
            yDir = i;
        }
    }
    var = (bestCost - cost[(yDir + 4) & 7]) >> 10;
}
}
