#include "LoopFilter.h"
#include "Parser.h"

namespace YamiAv1 {

using namespace YamiAv1;
LoopFilter::LoopFilter(const ConstFramePtr& frame)
    : m_frame(frame)
    , m_sequence(*frame->m_sequence)
    , m_filter(m_frame->m_loopFilter)
    , m_deltaLF(m_frame->m_deltaLf)
{
}

void LoopFilter::filter(const std::shared_ptr<YuvFrame>& frame) const
{
    const uint8_t* loop_filter_level = &(m_filter.loop_filter_level[0]);
    if (!loop_filter_level[0] && !loop_filter_level[1])
        return;
    for (int plane = 0; plane < m_sequence.NumPlanes; plane++) {
        if (plane == 0 || loop_filter_level[1 + plane]) {
            for (int pass = 0; pass < 2; pass++) {
                int rowStep = (plane == 0) ? 1 : (1 << m_sequence.subsampling_y);
                int colStep = (plane == 0) ? 1 : (1 << m_sequence.subsampling_x);
                for (int row = 0; row < m_frame->MiRows; row += rowStep) {
                    for (int col = 0; col < m_frame->MiCols; col += colStep) {
                        loop_filter_edge(frame, plane, pass, row, col);
                    }
                }
            }
        }
    }
}

static bool getIsBlockEdge(int xP, int yP, BLOCK_SIZE planeSize, int pass)
{
    if (!pass && !(xP % Block_Width[planeSize]))
        return true;
    if (pass && !(yP % Block_Height[planeSize]))
        return true;
    return false;
}

static bool getIsTxEdge(int xP, int yP, TX_SIZE txSize, int pass)
{
    if (!pass && !(xP % Tx_Width[txSize]))
        return true;
    if (pass && !(yP % Tx_Height[txSize]))
        return true;
    return false;
}

static bool getApplyFilter(bool isTxEdge, bool isBlockEdge, bool skip, bool isIntra)
{
    if (!isTxEdge)
        return false;
    return isBlockEdge || !skip || isIntra;
}

void LoopFilter::loop_filter_edge(const std::shared_ptr<YuvFrame>& frame,
    int plane, int pass, int row, int col) const
{
    int subX = (plane == 0) ? 0 : m_sequence.subsampling_x;
    int subY = (plane == 0) ? 0 : m_sequence.subsampling_y;
    int dx = (pass == 0) ? 1 : 0;
    int dy = (pass == 1) ? 1 : 0;
    int x = col * MI_SIZE;
    int y = row * MI_SIZE;
    row = (row | subY);
    col = (col | subX);
    bool onScreen = isOnScreen(x, y, pass);
    if (!onScreen)
        return;
    int xP = x >> subX;
    int yP = y >> subY;
    int prevRow = row - (dy << subY);
    int prevCol = col - (dx << subX);
    BLOCK_SIZE MiSize = m_frame->MiSizes[row][col];
    TX_SIZE txSz = m_frame->LoopfilterTxSizes[plane][row >> subY][col >> subX];
    BLOCK_SIZE planeSize = m_sequence.get_plane_residual_size(MiSize, plane);
    bool skip = m_frame->Skips[row][col];
    bool isIntra = m_frame->RefFrames[row][col][0] <= INTRA_FRAME;
    TX_SIZE prevTxSz = m_frame->LoopfilterTxSizes[plane][prevRow >> subY][prevCol >> subX];
    bool isBlockEdge = getIsBlockEdge(xP, yP, planeSize, pass);
    bool isTxEdge = getIsTxEdge(xP, yP, txSz, pass);
    bool applyFilter = getApplyFilter(isTxEdge, isBlockEdge, skip, isIntra);
    int filterSize = getFilterSize(txSz, prevTxSz, pass, plane);

    int lvl, limit, blimit, thresh;
    getFilterStrength(row, col, plane, pass, lvl, limit, blimit, thresh);
    if (!lvl)
        getFilterStrength(prevRow, prevCol, plane, pass, lvl, limit, blimit, thresh);

    for (int i = 0; i < MI_SIZE; i++) {
        if (applyFilter && lvl > 0) {
            sampleFilter(frame, xP + dy * i, yP + dx * i, plane, limit, blimit, thresh, dx, dy, filterSize);
        }
    }
}
void LoopFilter::sampleFilter(const std::shared_ptr<YuvFrame>& frame,
    int x, int y, int plane,
    int limit, int blimit, int thresh,
    int dx, int dy, int filterSize) const
{
    int hevMask, filterMask, flatMask, flatMask2;
    getFilterMask(frame, x, y, plane, limit, blimit, thresh, dx, dy, filterSize, hevMask, filterMask, flatMask, flatMask2);
    if (!filterMask)
        return;
    if (filterSize == 4 || !flatMask) {
        narrowFilter(frame, hevMask, x, y, plane, dx, dy);
    } else if (filterSize == 8 || !flatMask2) {
        wideFilter(frame, x, y, plane, dx, dy, 3);
    } else {
        wideFilter(frame, x, y, plane, dx, dy, 4);
    }
}
#define FILTER4_CLAMP(value) CLIP3(-(1 << (BitDepth - 1)), (1 << (BitDepth - 1)) - 1, (value))
void LoopFilter::narrowFilter(const std::shared_ptr<YuvFrame>& frame,
    int hevMask, int x, int y, int plane, int dx, int dy) const
{
    uint8_t BitDepth = m_sequence.BitDepth;
    uint8_t q0 = frame->getPixel(plane, x, y);
    uint8_t q1 = frame->getPixel(plane, x + dx, y + dy);
    uint8_t p0 = frame->getPixel(plane, x - dx, y - dy);
    uint8_t p1 = frame->getPixel(plane, x - dx * 2, y - dy * 2);

    int ps0 = p0 - (0x80 << (BitDepth - 8));
    int ps1 = p1 - (0x80 << (BitDepth - 8));
    int qs0 = q0 - (0x80 << (BitDepth - 8));
    int qs1 = q1 - (0x80 << (BitDepth - 8));
    int filter = hevMask ? FILTER4_CLAMP(ps1 - qs1) : 0;
    filter = FILTER4_CLAMP(filter + 3 * (qs0 - ps0));
    int filter1 = FILTER4_CLAMP(filter + 4) >> 3;
    int filter2 = FILTER4_CLAMP(filter + 3) >> 3;
    uint8_t oq0 = FILTER4_CLAMP(qs0 - filter1) + (0x80 << (BitDepth - 8));
    uint8_t op0 = FILTER4_CLAMP(ps0 + filter2) + (0x80 << (BitDepth - 8));
    frame->setPixel(plane, x, y, oq0);
    frame->setPixel(plane, x - dx, y - dy, op0);
    if (!hevMask) {
        filter = ROUND2(filter1, 1);
        uint8_t oq1 = FILTER4_CLAMP(qs1 - filter) + (0x80 << (BitDepth - 8));
        uint8_t op1 = FILTER4_CLAMP(ps1 + filter) + (0x80 << (BitDepth - 8));
        frame->setPixel(plane, x + dx, y + dy, oq1);
        frame->setPixel(plane, x - dx * 2, y - dy * 2, op1);
    }
}
void LoopFilter::wideFilter(const std::shared_ptr<YuvFrame>& frame,
    int x, int y, int plane, int dx, int dy, int log2Size) const
{
    int n;
    if (log2Size == 4) {
        n = 6;
    } else if (!plane) {
        n = 3;
    } else {
        n = 2;
    }
    int n2;
    if (log2Size == 3 && !plane) {
        n2 = 0;
    } else {
        n2 = 1;
    }
    std::vector<uint8_t> filtered(2 * n);
    uint8_t* F = &filtered[n];
    for (int i = -n; i < n; i++) {
        int t = 0;
        for (int j = -n; j <= n; j++) {
            int p = CLIP3(-(n + 1), n, i + j);
            int tap = (std::abs(j) <= n2) ? 2 : 1;
            t += frame->getPixel(plane, x + p * dx, y + p * dy) * tap;
        }
        F[i] = ROUND2(t, log2Size);
    }
    for (int i = -n; i < n; i++) {
        frame->setPixel(plane, x + i * dx, y + i * dy, F[i]);
    }
}
void LoopFilter::getFilterMask(const std::shared_ptr<YuvFrame>& frame,
    int x, int y, int plane, int limit, int blimit, int thresh,
    int dx, int dy, int filterSize, int& hevMask, int& filterMask, int& flatMask, int& flatMask2) const
{
    uint8_t q0, q1, q2, q3, q4, q5, q6, p0, p1, p2, p3, p4, p5, p6;

    q0 = frame->getPixel(plane, x, y);
    q1 = frame->getPixel(plane, x + dx, y + dy);
    q2 = frame->getPixel(plane, x + dx * 2, y + dy * 2);
    q3 = frame->getPixel(plane, x + dx * 3, y + dy * 3);

    p0 = frame->getPixel(plane, x - dx, y - dy);
    p1 = frame->getPixel(plane, x - dx * 2, y - dy * 2);
    p2 = frame->getPixel(plane, x - dx * 3, y - dy * 3);
    p3 = frame->getPixel(plane, x - dx * 4, y - dy * 4);
    if (filterSize == 16) {
        q4 = frame->getPixel(plane, x + dx * 4, y + dy * 4);
        q5 = frame->getPixel(plane, x + dx * 5, y + dy * 5);
        q6 = frame->getPixel(plane, x + dx * 6, y + dy * 6);
        p4 = frame->getPixel(plane, x - dx * 5, y - dy * 5);
        p5 = frame->getPixel(plane, x - dx * 6, y - dy * 6);
        p6 = frame->getPixel(plane, x - dx * 7, y - dy * 7);
    }

    uint8_t BitDepth = m_sequence.BitDepth;
    hevMask = 0;
    int threshBd = thresh << (BitDepth - 8);
    hevMask |= (std::abs(p1 - p0) > threshBd);
    hevMask |= (std::abs(q1 - q0) > threshBd);

    int filterLen;
    if (filterSize == 4) {
        filterLen = 4;
    } else if (plane) {
        filterLen = 6;
    } else if (filterSize == 8) {
        filterLen = 8;
    } else {
        filterLen = 16;
    }
    int limitBd = limit << (BitDepth - 8);
    int blimitBd = blimit << (BitDepth - 8);
    int mask = 0;
    mask |= (std::abs(p1 - p0) > limitBd);
    mask |= (std::abs(q1 - q0) > limitBd);
    mask |= (std::abs(p0 - q0) * 2 + std::abs(p1 - q1) / 2 > blimitBd);
    if (filterLen >= 6) {
        mask |= (std::abs(p2 - p1) > limitBd);
        mask |= (std::abs(q2 - q1) > limitBd);
    }
    if (filterLen >= 8) {
        mask |= (std::abs(p3 - p2) > limitBd);
        mask |= (std::abs(q3 - q2) > limitBd);
    }
    filterMask = (mask == 0);

    int thresholdBd = 1 << (BitDepth - 8);
    if (filterSize >= 8) {
        mask = 0;
        mask |= (std::abs(p1 - p0) > thresholdBd);
        mask |= (std::abs(q1 - q0) > thresholdBd);
        mask |= (std::abs(p2 - p0) > thresholdBd);
        mask |= (std::abs(q2 - q0) > thresholdBd);
        if (filterLen >= 8) {
            mask |= (std::abs(p3 - p0) > thresholdBd);
            mask |= (std::abs(q3 - q0) > thresholdBd);
        }
        flatMask = (mask == 0);
    } else {
        flatMask = 0;
    }
    if (filterSize >= 16) {
        mask = 0;
        mask |= (std::abs(p6 - p0) > thresholdBd);
        mask |= (std::abs(q6 - q0) > thresholdBd);
        mask |= (std::abs(p5 - p0) > thresholdBd);
        mask |= (std::abs(q5 - q0) > thresholdBd);
        mask |= (std::abs(p4 - p0) > thresholdBd);
        mask |= (std::abs(q4 - q0) > thresholdBd);
        flatMask2 = (mask == 0);
    } else {
        flatMask2 = 0;
    }
}

int LoopFilter::getFilterSize(TX_SIZE txSz, TX_SIZE prevTxSz, int pass, int plane) const
{
    int baseSize;
    if (!pass) {
        baseSize = std::min(Tx_Width[prevTxSz], Tx_Width[txSz]);
    } else {
        baseSize = std::min(Tx_Height[prevTxSz], Tx_Height[txSz]);
    }
    return !plane ? std::min(16, baseSize) : std::min(8, baseSize);
}
void LoopFilter::getFilterStrength(int row, int col, int plane, int pass, int& lvl, int& limit, int& blimit, int& thresh) const
{
    uint8_t segment = m_frame->SegmentIds[row][col];
    int ref = m_frame->RefFrames[row][col][0];
    PREDICTION_MODE mode = m_frame->YModes[row][col];
    int modeType = mode >= NEARESTMV && mode != GLOBALMV && mode != GLOBAL_GLOBALMV;
    int8_t deltaLF = getDeltaLF(row, col, plane, pass);
    lvl = getLvl(segment, ref, modeType, deltaLF, plane, pass);
    limit = getLimit(lvl);
    blimit = 2 * (lvl + 2) + limit;
    thresh = lvl >> 4;
}
int LoopFilter::getLimit(int lvl) const
{
    int shift;
    uint8_t loop_filter_sharpness = m_filter.loop_filter_sharpness;
    if (loop_filter_sharpness > 4) {
        shift = 2;
    } else if (loop_filter_sharpness > 0) {
        shift = 1;
    } else {
        shift = 0;
    }
    int limit = loop_filter_sharpness > 0 ? CLIP3(1, 9 - loop_filter_sharpness, lvl >> shift) : std::max(1, lvl >> shift);
    return limit;
}
int8_t LoopFilter::getLvl(uint8_t segment, int ref, int modeType, int8_t deltaLF, int plane, int pass) const
{
    int i = (plane == 0) ? pass : (plane + 1);
    int8_t baseFilterLevel = CLIP3(0, MAX_LOOP_FILTER, deltaLF + m_filter.loop_filter_level[i]);
    int8_t lvlSeg = baseFilterLevel;
    SEG_LVL_FEATURE feature = (SEG_LVL_FEATURE)(SEG_LVL_ALT_LF_Y_V + i);

    const Segmentation& seg = m_frame->m_segmentation;
    if (seg.seg_feature_active_idx(segment, feature)) {
        lvlSeg = seg.FeatureData[segment][feature] + lvlSeg;
        lvlSeg = CLIP3(0, MAX_LOOP_FILTER, lvlSeg);
    }
    const int8_t* loop_filter_ref_deltas = m_filter.loop_filter_ref_deltas;
    const int8_t* loop_filter_mode_deltas = m_filter.loop_filter_mode_deltas;
    if (m_filter.loop_filter_delta_enabled) {
        int8_t nShift = lvlSeg >> 5;
        if (ref == INTRA_FRAME) {
            lvlSeg = lvlSeg + (loop_filter_ref_deltas[INTRA_FRAME] << nShift);
        } else {
            lvlSeg = lvlSeg + (loop_filter_ref_deltas[ref] << nShift) + (loop_filter_mode_deltas[modeType] << nShift);
        }
        lvlSeg = CLIP3(0, MAX_LOOP_FILTER, lvlSeg);
    }
    return lvlSeg;
}
int8_t LoopFilter::getDeltaLF(int row, int col, int plane, int pass) const
{
    if (!m_deltaLF.delta_lf_multi)
        return m_frame->DeltaLFs[0][row][col];
    return m_frame->DeltaLFs[plane == 0 ? pass : (plane + 1)][row][col];
}

bool LoopFilter::isOnScreen(int x, int y, int pass) const
{
    if (x >= m_frame->FrameWidth || y >= m_frame->FrameHeight)
        return false;
    if (!pass && !x)
        return false;
    if (pass && !y)
        return false;
    return true;
}

}
