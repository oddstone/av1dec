#include "LoopRestoration.h"
#include "Av1Parser.h"
#include "log.h"

namespace Yami {
    namespace Av1 {
        const static int FILTER_BITS = 7;

        static void RoundingVariablesDerivation(
            bool isCompound, int BitDepth,
            int& InterRound0, int& InterRound1, int& InterPostRound)
        {
            InterRound0 = 3;
            InterRound1 = isCompound ? 7 : 11;
            if (BitDepth == 12) {
                InterRound0 += 2;
                if (!isCompound) {
                    InterRound1 -= 2;
                }
            }
            InterPostRound = 2 * FILTER_BITS - (InterRound0 + InterRound1);
        }

        static int count_units_in_frame(int unitSize, int frameSize) {
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

        std::shared_ptr<YuvFrame> LoopRestoration::filter()
        {
            if (!m_frame->m_loopRestoration.UsesLr) {
                return UpscaledCdefFrame;
            }
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
            RoundingVariablesDerivation(0, m_sequence.BitDepth,
                InterRound0, InterRound1, InterPostRound);

            std::shared_ptr<YuvFrame> LrFrame = YuvFrame::create(UpscaledCdefFrame);
            for (int y = 0; y < m_frame->FrameHeight; y += MI_SIZE) {
                for (int x = 0; x < m_frame->UpscaledWidth; x += MI_SIZE) {
                    for (int plane = 0; plane < m_sequence.NumPlanes; plane++) {
                        if (m_loopRestoration.FrameRestorationType[plane] != RESTORE_NONE) {
                            int row = y >> MI_SIZE_LOG2;
                            int col = x >> MI_SIZE_LOG2;
                            loop_restore_block(LrFrame, plane, row, col);
                        }
                    }
                }
            }
            return LrFrame;
        }

        std::vector<int8_t> getFilter(const std::vector<int8_t>& coeff)
        {
            std::vector<int8_t> filter(FILTER_BITS);
            filter[3] = 128;
            for (int i = 0; i < 3; i++) {
                uint8_t c = coeff[i];
                filter[i] = c;
                filter[6 - i] = c;
                filter[3] -= 2 * c;
            }
            return std::move(filter);
        }

        uint8_t LoopRestoration::get_source_sample(int plane, int x, int y,
            int StripeStartY, int StripeEndY)
        {
            x = std::min(PlaneEndX[plane], x);
            x = std::max(0, x);
            y = std::min(PlaneEndY[plane], y);
            y = std::max(0, y);
            if (y < StripeStartY) {
                y = std::max(StripeStartY - 2, y);
                return UpscaledCurrFrame->getPixel(plane, x, y);
            } else if (y > StripeEndY) {
                y = std::min(StripeEndY + 2, y);
                return UpscaledCurrFrame->getPixel(plane, x, y);
            } else {
                return UpscaledCdefFrame->getPixel(plane, x, y);
            }
        }
        void LoopRestoration::wienerFilter(const std::shared_ptr<YuvFrame>& LrFrame,
            int plane, int unitRow, int unitCol, int x, int y, int w, int h, int StripeStartY, int StripeEndY)
        {
            int BitDepth = m_sequence.BitDepth;
            std::vector<int8_t> vfilter = getFilter(m_loopRestoration.LrWiener[plane][unitRow][unitCol][0]);
            std::vector<int8_t> hfilter = getFilter(m_loopRestoration.LrWiener[plane][unitRow][unitCol][1]);
            std::vector<std::vector<int>> intermediate;
            intermediate.assign(h + 6, std::vector<int>(w));
            int offset = (1 << (BitDepth + FILTER_BITS - InterRound0 - 1));
            int limit = (1 << (BitDepth + 1 + FILTER_BITS - InterRound0)) - 1;
            for (int r = 0; r < h + 6; r++) {
                for (int c = 0; c < w; c++) {
                    int s = 0;
                    for (int t = 0; t < 7; t++) {
                        s += hfilter[t] * get_source_sample(plane, x + c + t - 3, y + r - 3, StripeStartY, StripeEndY);
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

        std::vector<std::vector<int>> LoopRestoration::boxFilter(int plane, int x, int y, int w, int h, uint8_t set, int StripeStartY, int StripeEndY, int pass)
        {
            uint8_t BitDepth = m_sequence.BitDepth;

            std::vector<std::vector<int>> F(h, std::vector<int>(w));
            int r = Sgr_Params[set][pass * 2 + 0];
            if (r == 0)
                return std::move(F);
            int eps = Sgr_Params[set][pass * 2 + 1];
            std::vector<std::vector<int>> A(h + 2, std::vector<int>(w + 2));
            std::vector<std::vector<int>> B(h + 2, std::vector<int>(w + 2));

            int n = (2 * r + 1) * (2 * r + 1);
            int n2e = n * n * eps;
            int s = (((1 << SGRPROJ_MTABLE_BITS) + n2e / 2) / n2e);
            for (int i = -1; i < h + 1; i++) {
                for (int j = -1; j < w + 1; j++) {
                    int a = 0;
                    int b = 0;
                    for (int dy = -r; dy <= r; dy++) {
                        for (int dx = -r; dx <= r; dx++) {
                            int c = get_source_sample(plane, x + j + dx, y + i + dy, StripeStartY, StripeEndY);
                            a += c * c;
                            b += c;
                        }
                    }
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

        void LoopRestoration::selfGuidedFilter(const std::shared_ptr<YuvFrame>& LrFrame,
            int plane, int unitRow, int unitCol, int x, int y, int w, int h, int StripeStartY, int StripeEndY)
        {
            uint8_t set = m_loopRestoration.LrSgrSet[plane][unitRow][unitCol];
            std::vector<std::vector<int>> flt0 = boxFilter(plane, x, y, w, h, set, StripeStartY, StripeEndY, 0);
            std::vector<std::vector<int>> flt1 = boxFilter(plane, x, y, w, h, set, StripeStartY, StripeEndY, 1);
            int w0 = m_loopRestoration.LrSgrXqd[plane][unitRow][unitCol][0];
            int w1 = m_loopRestoration.LrSgrXqd[plane][unitRow][unitCol][1];
            int w2 = (1 << SGRPROJ_PRJ_BITS) - w0 - w1;
            int r0 = Sgr_Params[set][0];
            int r1 = Sgr_Params[set][2];
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

        void LoopRestoration::loop_restore_block(const    std::shared_ptr<YuvFrame>& LrFrame,
            int plane, int row, int col)
        {
            int lumaY = row * MI_SIZE;
            int stripeNum = (lumaY + 8) / 64;
            int subX = 0;
            int subY = 0;
            if (plane) {
                subX = m_sequence.subsampling_x;
                subY = m_sequence.subsampling_y;
            }

            int StripeStartY = ((-8 + stripeNum * 64) >> subY);
            int StripeEndY = StripeStartY + (64 >> subY) - 1;
            int unitRow = std::min(unitRows[plane] - 1, ((row * MI_SIZE + 8) >> subY) / unitSize[plane]);
            int unitCol = std::min(unitCols[plane] - 1, (col * MI_SIZE >> subX) / unitSize[plane]);

            int x = (col * MI_SIZE >> subX);
            int y = (row * MI_SIZE >> subY);
            int w = std::min(MI_SIZE >> subX, PlaneEndX[plane] - x + 1);
            int h = std::min(MI_SIZE >> subY, PlaneEndY[plane] - y + 1);
            RestorationType rType = m_frame->m_loopRestoration.LrType[plane][unitRow][unitCol];
            if (rType == RESTORE_WIENER) {
                wienerFilter(LrFrame, plane, unitRow, unitCol, x, y, w, h, StripeStartY, StripeEndY);
            } else if (rType == RESTORE_SGRPROJ) {
                selfGuidedFilter(LrFrame, plane, unitRow, unitCol, x, y, w, h, StripeStartY, StripeEndY);
            }
        }
    }
}

