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

#pragma once

#include "Av1Common.h"
#include "VideoFrame.h"
#include <memory>

namespace YamiAv1 {
class Parser;
struct LoopFilterParams;
struct DeltaLf;

class LoopFilter {
public:
    LoopFilter(const ConstFramePtr&);
    void filter(const std::shared_ptr<YuvFrame>& frame) const;

private:
    void loop_filter_edge(const std::shared_ptr<YuvFrame>& frame,
        int plane, int pass, int row, int col) const;
    void sampleFilter(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane,
        int limit, int blimit, int thresh,
        int dx, int dy, int filterSize) const;
    void narrowFilter(const std::shared_ptr<YuvFrame>& frame,
        int hevMask, int x, int y, int plane, int dx, int dy) const;
    void wideFilter(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane, int dx, int dy, int log2Size) const;
    void getFilterMask(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane, int limit, int blimit, int thresh,
        int dx, int dy, int filterSize, int& hevMask, int& filterMask, int& flatMask, int& flatMask2) const;

    int getFilterSize(TX_SIZE txSz, TX_SIZE prevTxSz, int pass, int plane) const;
    void getFilterStrength(int row, int col, int plane, int pass, int& lvl, int& limit, int& blimit, int& thresh) const;
    int getLimit(int lvl) const;
    int8_t getLvl(uint8_t segment, int ref, int modeType, int8_t deltaLF, int plane, int pass) const;
    int8_t getDeltaLF(int row, int col, int plane, int pass) const;

    bool isOnScreen(int x, int y, int pass) const;

    ConstFramePtr m_frame;
    const SequenceHeader& m_sequence;
    const LoopFilterParams& m_filter;
    const DeltaLf& m_deltaLF;
};
}
