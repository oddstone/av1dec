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

#include "Parser.h"
#include "VideoFrame.h"
#include "enums.h"
#include <memory>

namespace YamiAv1 {

class LoopRestoration {
public:
    LoopRestoration(const ConstFramePtr&,
        const std::shared_ptr<YuvFrame>& upscaledCdefFrame,
        const std::shared_ptr<YuvFrame>& upscaledCurrFrame);
    std::shared_ptr<YuvFrame> filter();
    const static int REST_BORDER = 3;
    const static int MAX_REST_HEIGHT = (64 + REST_BORDER * 2);
    const static int MAX_REST_WIDTH = (256 + REST_BORDER * 2);

private:
    struct PlaneInfo;
    struct UnitInfo;
    struct StripeInfo;
    uint8_t get_source_sample(int plane, int x, int y, const StripeInfo& stripe);
    void wienerFilter(const std::shared_ptr<YuvFrame>& LrFrame,
        int plane, int unitRow, int unitCol,
        int x, int y, int w, int h, const StripeInfo& stripe);
    std::vector<std::vector<int>> boxFilter(int plane, int x, int y,
        int w, int h, uint8_t set, const StripeInfo& stripe, int pass);

    void selfGuidedFilter(const std::shared_ptr<YuvFrame>& LrFrame,
        int plane, int unitRow, int unitCol,
        int x, int y, int w, int h, const StripeInfo& stripe);
    void forEachPlane(std::shared_ptr<YuvFrame>& LrFrame);
    void forEachUnit(std::shared_ptr<YuvFrame>& LrFrame, const PlaneInfo& info);
    void forEachStripe(std::shared_ptr<YuvFrame>& LrFrame, const PlaneInfo& plane, const UnitInfo& unit);
    void forEachBlock(std::shared_ptr<YuvFrame>& LrFrame, const PlaneInfo& plane, const UnitInfo& unit, const StripeInfo& stripe);
    ConstFramePtr m_frame;
    const SequenceHeader& m_sequence;
    const LoopRestorationpParams& m_loopRestoration;
    static const int MAX_PLANES = 3;
    int unitSize[MAX_PLANES];
    int unitRows[MAX_PLANES];
    int unitCols[MAX_PLANES];
    int PlaneEndX[MAX_PLANES];
    int PlaneEndY[MAX_PLANES];
    const std::shared_ptr<YuvFrame>& UpscaledCdefFrame;
    const std::shared_ptr<YuvFrame>& UpscaledCurrFrame;
    int InterRound0;
    int InterRound1;
    int InterPostRound;

    int sumA[MAX_REST_HEIGHT][MAX_REST_WIDTH];
    int sumB[MAX_REST_HEIGHT][MAX_REST_WIDTH];
};
}
