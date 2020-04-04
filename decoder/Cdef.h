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

struct CdefParams;
struct ModeInfoBlock;

class Cdef {
public:
    Cdef(const ConstFramePtr& frame);
    std::shared_ptr<YuvFrame> filter(const std::shared_ptr<YuvFrame>& frame);

private:
    void cdefDirection(const std::shared_ptr<YuvFrame>& frame,
        int r, int, int& yDir, int& var);
    void cdefFilter(const std::shared_ptr<YuvFrame>& cdef,
        const std::shared_ptr<YuvFrame>& frame,
        int plane, int r, int c, int priStr, int secStr, int damping, int dir);
    uint8_t cdef_get_at(const std::shared_ptr<YuvFrame>& frame,
        int plane, int x0, int y0, int i, int j, int dir, int k,
        int sign, int subX, int subY, bool& CdefAvailable);
    bool is_inside_filter_region(int candidateR, int candidateC);
    void cdef_block(const std::shared_ptr<YuvFrame>& cdef,
        const std::shared_ptr<YuvFrame>& frame,
        int r, int c, int idx);
    const ModeInfoBlock& getModeInfo(int row, int col);
    ConstFramePtr m_frame;
    const SequenceHeader& m_sequence;
    const CdefParams& m_cdef;
};
}
