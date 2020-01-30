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

#include "VideoFrame.h"
#include <string.h>

namespace Yami {
struct YuvFrameImp : public YuvFrame {
    friend struct YuvFrame;

private:
    std::vector<uint8_t> m_data;
};

#define ROOF(b, a) ((b + (a - 1)) & ~(a - 1))
std::shared_ptr<YuvFrame> YuvFrame::create(int width, int height)
{
    std::shared_ptr<YuvFrameImp> p(new YuvFrameImp);
    int alignedW = ROOF(width, 128);
    int alignedH = ROOF(height, 128);
    p->m_data.resize(alignedW * alignedH * 3 / 2);
    p->width = width;
    p->height = height;
    p->widths[0] = width;
    p->widths[1] = width / 2;
    p->widths[2] = width / 2;
    p->heights[0] = height;
    p->heights[1] = height / 2;
    p->heights[2] = height / 2;
    p->data[0] = &p->m_data[0];
    p->data[1] = &p->m_data[alignedW * alignedH];
    p->data[2] = &p->m_data[alignedW * alignedH * 5 / 4];
    p->strides[0] = alignedW;
    p->strides[1] = alignedW / 2;
    p->strides[2] = alignedW / 2;
    return std::dynamic_pointer_cast<YuvFrame>(p);
}

std::shared_ptr<YuvFrame> YuvFrame::create(const std::shared_ptr<YuvFrame>& other)
{
    std::shared_ptr<YuvFrame> frame = YuvFrame::create(other->width, other->height);
    if (!frame)
        return frame;
    for (int p = 0; p < MAX_PLANES; p++) {
        const uint8_t* src = other->data[p];
        uint8_t* dest = frame->data[p];
        for (int h = 0; h < other->heights[p]; h++) {
            memcpy(dest, src, other->widths[p]);
            src += other->strides[p];
            dest += other->strides[p];
        }
    }
    return frame;
}
}