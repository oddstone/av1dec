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

#include <memory>
#include <vector>

namespace Yami {

struct YuvFrame {
    static const int MAX_PLANES = 3;
    int64_t pts;
    int width;
    int height;
    uint8_t* data[MAX_PLANES];
    int strides[MAX_PLANES];
    int widths[MAX_PLANES];
    int heights[MAX_PLANES];
    static std::shared_ptr<YuvFrame> create(int width, int height);
    static std::shared_ptr<YuvFrame> create(const std::shared_ptr<YuvFrame>&);
    inline uint8_t getPixel(int plane, int x, int y);
    inline void setPixel(int plane, int x, int y, uint8_t pixel);
    void extendBorder(int borders);
};

inline uint8_t YuvFrame::getPixel(int plane, int x, int y)
{
    return data[plane][y * strides[plane] + x];
}

inline void YuvFrame::setPixel(int plane, int x, int y, uint8_t pixel)
{
    data[plane][y * strides[plane] + x] = pixel;
}
}