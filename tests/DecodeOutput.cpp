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

#include "DecodeOutput.h"
#include "log.h"
#include <functional>
#include <string>
#include <iostream>

std::shared_ptr<DecodeOutput> DecodeOutput::create(const char* path)
{
    std::shared_ptr<DecodeOutput> out;
    FILE* fp = fopen(path, "wb");
    if (!fp)
        return out;
    out.reset(new DecodeOutput(fp));
    return out;
}

DecodeOutput::DecodeOutput(FILE* fp)
    : m_fp(fp)
{
}

bool DecodeOutput::output(const std::shared_ptr<Yami::YuvFrame>& frame)
{
    for (int p = 0; p < Yami::YuvFrame::MAX_PLANES; p++) {
        uint8_t* start = frame->data[p];
        int stride = frame->strides[p];
        int width = frame->width;
        int height = frame->height;
        if (p) {
            width >>= 1;
            height >>= 1;
        }
        for (int h = 0; h < height; h++) {
            uint8_t* line = start + h * stride;
            if (fwrite(line, 1, width, m_fp) != width) {
                ERROR("write file failed");
                return false;
            }
        }
    }
    fflush(m_fp);
    return true;
}

DecodeOutput::~DecodeOutput()
{
    fclose(m_fp);
}

std::shared_ptr<DecodeOutputMd5> DecodeOutputMd5::create()
{
    std::shared_ptr<DecodeOutputMd5> out;
    out.reset(new DecodeOutputMd5());
    return out;
}

DecodeOutputMd5::DecodeOutputMd5()
{
    MD5_Init(&m_ctx);
}

bool DecodeOutputMd5::output(const std::shared_ptr<Yami::YuvFrame>& frame)
{
    for (int p = 0; p < Yami::YuvFrame::MAX_PLANES; p++) {
        uint8_t* start = frame->data[p];
        int stride = frame->strides[p];
        int width = frame->width;
        int height = frame->height;
        if (p) {
            width >>= 1;
            height >>= 1;
        }
        for (int h = 0; h < height; h++) {
            uint8_t* line = start + h * stride;
            MD5_Update(&m_ctx, line, width);
        }
    }
    return true;
}

std::string toString(const unsigned char digest[16])
{
    static const char hex[] = "0123456789abcdef";
    std::string str;
    for (int i = 0; i < 16; i++) {
        unsigned char d = digest[i];
        str.push_back(hex[d >> 4]);
        str.push_back(hex[d & 0xf]);
    }
    return std::move(str);
}

DecodeOutputMd5::~DecodeOutputMd5()
{
    unsigned char digest[16];
    MD5_Final(digest, &m_ctx);
    std::string str = toString(digest);
    std::cout << "md5=" << str;
}
