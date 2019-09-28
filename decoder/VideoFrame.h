#pragma once

#include <memory>
#include <vector>

namespace Yami {
    struct YuvFrame {
        static const int MAX_PLANES = 3;
        int64_t     pts;
        int         width;
        int         height;
        uint8_t* data[MAX_PLANES];
        int         strides[MAX_PLANES];
        int         widths[MAX_PLANES];
        int         heights[MAX_PLANES];
        static std::shared_ptr<YuvFrame> create(int width, int height);
        static std::shared_ptr<YuvFrame> create(const std::shared_ptr<YuvFrame>&);
        inline uint8_t getPixel(int plane, int x, int y);
        inline void setPixel(int plane, int x, int y, uint8_t pixel);
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