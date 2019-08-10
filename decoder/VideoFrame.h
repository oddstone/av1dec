#pragma once

#include <memory>
#include <vector>


struct YuvFrame {
    static const int MAX_PLANES = 3;
    int64_t     pts;
    int         width;
    int         height;
    uint8_t*    data[MAX_PLANES];
    int         strides[MAX_PLANES];
    static std::shared_ptr<YuvFrame> create(int width, int height);
};
