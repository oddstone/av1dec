#include "VideoFrame.h"
#include <string.h>

namespace Yami {
    struct YuvFrameImp : public YuvFrame
    {
        friend struct YuvFrame;
    private:
        std::vector<uint8_t> m_data;
    };

#define ROOF(b, a) ((b + (a -1)) & ~(a-1))
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