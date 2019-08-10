#include "VideoFrame.h"

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
    int alignedW = ROOF(width, 8);
    int alignedH = ROOF(height, 8);
    p->m_data.resize(alignedW * alignedH  * 3 / 2);
    p->width = width;
    p->height = height;
    p->data[0] = &p->m_data[0];
    p->data[1] = &p->m_data[alignedW * alignedH];
    p->data[2] = &p->m_data[alignedW * alignedH * 5 /4];
    p->strides[0] = alignedW;
    p->strides[1] = alignedW / 2;
    p->strides[2] = alignedW / 2;
    return std::dynamic_pointer_cast<YuvFrame>(p);
}

