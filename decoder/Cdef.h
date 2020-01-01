#pragma once

#include <memory>
#include "Av1Common.h"
#include "VideoFrame.h"

namespace YamiAv1 {

struct CdefParams;

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
    ConstFramePtr m_frame;
    const SequenceHeader& m_sequence;
    const CdefParams& m_cdef;
};

}

