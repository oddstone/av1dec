#pragma once
#include <memory>
#include "Av1Common.h"

namespace YamiAv1 {

struct BlockTree {
public:
    virtual void parse() = 0;
    virtual bool decode(std::shared_ptr<Yami::YuvFrame>&, const FrameStore&) = 0;
};

}
