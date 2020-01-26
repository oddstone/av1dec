#pragma once

#include "Av1Common.h"
#include <deque>
#include <memory>
#include <stdint.h>
#include <vector>

namespace Yami {
struct YuvFrame;
}

namespace YamiAv1 {

class Parser;
struct FrameHeader;
class Tile;
typedef std::shared_ptr<FrameHeader> FramePtr;
typedef std::vector<std::shared_ptr<Tile>> Tiles;

class Decoder {
public:
    bool decode(uint8_t* data, size_t size);
    std::shared_ptr<Yami::YuvFrame> getOutput();
    Decoder();
    ~Decoder();

private:
    bool decodeFrame(Tiles& tiles);
    void showExistingFrame();
    std::shared_ptr<Yami::YuvFrame> decode_frame_wrapup(const std::shared_ptr<Yami::YuvFrame>&);
    void updateFrameStore(const FrameHeader& h, const std::shared_ptr<Yami::YuvFrame> frame);
    void frame_end_update_cdf(Tiles& tiles);
    std::shared_ptr<Yami::YuvFrame> upscaling(const std::shared_ptr<Yami::YuvFrame>&);
    std::unique_ptr<Parser> m_parser;
    std::deque<std::shared_ptr<Yami::YuvFrame>> m_output;
    FramePtr m_frame;
    Tiles m_tiles;
    FrameStore m_store;
};
}
