#pragma once

#include <memory>
#include <deque>
#include <vector>
#include <stdint.h>

struct YuvFrame;
class Tile;

namespace YamiParser {
namespace Av1 {
    class Parser;
    typedef std::vector<std::shared_ptr<Tile>> Tiles;
    class Decoder {
    public:
        bool decode(uint8_t* data, size_t size);
        std::shared_ptr<YuvFrame> getOutput();
        Decoder();
        ~Decoder();
    private:
        bool decodeFrame(Tiles tiles);
        std::unique_ptr<Parser> m_parser;
        std::deque<std::shared_ptr<YuvFrame>>    m_output;
        Tiles               m_tiles;
    };
};
};
