#pragma once

#include <memory>
#include <stdint.h>

namespace YamiParser {
namespace Av1 {
    class Parser;
    class Decoder {
    public:
        bool decode(uint8_t* data, size_t size);
        Decoder();
        ~Decoder();
        std::unique_ptr<Parser> m_parser;
    };
};
};
