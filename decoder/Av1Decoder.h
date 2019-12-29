#ifndef Av1Decoder_h
#define Av1Decoder_h

#include "Av1Common.h"
#include <memory>
#include <deque>
#include <vector>
#include <stdint.h>

namespace Yami {
    struct YuvFrame;
    namespace Av1 {
        class Parser;
        struct FrameHeader;
        class Tile;
        typedef std::shared_ptr<FrameHeader> FramePtr;
        typedef std::vector<std::shared_ptr<Tile>> Tiles;
        class Decoder {
        public:
            bool decode(uint8_t* data, size_t size);
            std::shared_ptr<YuvFrame> getOutput();
            Decoder();
            ~Decoder();
        private:
            bool decodeFrame(Tiles tiles);
            std::shared_ptr<YuvFrame> decode_frame_wrapup(const std::shared_ptr<YuvFrame>&);
            void updateFrameStore(const FrameHeader& h, const std::shared_ptr<YuvFrame> frame);
            std::shared_ptr<YuvFrame> upscaling(const std::shared_ptr<YuvFrame>&);
            std::unique_ptr<Parser> m_parser;
            std::deque<std::shared_ptr<YuvFrame>> m_output;
            FramePtr m_frame;
            Tiles m_tiles;
            FrameStore m_store;
        };
    }
}
#endif