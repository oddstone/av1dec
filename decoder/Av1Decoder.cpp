#include "Av1Decoder.h"
#include "Av1Parser.h"
#include "bitReader.h"
#include "VideoFrame.h"
#include "log.h"

namespace YamiParser {
namespace Av1 {
    Decoder::Decoder()
    {
        m_parser.reset(new Parser);
    }

    Decoder::~Decoder()
    {
    }

    bool Decoder::decode(uint8_t* data, size_t size)
    {
        BitReader reader(data, size);
        while (reader.getRemainingBitsCount() > 0) {
            obu_header header;
            if (!header.parse(reader))
                return false;
            bool ret;
            ObuType type = header.obu_type;
            uint64_t sz = header.obu_size;
            printf("type = %s, extension = %d, obu_size = %d\r\n", obuType2String(type), header.obu_extension_flag, (int)sz);

            BitReader br(data + (reader.getPos() >> 3), sz);
            if (type == OBU_SEQUENCE_HEADER) {
                ret = m_parser->parseSequenceHeader(br);
            } else if (type == OBU_TD) {
                ret = m_parser->parseTemporalDelimiter(br);
            } else if (type == OBU_FRAME_HEADER) {
                ret = m_parser->parseFrameHeader(br);
            } else if (type == OBU_TILE_GROUP) {
                if (!m_parser->m_frame) {
                    ASSERT(0 && "no frame header");
                }
                TileGroup group;
                ret = m_parser->parseTileGroup(br, group);
                if (ret) {
                    m_tiles.insert(m_tiles.end(), group.begin(), group.end());
                    if (m_tiles.size() == m_parser->m_frame->NumTiles) {
                        ret = decodeFrame(m_tiles);
                        m_tiles.clear();
                    }
                }
            } else if (type == OBU_METADATA) {
                ret = m_parser->parseMetadata(br);
            } else if (type == OBU_PADDING) {
                ret = m_parser->parsePadding(br);
            } else if (type == OBU_FRAME) {
                TileGroup group;
                ret = m_parser->parseFrame(br, group);
                if (ret) {
                    ret = decodeFrame(group);
                }
            } else {
                ret = m_parser->praseReserved(br);
            }
            if (!ret) {
                return false;
            }
            reader.skip(sz << 3);
        }
        return true;
    }
    bool Decoder::decodeFrame(TileGroup tiles)
    {
        FrameHeader& h = *m_parser->m_frame;
        std::shared_ptr<YuvFrame> frame = YuvFrame::create(h.FrameWidth, h.FrameHeight);
        if (!frame)
            return false;
        for (auto& t : tiles) {
            if (!t->decode(frame)) {
                return false;
            }
        }
        m_output.push_back(frame);
        return true;
    }
    std::shared_ptr<YuvFrame> Decoder::getOutput()
    {
        std::shared_ptr<YuvFrame> f;
        if (!m_output.empty()) {
            f = m_output.front();
            m_output.pop_front();
        }
        return f;
    }
        
};
};
