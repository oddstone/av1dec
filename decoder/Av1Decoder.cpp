#include "Av1Decoder.h"
#include "Av1Parser.h"
#include "Cdef.h"
#include "LoopFilter.h"
#include "LoopRestoration.h"
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
            //printf("type = %s, extension = %d, obu_size = %d\r\n", obuType2String(type), header.obu_extension_flag, (int)sz);

            BitReader br(data + (reader.getPos() >> 3), sz);
            if (type == OBU_SEQUENCE_HEADER) {
                ret = m_parser->parseSequenceHeader(br);
            } else if (type == OBU_TD) {
                ret = m_parser->parseTemporalDelimiter(br);
            } else if (type == OBU_FRAME_HEADER) {
                m_frame = m_parser->parseFrameHeader(br);
                ret = bool(m_frame);
            } else if (type == OBU_TILE_GROUP) {
                if (!m_frame) {
                    ASSERT(0 && "no frame header");
                }
                TileGroup group;
                ret = m_parser->parseTileGroup(br, m_frame, group);
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
                m_frame = m_parser->parseFrame(br, group);
                if (m_frame) {
                    ret = decodeFrame(group);
                } else {
                    ret = false;
                }
                m_tiles.clear();
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
        FrameHeader& h = *m_frame;
        std::shared_ptr<YuvFrame> frame = YuvFrame::create(h.FrameWidth, h.FrameHeight);
        if (!frame)
            return false;
        for (auto& t : tiles) {
            if (!t->decode(frame)) {
                return false;
            }
        }
        if (!h.disable_frame_end_update_cdf ) {
            printf("warning: disable_frame_end_update_cdf is not supported");
        }
        std::shared_ptr<YuvFrame> filtered = decode_frame_wrapup(frame);
        m_output.push_back(filtered);
        return true;
    }

    std::shared_ptr<YuvFrame> Decoder::decode_frame_wrapup(const std::shared_ptr<YuvFrame>& frame)
    {
        FrameHeader& h = *m_frame;
        if (h.show_existing_frame)
            return frame;
        LoopFilter filter(*m_parser);
        filter.filter(frame);

        Cdef cdef(*m_parser);
        std::shared_ptr<YuvFrame> CdefFrame = cdef.filter(frame);
        std::shared_ptr<YuvFrame> UpscaledCdefFrame = upscaling(CdefFrame);
        std::shared_ptr<YuvFrame> UpscaledCurrFrame = upscaling(frame);
        LoopRestoration restoration(*m_parser, UpscaledCdefFrame, UpscaledCurrFrame);
        std::shared_ptr<YuvFrame> lrFrame = restoration.filter();
        return lrFrame;
    }

    std::shared_ptr<YuvFrame> Decoder::upscaling(const std::shared_ptr<YuvFrame>& frame)
    {
        FrameHeader& h = *m_parser->m_frame;
        if (!h.use_superres)
            return frame;
        ASSERT(0);
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
