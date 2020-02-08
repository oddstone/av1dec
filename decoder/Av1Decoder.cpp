/*
 * Copyright 2020, av1dec authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Av1Decoder.h"
#include "Cdef.h"
#include "LoopFilter.h"
#include "LoopRestoration.h"
#include "Parser.h"
#include "VideoFrame.h"
#include "BitReader.h"
#include "log.h"

namespace YamiAv1 {

using namespace Yami;

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
            if (ret) {
                if (m_frame->show_existing_frame) {
                    showExistingFrame();
                }
            }
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

void Decoder::updateFrameStore(const FrameHeader& h, const std::shared_ptr<YuvFrame> frame)
{
    m_store.resize(NUM_REF_FRAMES);
    for (int i = 0; i < NUM_REF_FRAMES; i++) {
        if (h.refresh_frame_flags & (1 << i)) {
            m_store[i] = frame;
        }
    }
}

void Decoder::frame_end_update_cdf(Tiles& tiles)
{
    for (auto& t : tiles) {
        t->frame_end_update_cdf();
    }
}

bool Decoder::decodeFrame(Tiles& tiles)
{
    FrameHeader& h = *m_frame;
    std::shared_ptr<YuvFrame> frame = YuvFrame::create(h.FrameWidth, h.FrameHeight);
    if (!frame)
        return false;
    for (auto& t : tiles) {
        if (!t->decode(frame, m_store)) {
            return false;
        }
    }
    frame_end_update_cdf(tiles);

    std::shared_ptr<YuvFrame> filtered = decode_frame_wrapup(frame);
//#define DUMP
#ifdef DUMP
    static int i = 0;
    i++;
    //if (i == 2) {
        m_output.push_back(filtered);
    //}
#else
    if (m_frame->show_frame)
        m_output.push_back(filtered);
#endif
    updateFrameStore(h, filtered);
    m_parser->finishFrame();
    return true;
}

void Decoder::showExistingFrame()
{
    FrameHeader& h = *m_frame;

    const std::shared_ptr<YuvFrame>& frame = m_store[h.frame_to_show_map_idx];
    m_output.push_back(frame);
    updateFrameStore(h, frame);

    h.referenceFrameLoading();
    h.motionVectorStorage();
    m_parser->finishFrame();
}

std::shared_ptr<YuvFrame> Decoder::decode_frame_wrapup(const std::shared_ptr<YuvFrame>& frame)
{
    FrameHeader& h = *m_frame;
#ifdef DUMP
    static int i = 0;
    i++;
    if (i == 2) {
        return frame;
    }
#endif
    LoopFilter filter(m_frame);
    filter.filter(frame);

    Cdef cdef(m_frame);
    std::shared_ptr<YuvFrame> CdefFrame = cdef.filter(frame);
    std::shared_ptr<YuvFrame> UpscaledCdefFrame = upscaling(CdefFrame);
    std::shared_ptr<YuvFrame> UpscaledCurrFrame = upscaling(frame);
    LoopRestoration restoration(m_frame, UpscaledCdefFrame, UpscaledCurrFrame);
    std::shared_ptr<YuvFrame> lrFrame = restoration.filter();
    m_frame->motionVectorStorage();
    return lrFrame;
}

std::shared_ptr<YuvFrame> Decoder::upscaling(const std::shared_ptr<YuvFrame>& frame)
{
    FrameHeader& h = *m_parser->m_frame;
    if (!h.use_superres)
        return frame;
    ASSERT(0);
    return frame;
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
}
