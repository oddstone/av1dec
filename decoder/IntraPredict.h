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

#pragma once
#include "Block.h"

#include <vector>

namespace YamiAv1 {

class Block::IntraPredict {
public:
    IntraPredict(const Block& block, const std::shared_ptr<YuvFrame>& m_yuv,
        int plane, int startX, int startY, int log2w, int log2h,
        std::vector<std::vector<uint8_t>>& pred);
    void predict_intra(int availL, int availU, bool decodedUpRight, bool decodedBottomLeft, int mode);
    void predict_chroma_from_luma(TX_SIZE txSz);

private:
    void recursiveIntraPrediction(const uint8_t* AboveRow, const uint8_t* LeftCol,
        const std::shared_ptr<YuvFrame>& frame);
    void paethPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);
    void dcPredict(bool haveAbove, bool haveLeft,
        const uint8_t* AboveRow, const uint8_t* LeftCol);
    bool getLeftSmooth() const;
    bool getAboveSmooth() const;
    bool getSmooth(int r, int c) const;
    bool get_filter_type(bool haveLeft, bool haveAbove) const;
    uint8_t* intraEdgeUpsample(const uint8_t* edge, int numPx, std::vector<uint8_t>& upsampled) const;
    void directionalIntraPredict(bool haveAbove, bool haveLeft, uint8_t* AboveRow, uint8_t* LeftCol,
        int mode);
    void smoothPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);
    void smoothVPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);
    void smoothHPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);

    const Block& m_block;
    const Tile& m_tile;
    const FrameHeader& m_frame;
    const SequenceHeader& m_sequence;
    const int plane;
    const int x;
    const int y;
    const int log2W;
    const int log2H;
    const int w;
    const int h;
    std::shared_ptr<YuvFrame> m_yuv;
    std::vector<std::vector<uint8_t>>& m_pred;
};

}