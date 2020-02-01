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

namespace Yami {

struct YuvFrame;
}

namespace YamiAv1 {

enum TxClass {
    TX_CLASS_2D,
    TX_CLASS_HORIZ,
    TX_CLASS_VERT
};

class TransformBlock {
public:
    TransformBlock(Block& block, int plane, int baseX, int baseY, int startX, int startY, TX_SIZE txSz, bool skip);
    void parse();
    bool decode(std::shared_ptr<YuvFrame>& frame);

private:
    TX_TYPE compute_tx_type() const;
    bool is_tx_type_in_set(TxSet txSet, TX_TYPE txType) const;
    TxSet get_tx_set() const;
    uint8_t getAllZeroCtx();
    int16_t get_q_idx();
    void transform_type(TX_TYPE type);
    void transform_type();

    const int16_t* get_mrow_scan() const;
    const int16_t* get_mcol_scan() const;
    const int16_t* get_default_scan() const;
    const int16_t* get_scan() const;

    TxClass get_tx_class() const;

    int getEob(int eobMultisize);
    uint8_t get_coeff_base_ctx(int pos, int c, bool isEob) const;
    uint8_t getCoeffBrCtx(int pos);
    uint8_t getDcSignCtx() const;
    bool getSign(int c, int pos);
    int16_t getLevel();
    int16_t getLevel(int c, int eob, int pos);

    int coeffs();
    int dc_q(int16_t b) const;
    int ac_q(int16_t b) const;
    int get_dc_quant() const;
    int get_ac_quant() const;
    int getQ2(int i, int j) const;
    void inverseTransform();
    void reconstruct();
    PREDICTION_MODE getIntraDir();

    EntropyDecoder& m_entropy;
    Block& m_block;
    Tile& m_tile;
    FrameHeader& m_frame;
    const SequenceHeader& m_sequence;

    const bool m_skip;

    const int plane;
    const PLANE_TYPE ptype;
    const int m_baseX;
    const int m_baseY;
    const int x;
    const int y;
    const int x4;
    const int y4;
    const TX_SIZE txSz;
    const TX_SIZE txSzSqr;
    const TX_SIZE txSzSqrUp;
    const TX_SIZE txSzCtx;
    TxClass txClass;
    const int w;
    const int h;
    const int w4;
    const int h4;
    const int log2W;
    const int log2H;

    TX_TYPE PlaneTxType;
    int Quant[1024];
    int Dequant[64][64];
    int Residual[64][64];

    //for reconstruction
    int dqDenom;
    const int tw;
    const int th;
    bool flipUD = false;
    bool flipLR = false;
    int m_eob = 0;
};
}
