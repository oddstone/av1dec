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

#include "prob.h"
#include <memory>
#include <stdint.h>
#include <vector>

namespace Yami {

class BitReader;
}

namespace YamiAv1 {

class SymbolDecoder {
    typedef uint32_t od_ec_window;
    const int OD_EC_WINDOW_SIZE = ((int)sizeof(od_ec_window) * 8);

public:
    SymbolDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update);
    uint8_t read(std::vector<aom_cdf_prob>& icdf);
    uint8_t readBool();
    uint32_t readNS();
    uint8_t read(std::vector<aom_cdf_prob>& icdf, bool forceDisableUpdate);
    uint8_t read(std::vector<aom_cdf_prob>& icdf, int) = delete;

private:
    void refill();
    int renormalize(od_ec_window dif, unsigned rng, int ret);
    void updateCdf(std::vector<aom_cdf_prob>& icdf, uint8_t symbol);
    //same as libaom
    /*The start of the current input buffer.*/
    const unsigned char* m_buf;
    /*An offset used to keep track of tell after reaching the end of the stream.
    This is constant throughout most of the decoding process, but becomes
     important once we hit the end of the buffer and stop incrementing bptr
     (and instead pretend cnt has lots of bits).*/
    int32_t m_tell_offs;
    /*The end of the current input buffer.*/
    const unsigned char* m_end;
    /*The read pointer for the entropy-coded bits.*/
    const unsigned char* m_bptr;
    /*The difference between the high end of the current range, (low + rng), and
     the coded value, minus 1.
    This stores up to OD_EC_WINDOW_SIZE bits of that difference, but the
     decoder only uses the top 16 bits of the window to decode the next symbol.
    As we shift up during renormalization, if we don't have enough bits left in
     the window to fill the top 16, we'll read in more bits of the coded
     value.*/
    od_ec_window m_dif;
    /*The number of values in the current range.*/
    uint16_t m_rng;
    /*The number of bits of data in the current value.*/
    int16_t m_cnt;

    bool DisableCdfUpdate;
};
}
