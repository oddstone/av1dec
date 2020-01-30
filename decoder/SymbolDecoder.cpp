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

#include "SymbolDecoder.h"
#include "../aom/prob.h"
#include "bitReader.h"
#include <algorithm>

namespace YamiAv1 {

using std::vector;

SymbolDecoder::SymbolDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update)
    : DisableCdfUpdate(disable_cdf_update)
{
    m_buf = data;
    m_tell_offs = 10 - (OD_EC_WINDOW_SIZE - 8);
    m_end = data + sz;
    m_bptr = data;
    m_dif = ((od_ec_window)1 << (OD_EC_WINDOW_SIZE - 1)) - 1;
    m_rng = 0x8000;
    m_cnt = -15;
    refill();
}

static const uint8_t EC_PROB_SHIFT = 6;
static const uint8_t EC_MIN_PROB = 4;

static uint8_t log(uint16_t n)
{
    uint8_t ret = 0;
    while (n >>= 1) {
        ret++;
    }
    return ret;
}

uint8_t SymbolDecoder::readBool()
{
    static vector<aom_cdf_prob> icdf = { AOM_CDF2(1 << 14) };
    return read(icdf, true);
}

uint8_t SymbolDecoder::read(vector<aom_cdf_prob>& icdf)
{
    return read(icdf, DisableCdfUpdate);
}

uint8_t SymbolDecoder::read(vector<aom_cdf_prob>& icdf, bool disableUpdate)
{
    od_ec_window dif;
    unsigned r;
    unsigned c;
    unsigned u;
    unsigned v;
    int ret;
    dif = m_dif;
    r = m_rng;
    const int nsyms = icdf.size() - 1;
    const int N = nsyms - 1;

//#define DUMP_SYMBOL
#ifdef DUMP_SYMBOL
    static FILE* fp = fopen("symbol.txt", "w");
    fprintf(fp, "dif = %u, rng = %d, cnt = %d\r\n", m_dif, m_rng, m_cnt);
    fflush(fp);
#endif
    c = (unsigned)(dif >> (OD_EC_WINDOW_SIZE - 16));
    v = r;
    ret = -1;
    do {
        u = v;
        v = ((r >> 8) * (uint32_t)(icdf[++ret] >> EC_PROB_SHIFT) >> (7 - EC_PROB_SHIFT - CDF_SHIFT));
        v += EC_MIN_PROB * (N - ret);
    } while (c < v);

    r = u - v;
    dif -= (od_ec_window)v << (OD_EC_WINDOW_SIZE - 16);

    if (!disableUpdate)
        updateCdf(icdf, ret);
    return renormalize(dif, r, ret);
}

void SymbolDecoder::updateCdf(vector<aom_cdf_prob>& cdf, uint8_t symbol)
{
    uint16_t N = cdf.size() - 1;
    uint8_t two = 2;
    uint8_t rate = 3 + (cdf[N] > 15) + (cdf[N] > 31) + std::min(log(N), two);
    uint16_t tmp = 1 << 15;

    for (uint16_t i = 0; i < N - 1; i++) {
        tmp = (i == symbol) ? 0 : tmp;
        if (tmp < cdf[i]) {
            cdf[i] -= ((cdf[i] - tmp) >> rate);
        } else {
            cdf[i] += ((tmp - cdf[i]) >> rate);
        }
    }
    cdf[N] += (cdf[N] < 32);
}

int SymbolDecoder::renormalize(od_ec_window dif, unsigned rng, int ret)
{
    int d;
    /*The number of leading zeros in the 16-bit binary representation of rng.*/
    d = 15 - log(rng);
    /*d bits in m_dif are consumed.*/
    m_cnt -= d;
    /*This is equivalent to shifting in 1's instead of 0's.*/
    m_dif = ((dif + 1) << d) - 1;
    m_rng = rng << d;
    if (m_cnt < 0)
        refill();
    return ret;
}

#define OD_EC_LOTS_OF_BITS (0x4000)

void SymbolDecoder::refill()
{
    int s;
    od_ec_window dif;
    int16_t cnt;
    const unsigned char* bptr;
    const unsigned char* end;
    dif = m_dif;
    cnt = m_cnt;
    bptr = m_bptr;
    end = m_end;
    s = OD_EC_WINDOW_SIZE - 9 - (cnt + 15);
    for (; s >= 0 && bptr < end; s -= 8, bptr++) {
        /*Each time a byte is inserted into the window (dif), bptr advances and cnt
       is incremented by 8, so the total number of consumed bits (the return
       value of od_ec_dec_tell) does not change.*/
        assert(s <= OD_EC_WINDOW_SIZE - 8);
        dif ^= (od_ec_window)bptr[0] << s;
        cnt += 8;
    }
    if (bptr >= end) {
        /*We've reached the end of the buffer. It is perfectly valid for us to need
       to fill the window with additional bits past the end of the buffer (and
       this happens in normal operation). These bits should all just be taken
       as zero. But we cannot increment bptr past 'end' (this is undefined
       behavior), so we start to increment m_tell_offs. We also don't want
       to keep testing bptr against 'end', so we set cnt to OD_EC_LOTS_OF_BITS
       and adjust m_tell_offs so that the total number of unconsumed bits in
       the window (m_cnt - m_tell_offs) does not change. This effectively
       puts lots of zero bits into the window, and means we won't try to refill
       it from the buffer for a very long time (at which point we'll put lots
       of zero bits into the window again).*/
        m_tell_offs += OD_EC_LOTS_OF_BITS - cnt;
        cnt = OD_EC_LOTS_OF_BITS;
    }
    m_dif = dif;
    m_cnt = cnt;
    m_bptr = bptr;
}

}
