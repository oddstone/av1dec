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
