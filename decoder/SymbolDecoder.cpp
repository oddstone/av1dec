#include "SymbolDecoder.h"
#include "../aom/prob.h"
#include "bitReader.h"
#include <algorithm>

namespace Yami {
namespace Av1 {
    SymbolDecoder::SymbolDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update)
        : DisableCdfUpdate(disable_cdf_update)
    {
        m_reader.reset(new Yami::BitReader(data, sz));
        uint8_t numBits;
        if (sz <= 1)
            numBits = 8;
        else
            numBits = 15;
        uint16_t buf = m_reader->read(numBits);
        uint16_t paddedBuf = (buf << (15 - numBits));
        SymbolValue = ((1 << 15) - 1) ^ paddedBuf;
        SymbolRange = 1 << 15;
        SymbolMaxBits = 8 * sz - 15;
    }

    SymbolDecoder::~SymbolDecoder()
    {
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
        static uint16_t icdf[] = { AOM_CDF2(1 << 14) };
        return read(icdf, 2, true);
    }

    uint8_t SymbolDecoder::read(uint16_t* icdf, uint8_t nicdf)
    {
        return read(icdf, nicdf, DisableCdfUpdate);
    }

    uint8_t SymbolDecoder::read(uint16_t* icdf, uint8_t nicdf, bool disableUpdate)
    {
        uint16_t cur = SymbolRange;
        uint8_t symbol = -1;
        uint16_t prev;
        uint8_t N = nicdf - 1;
        do {
            symbol++;
            prev = cur;
            uint16_t f = icdf[symbol];
            cur = ((SymbolRange >> 8) * (f >> EC_PROB_SHIFT)) >> (7 - EC_PROB_SHIFT);
            cur += EC_MIN_PROB * (N - symbol);

        } while (SymbolValue < cur);
        SymbolRange = prev - cur;
        SymbolValue -= cur;
        renormalize();
        if (!disableUpdate)
            updateCdf(icdf, nicdf, symbol);
        static FILE* fp = fopen("symbol.txt", "w");
        fprintf(fp, "%d\r\n", SymbolRange);
        fflush(fp);
        //39901 
        if (SymbolRange == 49364)
            printf("%d\r\n", SymbolRange);
        return symbol;
    }

    void SymbolDecoder::updateCdf(uint16_t* cdf, uint8_t ncdf, uint8_t symbol)
    {
        uint16_t N = ncdf;
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

    void SymbolDecoder::renormalize()
    {
        uint32_t bits = 15 - log(SymbolRange);
        SymbolRange <<= bits;
        uint32_t numBits = std::min(bits, std::max(0U, SymbolMaxBits));
        uint16_t newData = m_reader->read(numBits);
        uint16_t paddedData = newData << (bits - numBits);
        SymbolValue = paddedData ^ (((SymbolValue + 1) << bits) - 1);
        SymbolMaxBits = SymbolMaxBits - bits;
    }
}
}