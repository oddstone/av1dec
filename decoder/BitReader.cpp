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


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "BitReader.h"
#include <assert.h>

#define LOAD8BYTESDATA_BE(x) \
    (((uint64_t)((const uint8_t*)(x))[0] << 56) | ((uint64_t)((const uint8_t*)(x))[1] << 48) | ((uint64_t)((const uint8_t*)(x))[2] << 40) | ((uint64_t)((const uint8_t*)(x))[3] << 32) | ((uint64_t)((const uint8_t*)(x))[4] << 24) | ((uint64_t)((const uint8_t*)(x))[5] << 16) | ((uint64_t)((const uint8_t*)(x))[6] << 8) | ((uint64_t)((const uint8_t*)(x))[7]))

namespace Yami {

const uint32_t BitReader::CACHEBYTES = sizeof(unsigned long int);

BitReader::BitReader(const uint8_t* pdata, uint32_t size)
    : m_stream(pdata)
    , m_size(size)
    , m_cache(0)
    , m_loadBytes(0)
    , m_bitsInCache(0)
{
    //assert(pdata && size);
}

void BitReader::loadDataToCache(uint32_t nbytes)
{
    unsigned long int tmp = 0;
    const uint8_t* pStart = m_stream + m_loadBytes;

    if (nbytes == 8)
        tmp = LOAD8BYTESDATA_BE(pStart);
    else {
        for (uint32_t i = 0; i < nbytes; i++) {
            tmp <<= 8;
            tmp |= pStart[i];
        }
    }

    m_cache = tmp;
    m_loadBytes += nbytes;
    m_bitsInCache = nbytes << 3;
}

inline void BitReader::reload()
{
    assert(m_size >= m_loadBytes);
    uint32_t remainingBytes = m_size - m_loadBytes;
    if (remainingBytes > 0)
        loadDataToCache(std::min(remainingBytes, CACHEBYTES));
}

inline uint32_t BitReader::extractBitsFromCache(uint32_t nbits)
{
    if (!nbits)
        return 0;
    uint32_t tmp = 0;
    tmp = m_cache << ((CACHEBYTES << 3) - m_bitsInCache) >> ((CACHEBYTES << 3) - nbits);
    m_bitsInCache -= nbits;
    return tmp;
}

bool BitReader::read(uint32_t& v, uint32_t nbits)
{
    assert(nbits <= (CACHEBYTES << 3));

    if (nbits <= m_bitsInCache) {
        v = extractBitsFromCache(nbits);
        return true;
    }
    /*not enough bits, need to save remaining bits
      in current cache and then reload more bits*/
    uint32_t toBeReadBits = nbits - m_bitsInCache;
    uint32_t tmp = extractBitsFromCache(m_bitsInCache);
    reload();
    if (toBeReadBits > m_bitsInCache)
        return false;
    v = tmp << toBeReadBits | extractBitsFromCache(toBeReadBits);
    return true;
}

uint32_t BitReader::read(uint32_t nbits)
{
    uint32_t res;
    if (read(res, nbits))
        return res;
    return 0;
}

bool BitReader::skip(uint32_t nbits)
{
    uint32_t tmp;
    uint32_t size = sizeof(tmp) << 3;
    while (nbits > size) {
        if (!readT(tmp))
            return false;
        nbits -= size;
    }
    if (!read(tmp, nbits))
        return false;
    return true;
}

uint32_t BitReader::peek(uint32_t nbits) const
{
    BitReader tmp(*this);

    return tmp.read(nbits);
}

uint32_t floorLog2(uint32_t n)
{
    if (!n)
        return 0;
    int i = 0;
    while ((1 << i) <= n) {
        i++;
    }
    return i - 1;
}

bool BitReader::readNs(uint32_t& v, uint32_t n)
{
    uint32_t w = floorLog2(n) + 1;
    uint32_t m = (1 << w) - n;
    if (!read(v, w - 1))
        return false;
    if (v < m)
        return true;
    uint32_t extra;
    if (!read(extra, 1))
        return false;
    v = (v << 1) - m + extra;
    return true;
}
bool BitReader::readSu(int8_t& v, uint32_t n)
{
    int16_t value;
    if (!readSu(value, n))
        return false;
    v = (int8_t)value;
    return true;
}

bool BitReader::readSu(int16_t& v, uint32_t n)
{
    uint32_t value;
    if (!read(value, n))
        return false;
    uint32_t signMask = 1 << (n - 1);
    if (value & signMask)
        v = (int16_t)(value - 2 * signMask);
    else
        v = (int16_t)value;
    return true;
}

bool BitReader::readLe(uint32_t& v, uint32_t nBytes)
{
    v = 0;
    for (uint32_t i = 0; i < nBytes; i++) {
        uint8_t byte;
        if (!readT(byte))
            return false;
        v += (byte << (i * 8));
    }
    return true;
}

bool BitReader::readUe(uint32_t& v)
{
    int32_t leadingZeroBits = -1;

    for (uint32_t b = 0; !b; leadingZeroBits++) {
        if (!read(b, 1))
            return false;
    }

    if (!read(v, leadingZeroBits))
        return false;
    v = (1 << leadingZeroBits) - 1 + v;
    return true;
}

} /*namespace Yami*/
