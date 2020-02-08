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

#include <algorithm> /*std::min*/
#include <stdint.h>

namespace Yami {

class BitReader {
public:
    static const uint32_t CACHEBYTES;
    BitReader(const uint8_t* data, uint32_t size);
    virtual ~BitReader() {}

    /* Read specified bits(<= 8*sizeof(uint32_t)) as a uint32_t to v */
    /* if not enough data, it will return false, eat all data and keep v untouched */
    bool read(uint32_t& v, uint32_t nbits);
    /* Read specified bits(<= 8*sizeof(uint32_t)) as a uint32_t return value */
    /* will return 0 if not enough data*/
    uint32_t read(uint32_t nbits);

    /*TODO: change to this to read, and change read to readBits */
    template <class T>
    inline bool readT(T& v, uint32_t nbits);

    template <class T>
    inline bool readT(T& v);

    inline bool readT(bool& v);

    bool readUe(uint32_t& v);

    bool readNs(uint32_t& v, uint32_t n);
    bool readSu(int8_t& v, uint32_t n);
    bool readSu(int16_t& v, uint32_t n);

    bool readLe(uint32_t& v, uint32_t nBytes);

    /*read the next nbits bits from the bitstream but not advance the bitstream pointer*/
    uint32_t peek(uint32_t nbits) const;

    /* this version will check read beyond boundary */
    template <class T>
    bool peek(T& v, uint32_t nbits) const;

    bool skip(uint32_t nbits);

    const uint8_t* getCurrent()
    {
        uint32_t left = (uint32_t)(getRemainingBitsCount() >> 3);

        return m_stream + (m_size - left);
    }
    /* Get the total bits that had been read from bitstream, and the return
     * value also is the position of the next bit to be read. */
    uint64_t getPos() const
    {
        return (static_cast<uint64_t>(m_loadBytes) << 3) - m_bitsInCache;
    }

    uint64_t getRemainingBitsCount() const
    {
        return m_bitsInCache + (static_cast<uint64_t>(m_size - m_loadBytes) << 3);
    }

    bool end() const
    {
        return (getPos() >= (static_cast<uint64_t>(m_size) << 3));
    }

protected:
    virtual void loadDataToCache(uint32_t nbytes);

    const uint8_t* m_stream; /*a pointer to source data*/
    uint32_t m_size; /*the size of source data in bytes*/
    unsigned long int m_cache; /*the buffer which load less than or equal to 8 bytes*/
    uint32_t m_loadBytes; /*the total bytes of data read from source data*/
    uint32_t m_bitsInCache; /*the remaining bits in cache*/
private:
    inline uint32_t extractBitsFromCache(uint32_t nbits);
    inline void reload();
};

template <class T>
bool BitReader::readT(T& v, uint32_t nbits)
{
    uint32_t tmp;
    if (!read(tmp, nbits))
        return false;
    v = (T)tmp;
    return true;
}

template <class T>
bool BitReader::readT(T& v)
{
    return readT(v, sizeof(v) << 3);
}

bool BitReader::readT(bool& v)
{
    return readT(v, 1);
}

template <class T>
bool BitReader::peek(T& v, uint32_t nbits) const
{
    BitReader tmp(*this);
    return tmp.readT(v, nbits);
}

} /*namespace Yami*/
