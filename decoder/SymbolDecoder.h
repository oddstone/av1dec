#pragma once
#include <memory>
#include <stdint.h>

namespace Yami {

class BitReader;

}

namespace YamiAv1 {
class SymbolDecoder {
public:
    SymbolDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update);
    ~SymbolDecoder();
    uint8_t read(uint16_t* icdf, uint8_t nicdf);
    uint8_t readBool();
    uint32_t readNS();
    uint8_t read(uint16_t* icdf, uint8_t nicdf, bool forceDisableUpdate);

private:
    void renormalize();
    void updateCdf(uint16_t* cdf, uint8_t ncdf, uint8_t symbol);
    //bool UpdateCdfs;
    //uint32_t MaxTileSize;
    uint16_t SymbolRange;
    uint16_t SymbolValue;
    uint32_t SymbolMaxBits;
    bool DisableCdfUpdate;
    std::unique_ptr<Yami::BitReader> m_reader;
};
}
