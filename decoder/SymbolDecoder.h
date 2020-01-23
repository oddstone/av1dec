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
public:
    SymbolDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update);
    ~SymbolDecoder();
    uint8_t read(std::vector<aom_cdf_prob>& icdf);
    uint8_t readBool();
    uint32_t readNS();
    uint8_t read(std::vector<aom_cdf_prob>& icdf, bool forceDisableUpdate);
    uint8_t read(std::vector<aom_cdf_prob>& icdf, int) = delete;

private:
    void renormalize();
    void updateCdf(std::vector<aom_cdf_prob>& icdf, uint8_t symbol);
    //bool UpdateCdfs;
    //uint32_t MaxTileSize;
    uint16_t SymbolRange;
    uint32_t SymbolValue;
    uint32_t SymbolMaxBits;
    bool DisableCdfUpdate;
    std::unique_ptr<Yami::BitReader> m_reader;
};
}
