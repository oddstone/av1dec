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
    int plane;
    int x;
    int y;
    int log2W;
    int log2H;
    int w;
    int h;
    std::shared_ptr<YuvFrame> m_yuv;
    std::vector<std::vector<uint8_t>>& m_pred;
};

}