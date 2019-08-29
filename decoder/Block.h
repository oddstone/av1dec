#ifndef Block_h
#define Block_h

#include "../aom/enums.h"
#include "Av1Tile.h"
#include "BlockTree.h"
#include "EntropyDecoder.h"
#include <memory>
#include <stdint.h>

namespace YamiParser {
namespace Av1 {
    class SymbolDecoder;
    struct FrameHeader;
    struct SequenceHeader;
    struct FrameHeader;
}
}

using namespace YamiParser::Av1;

struct YuvFrame;

class Block    : public BlockTree {
    friend class TransformBlock;

public:
    Block(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE bSize);
    void parse();
    bool decode(std::shared_ptr<YuvFrame>& frame);

private:
    void intra_segment_id();
    void mode_info();
    void read_cfl_alphas();
    void intra_frame_mode_info();
    void inter_frame_mode_info();

    uint8_t getSkipCtx();
    bool readSkip();

    void readCdef();
    void read_delta_qindex(bool readDeltas);
    void read_delta_lf(bool readDeltas);

    PREDICTION_MODE intra_frame_y_mode();
    void intra_angle_info_y();
    UV_PREDICTION_MODE uv_mode();
    void intra_angle_info_uv();
    void filter_intra_mode_info();

    void palette_tokens();
    void read_block_tx_size();
    void read_tx_size(bool allowSelect);
    void compute_prediction();

    void reset_block_context();
    BLOCK_SIZE get_plane_residual_size(int subsize, int plane);
    bool is_tx_type_in_set(TxSet txSet, TX_TYPE txType);
    TX_TYPE compute_tx_type(int plane, TX_SIZE txSz, int blockX, int blockY);
    int get_above_tx_width(uint32_t row, uint32_t col);
    int get_left_tx_height(uint32_t         row, uint32_t col);
    uint8_t getTxDepthCtx(TX_SIZE maxRectTxSize);
    TX_SIZE get_tx_size(int plane, TX_SIZE txSz);
    TxSet get_tx_set(TX_SIZE txSz);


    void predict_intra(int plane, int startX, int startY,
        int availL, int availU, bool decodedUpRight, bool decodedBottomLeft,
        int mode, int log2W, int log2H);
    uint8_t getAllZeroCtx(int plane, int x4, int y4, int w4, int h4, TX_SIZE txSz);
    int16_t get_q_idx();

    void transform_block(int plane, int baseX, int baseY, TX_SIZE txSz, int x, int y);
    void residual();

    FrameHeader& m_frame;
    const SequenceHeader& m_sequence;
    Tile& m_tile;
    BlockDecoded m_decoded;

    //from spec
    uint32_t MiRow;
    uint32_t MiCol;
    BLOCK_SIZE MiSize;
    uint32_t bw4;
    uint32_t bh4;
    bool HasChroma;
    bool AvailU;
    bool AvailL;
    bool AvailUChroma;
    bool AvailLChroma;
    int subsampling_x;
    int subsampling_y;
    bool is_inter;
    uint8_t segment_id;
    bool Lossless;
    bool skip;
    PREDICTION_MODE YMode;
    uint8_t AngleDeltaY;
    UV_PREDICTION_MODE UVMode;
    uint8_t AngleDeltaUV;
    int8_t CflAlphaU;
    int8_t CflAlphaV;
    uint32_t PaletteSizeY = 0;
    uint32_t PaletteSizeUV = 0;
    bool use_filter_intra;
    FILTER_INTRA_MODE filter_intra_mode;
    TX_SIZE TxSize;
    int RefFrame[2];
    uint32_t sbMask;

    EntropyDecoder& m_entropy;
    std::deque<std::shared_ptr<TransformBlock>> m_transformBlocks;
};
#endif
