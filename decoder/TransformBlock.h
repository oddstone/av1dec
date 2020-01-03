#pragma once

#include "Block.h"

namespace Yami {

struct YuvFrame;
}

namespace YamiAv1 {

enum TxClass {
    TX_CLASS_2D,
    TX_CLASS_HORIZ,
    TX_CLASS_VERT
};

class TransformBlock {
public:
    TransformBlock(Block& block, int plane, int startX, int startY, TX_SIZE txSz, bool skip);
    void parse();
    bool decode(std::shared_ptr<YuvFrame>& frame);

private:
    TX_TYPE compute_tx_type() const;
    bool is_tx_type_in_set(TxSet txSet, TX_TYPE txType) const;
    TxSet get_tx_set() const;
    uint8_t getAllZeroCtx();
    int16_t get_q_idx();
    void transform_type(TX_TYPE type);
    void transform_type();

    const int16_t* get_mrow_scan() const;
    const int16_t* get_mcol_scan() const;
    const int16_t* get_default_scan() const;
    const int16_t* get_scan() const;

    TxClass get_tx_class() const;

    int getEob(int eobMultisize);
    uint8_t get_coeff_base_ctx(int pos, int c, bool isEob) const;
    uint8_t getCoeffBrCtx(int pos);
    uint8_t getDcSignCtx() const;
    bool getSign(int c, int pos);
    int16_t getLevel();
    int16_t getLevel(int c, int eob, int pos);

    int coeffs();
    int dc_q(int16_t b) const;
    int ac_q(int16_t b) const;
    int get_dc_quant() const;
    int get_ac_quant() const;
    int getQ2(int i, int j) const;
    void inverseTransform();
    void reconstruct();
    PREDICTION_MODE getIntraDir();


    EntropyDecoder& m_entropy;
    Block& m_block;
    Tile& m_tile;
    FrameHeader& m_frame;
    const SequenceHeader& m_sequence;

    bool m_skip;

    int plane;
    PLANE_TYPE ptype;
    int x;
    int y;
    int x4;
    int y4;
    TX_SIZE txSz;
    TX_SIZE txSzSqr;
    TX_SIZE txSzSqrUp;
    TX_SIZE txSzCtx;
    TxClass txClass;
    int w;
    int h;
    int w4;
    int h4;
    int log2W;
    int log2H;

    TX_TYPE PlaneTxType;
    int Quant[1024];
    int Dequant[64][64];
    int Residual[64][64];

    //for reconstruction
    int dqDenom;
    int tw;
    int th;
    bool flipUD;
    bool flipLR;
    int m_eob;
};
}
