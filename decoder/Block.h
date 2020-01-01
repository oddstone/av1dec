#pragma once

#include "../aom/enums.h"
#include "Tile.h"
#include "BlockTree.h"
#include "EntropyDecoder.h"
#include <memory>
#include <stdint.h>

namespace Yami {

struct YuvFrame;
}

namespace YamiAv1 {

class SymbolDecoder;
class TransformBlock;

class Block : public BlockTree {
    friend class TransformBlock;
    class FindMvStack;
    class PredictInter;

    class LocalWarp {
        friend class PredictInter;

    public:
        LocalWarp(Block& block);
        void find_warp_samples();
        void warpEstimation();
        void setupShear();
        bool setupShear(const int warpParams[6], int& alpha, int& beta, int& gamma, int& delta) const;
        int NumSamples = 0;
        bool LocalValid;

    private:
        void add_sample(int deltaRow, int deltaCol);

        void resolveDivisor(int d, int& divShift, int& divFactor) const;

        const Block& m_block;
        const Tile& m_tile;
        const FrameHeader& m_frame;

        int NumSamplesScanned = 0;

        const int w4;
        const int h4;
        const uint32_t MiRow;
        const uint32_t MiCol;
        std::vector<std::vector<int16_t>> CandList;
        int LocalWarpParams[6];
    };

public:
    Block(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE bSize);
    void parse();
    bool decode(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore);

private:
    void intra_segment_id();
    void mode_info();
    void read_cfl_alphas();
    void intra_frame_mode_info();
    void inter_frame_mode_info();

    uint8_t getSkipCtx();
    bool read_skip();

    void read_cdef();
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
    uint8_t getTxfmSplitCtx(uint32_t row, uint32_t col, TX_SIZE txSz);
    void read_var_tx_size(uint32_t row, uint32_t col, TX_SIZE txSz, int depth);
    void compute_prediction(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore);

    void reset_block_context();
    BLOCK_SIZE get_plane_residual_size(int subsize, int plane);
    bool is_tx_type_in_set(TxSet txSet, TX_TYPE txType);
    TX_TYPE compute_tx_type(int plane, TX_SIZE txSz, int blockX, int blockY);
    int get_above_tx_width(uint32_t row, uint32_t col);
    int get_left_tx_height(uint32_t row, uint32_t col);
    uint8_t getTxDepthCtx(TX_SIZE maxRectTxSize);
    TX_SIZE get_tx_size(int plane, TX_SIZE txSz);
    TxSet get_tx_set(TX_SIZE txSz);

    uint8_t getAllZeroCtx(int plane, int x4, int y4, int w4, int h4, TX_SIZE txSz);
    int16_t get_q_idx();

    void transform_block(int plane, int baseX, int baseY, TX_SIZE txSz, int x, int y);
    void transform_tree(int baseX, int baseY, int w, int h);
    void residual();

    //for inter
    void inter_segment_id(bool preSkip);
    bool read_skip_mode();
    uint8_t getIsInterCtx();
    void read_is_inter();
    bool needs_interp_filter();
    void inter_block_mode_info();
    void intra_block_mode_info();
    void read_ref_frames();
    bool seg_feature_active(SEG_LVL_FEATURE feature);
    int16_t getSegFeature(SEG_LVL_FEATURE feature);
    uint8_t getCompModeCtx();
    uint8_t getCompReferenceTypeCtx();
    uint8_t getInterpFilterCtx();

    uint8_t count_refs(uint8_t frameType);
    uint8_t ref_count_ctx(uint8_t counts0, uint8_t counts1);

    uint8_t getUniCompRefCtx();
    uint8_t getUniCompRefP1Ctx();
    uint8_t getUniCompRefP2Ctx();

    uint8_t getCompRefCtx();
    uint8_t getCompRefP1Ctx();
    uint8_t getCompRefP2Ctx();
    uint8_t getCompBwdRefCtx();
    uint8_t getCompBwdRefP1Ctx();

    uint8_t getSingleRefP1Ctx();
    uint8_t getSingleRefP2Ctx();
    uint8_t getSingleRefP3Ctx();
    uint8_t getSingleRefP4Ctx();
    uint8_t getSingleRefP5Ctx();
    uint8_t getSingleRefP6Ctx();

    void readCompReference();
    void readSingleReference();

    int16_t read_mv_component(uint8_t MvCtx, uint8_t comp);
    void read_mv(Mv PredMv[2], int ref);
    void assign_mv(const FindMvStack& find, bool isCompound);
    PREDICTION_MODE get_mode(int refList);
    void read_interintra_mode(bool isCompound);
    void read_motion_mode(bool isCompound);
    bool has_overlappable_candidates();
    void read_compound_type(bool isCompound);
    uint8_t getCompGroupIdxCtx();
    uint8_t getCompoundIdxCtx();

    uint8_t getInterpFilterCtx(int dir);

    FrameHeader& m_frame;
    const SequenceHeader& m_sequence;
    Tile& m_tile;
    BlockDecoded& m_decoded;

    //from spec
    uint32_t MiRow;
    uint32_t MiCol;
    BLOCK_SIZE MiSize;
    uint32_t bw4;
    uint32_t bh4;
    uint32_t bw;
    uint32_t bh;
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
    bool skip_mode;
    bool skip;
    PREDICTION_MODE YMode;
    int8_t AngleDeltaY;
    UV_PREDICTION_MODE UVMode;
    int8_t AngleDeltaUV;
    int8_t CflAlphaU;
    int8_t CflAlphaV;
    uint32_t PaletteSizeY = 0;
    uint32_t PaletteSizeUV = 0;
    bool use_filter_intra;
    FILTER_INTRA_MODE filter_intra_mode;
    TX_SIZE TxSize;
    int8_t RefFrame[2];
    bool use_intrabc;
    uint32_t sbMask;

    int MaxLumaW;
    int MaxLumaH;

    //for inter
    uint8_t LeftRefFrame[2];
    uint8_t AboveRefFrame[2];
    bool LeftIntra;
    bool AboveIntra;
    bool LeftSingle;
    bool AboveSingle;

    uint8_t RefMvIdx;
    std::vector<Mv> m_mv;
    bool interintra;
    bool wedge_interintra;
    uint8_t wedge_index;
    uint8_t wedge_sign;
    MOTION_MODE motion_mode;

    bool comp_group_idx;
    bool compound_idx;
    COMPOUND_TYPE compound_type;
    bool mask_type;

    InterpFilter interp_filter[2];

    EntropyDecoder& m_entropy;
    std::deque<std::shared_ptr<TransformBlock>> m_transformBlocks;
    LocalWarp m_localWarp;

    bool has_nearmv() const;
};

inline bool is_directional_mode(PREDICTION_MODE mode)
{
    return (mode >= V_PRED) && (mode <= D67_PRED);
}

inline bool is_directional_mode(UV_PREDICTION_MODE mode)
{
    return (mode >= UV_V_PRED) && (mode <= UV_D67_PRED);
}

inline bool is_directional_mode(int mode)
{
    return is_directional_mode((PREDICTION_MODE)mode);
}
}
