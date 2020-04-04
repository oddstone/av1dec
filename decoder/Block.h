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

#include "../aom/enums.h"
#include "BlockTree.h"
#include "EntropyDecoder.h"
#include "Tile.h"
#include <memory>
#include <stdint.h>
#include <vector>

namespace Yami {

struct YuvFrame;

}

namespace YamiAv1 {

class SymbolDecoder;
class TransformBlock;
struct ModeInfoBlock;

class Block : public BlockTree {
    friend class TransformBlock;
    class FindMvStack;
    class InterPredict;
    class IntraPredict;
    class ReadRefFrames;

    class Palette {
    public:
        Palette(Block& block);
        void palette_mode_info();
        void palette_tokens();
        bool isPalettePredict(int plane) const;
        void predict_palette(int plane, int startX, int startY, int x, int y, TX_SIZE txSz, std::shared_ptr<YuvFrame>& frame) const;
        void updateFrameContext(int y, int x);

    private:
        uint8_t getHasPaletteYCtx();
        uint8_t getHasPaletteUVCtx();
        uint32_t getPaletteBits(uint32_t minBits);
        std::vector<uint8_t> get_palette_cache(int plane) const;

        const Block& m_block;
        FrameHeader& m_frame;
        const SequenceHeader& m_sequence;
        EntropyDecoder& m_entropy;
        const int bw;
        const int bh;
        const int MiCol;
        const int MiRow;
        uint8_t& PaletteSizeY;
        uint8_t& PaletteSizeUV;
        std::vector<uint8_t> palette_colors_y;
        std::vector<uint8_t> palette_colors_u;
        std::vector<uint8_t> palette_colors_v;
        std::vector<std::vector<uint8_t>> ColorMapY;
        std::vector<std::vector<uint8_t>> ColorMapUV;
    };

    class LocalWarp {
        friend class InterPredict;

    public:
        LocalWarp(Block& block);
        void find_warp_samples();
        void warpEstimation();
        void setupShear();
        bool setupShear(const int warpParams[6], int& alpha, int& beta, int& gamma, int& delta) const;
        int NumSamples = 0;
        bool LocalValid;

    private:
        const ModeInfoBlock& getModeInfo(int row, int col);
        void add_sample(int deltaRow, int deltaCol);

        void resolveDivisor(int64_t d, int& divShift, int& divFactor) const;

        const Block& m_block;
        const Tile& m_tile;
        const FrameHeader& m_frame;

        int NumSamplesScanned = 0;

        const int w4;
        const int h4;
        const int MiRow;
        const int MiCol;
        std::vector<std::vector<int16_t>> CandList;
        int LocalWarpParams[6];
    };

public:
    Block(Tile& tile, int r, int c, BLOCK_SIZE bSize);
    void parse();
    bool decode(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore);

private:
    const ModeInfoBlock& getModeInfo(int row, int col) const ;
    ModeInfoBlock& getModeInfo(int row, int col);
    void intra_segment_id();
    void mode_info();
    void read_cfl_alphas();
    void intra_frame_mode_info();
    void inter_frame_mode_info();

    uint8_t getSkipCtx();
    bool read_skip();

    void read_cdef();
    void read_delta_qindex();
    void read_delta_lf();

    PREDICTION_MODE intra_frame_y_mode();
    void intra_angle_info_y();
    UV_PREDICTION_MODE uv_mode();
    void intra_angle_info_uv();
    void filter_intra_mode_info();

    void palette_tokens();
    void read_block_tx_size();
    void read_tx_size(bool allowSelect);
    uint8_t getTxfmSplitCtx(int row, int col, TX_SIZE txSz);
    void read_var_tx_size(int row, int col, TX_SIZE txSz, int depth);
    void compute_prediction(std::shared_ptr<YuvFrame>& frame, const FrameStore& frameStore);

    void reset_block_context();
    BLOCK_SIZE get_plane_residual_size(int subsize, int plane);
    bool is_tx_type_in_set(TxSet txSet, TX_TYPE txType);
    TX_TYPE compute_tx_type(int plane, TX_SIZE txSz, int blockX, int blockY);
    int get_above_tx_width(int row, int col);
    int get_left_tx_height(int row, int col);
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
    int getSkipModeCtx();
    bool read_skip_mode();
    uint8_t getIsInterCtx();
    void read_is_inter();
    bool needs_interp_filter();
    void inter_block_mode_info();
    void intra_block_mode_info();
    void read_ref_frames();
    bool seg_feature_active(SEG_LVL_FEATURE feature) const;
    int16_t getSegFeature(SEG_LVL_FEATURE feature) const;
    uint8_t getInterpFilterCtx();

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
    const int MiRow;
    const int MiCol;
    const BLOCK_SIZE MiSize;
    const int bw4;
    const int bh4;
    const int bw;
    const int bh;
    bool HasChroma;
    bool AvailU;
    bool AvailL;
    bool AvailUChroma;
    bool AvailLChroma;
    const int subsampling_x;
    const int subsampling_y;
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
    uint8_t PaletteSizeY = 0;
    uint8_t PaletteSizeUV = 0;
    bool use_filter_intra;
    FILTER_INTRA_MODE filter_intra_mode;
    TX_SIZE TxSize;
    int8_t RefFrame[2];
    bool use_intrabc;
    const int sbMask;

    int MaxLumaW;
    int MaxLumaH;

    //for inter
    int8_t LeftRefFrame[2];
    int8_t AboveRefFrame[2];
    bool LeftIntra;
    bool AboveIntra;
    bool LeftSingle;
    bool AboveSingle;

    uint8_t RefMvIdx;
    std::vector<Mv> m_mv;
    bool interintra;
    INTERINTRA_MODE interintra_mode;
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
    Palette m_palette;
    //from tile
    bool& ReadDeltas;
    //keep a local copy of CurrentQIndex
    int CurrentQIndex;
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
