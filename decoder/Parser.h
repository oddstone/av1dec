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

#include "Tile.h"
#include "bitReader.h"
#include <list>
#include <memory>
#include <stdint.h>
#include <string.h>
#include <vector>

namespace YamiAv1 {

#define CLIP3(min, max, v) ((v) > (max) ? (max) : (((v) < (min)) ? (min) : (v)))
class Block;
class TransformBlock;

struct SequenceHeader;
struct FrameHeader;
typedef std::vector<std::shared_ptr<Tile>> TileGroup;

using Yami::BitReader;

struct obu_extension_header {
    uint8_t temporal_id;
    uint8_t spatial_id;
    uint8_t quality_id;
    bool parse(BitReader& br);
};

enum ObuType {
    OBU_SEQUENCE_HEADER = 1,
    OBU_TD = 2,
    OBU_FRAME_HEADER = 3,
    OBU_TILE_GROUP = 4,
    OBU_METADATA = 5,
    OBU_FRAME = 6,
    OBU_REDUNDANT_FRAME_HEADER = 7,
    OBU_PADDING = 15,
};
const char* obuType2String(uint8_t type);

struct obu_header {
    bool parse(BitReader& br);
    ObuType obu_type;
    bool obu_extension_flag;
    bool obu_has_size_field;
    obu_extension_header extension;
    uint64_t obu_size;

private:
    bool parseLeb128(BitReader& br, uint64_t& v);
};

enum ColorPrimaries {
    CP_BT_709 = 1,
    CP_UNSPECIFIED = 2,
    CP_BT_470_M = 4,
    CP_BT_470_B_G = 5,
    CP_BT_601 = 6,
    CP_SMPTE_240 = 7,
    CP_GENERIC_FILM = 8,
    CP_BT_2020 = 9,
    CP_XYZ = 10,
    CP_SMPTE_431 = 11,
    CP_SMPTE_432 = 12,
    CP_EBU_3213 = 22,
};

enum TransferCharacteristic {
    TC_RESERVED_0 = 0,
    TC_BT_709 = 1,
    TC_UNSPECIFIED = 2,
    TC_RESERVED_3 = 3,
    TC_BT_470_M = 4,
    TC_BT_470_B_G = 5,
    TC_BT_601 = 6,
    TC_SMPTE_240 = 7,
    TC_LINEAR = 8,
    TC_LOG_100 = 9,
    TC_LOG_100_SQRT10 = 10,
    TC_IEC_61966 = 11,
    TC_BT_1361 = 12,
    TC_SRGB = 13,
    TC_BT_2020_10_BIT = 14,
    TC_BT_2020_12_BIT = 15,
    TC_SMPTE_2084 = 16,
    TC_SMPTE_428 = 17,
    TC_HLG = 18,
};

enum MatrixCoefficient {
    MC_IDENTITY = 0,
    MC_BT_709 = 1,
    MC_UNSPECIFIED = 2,
    MC_RESERVED_3 = 3,
    MC_FCC = 4,
    MC_BT_470_B_G = 5,
    MC_BT_601 = 6,
    MC_SMPTE_240 = 7,
    MC_SMPTE_YCGCO = 8,
    MC_BT_2020_NCL = 9,
    MC_BT_2020_CL = 10,
    MC_SMPTE_2085 = 11,
    MC_CHROMAT_NCL = 12,
    MC_CHROMAT_CL = 13,
    MC_ICTCP = 14,
};

enum ChromaSamplePosition {
    CSP_UNKNOWN = 0,
    CSP_VERTICAL = 1,
    CSP_COLOCATED = 2,
    CSP_RESERVED = 3,
};

const static uint8_t SELECT_SCREEN_CONTENT_TOOLS = 2;
const static uint8_t SELECT_INTEGER_MV = 2;

struct SequenceHeader {
    uint8_t seq_profile;
    bool still_picture;
    bool reduced_still_picture_header;
    bool timing_info_present_flag;
    bool decoder_model_info_present_flag;
    bool initial_display_delay_present_flag;

    uint8_t frame_width_bits_minus_1;
    uint8_t frame_height_bits_minus_1;
    uint32_t max_frame_width_minus_1;
    uint32_t max_frame_height_minus_1;

    bool frame_id_numbers_present_flag;
    uint8_t delta_frame_id_length_minus2;
    uint8_t additional_frame_id_length_minus1;

    bool use_128x128_superblock;

    bool enable_filter_intra;
    bool enable_intra_edge_filter;
    bool enable_interintra_compound;
    bool enable_masked_compound;
    bool enable_warped_motion;
    bool enable_dual_filter;

    bool enable_order_hint;
    bool enable_jnt_comp;
    bool enable_ref_frame_mvs;

    //bool seq_choose_screen_content_tools;
    uint8_t seq_force_screen_content_tools;
    bool seq_choose_integer_mv;
    uint8_t seq_force_integer_mv;

    //uint8_t order_hint_bits_minus1;
    uint8_t OrderHintBits;
    bool enable_superres;
    bool enable_cdef;
    bool enable_restoration;

    //clolor_config()
    uint8_t BitDepth;
    bool mono_chrome;
    uint8_t NumPlanes;
    uint8_t color_primaries;
    uint8_t transfer_characteristics;
    uint8_t matrix_coefficients;

    uint8_t color_space;
    uint8_t transfer_function;
    bool color_range;
    int subsampling_x;
    int subsampling_y;
    uint8_t chroma_sample_position;
    bool separate_uv_delta_q;

    uint32_t num_units_in_tick;
    uint32_t time_scale;
    bool equal_picture_interval;
    uint32_t num_ticks_per_picture_minus1;

    bool film_grain_params_present;

    struct OperatingPoint {
        OperatingPoint()
        {
            memset(this, 0, sizeof(*this));
        }
        uint16_t operating_point_idc;
        uint8_t seq_level_idx;
        bool seq_tier;
        bool decoder_model_present_for_this_op;
        bool initial_display_delay_present_for_this_op;
        //depends on decoder_rate_model_param_present_flag;
        //uint16_t decode_to_display_rate_ratio;
        //uint32_t initial_display_delay;
        //uint8_t extra_frame_buffers;
    };
    std::vector<OperatingPoint> operating_points;

    bool parse(BitReader& br);

    //utils function
    BLOCK_SIZE get_plane_residual_size(int subsize, int plane) const;

private:
    bool parseColorConfig(BitReader& br);
    bool parseTimingInfo(BitReader& br);
};

class Parser;
struct FrameHeader;
enum FrameType {
    KEY_FRAME = 0,
    INTER_FRAME = 1,
    INTRA_ONLY_FRAME = 2,
    SWITCH_FRAME = 3,
};

enum TXMode {
    ONLY_4X4 = 0,
    TX_MODE_LARGEST = 1,
    TX_MODE_SELECT = 2,
};
const static uint8_t NUM_REF_FRAMES = 8;
const static uint8_t REFS_PER_FRAME = 7;
const static uint8_t PRIMARY_REF_NONE = 7;
const static uint16_t MAX_TILE_WIDTH = 4096;
const static int MAX_TILE_AREA = 4906 * 2304;
const static int MAX_TILE_COLS = 64;
const static int MAX_TILE_ROWS = 64;

class RefFrame {
public:
    RefFrame()
        : RefValid(false)
    {
    }

    bool RefValid;
    uint32_t RefFrameId;
    int RefUpscaledWidth;
    int RefFrameWidth;
    int RefFrameHeight;
    int RefRenderWidth;
    int RefRenderHeight;
    int RefMiCols;
    int RefMiRows;
    uint8_t RefFrameType;
    int RefSubsamplingX;
    int RefSubsamplingY;
    uint8_t RefBitDepth;
    uint8_t RefOrderHint;
    uint8_t SavedOrderHints[NUM_REF_FRAMES];

    //FrameStore
    std::vector<std::vector<int8_t>> SavedRefFrames;
    std::vector<std::vector<Mv>> SavedMvs;

    std::shared_ptr<Cdfs> SavedCdfs;
    int SavedGmParams[NUM_REF_FRAMES][6];
    std::vector<std::vector<uint8_t>> SavedSegmentIds;

    int8_t loop_filter_mode_deltas[2];
    int8_t loop_filter_ref_deltas[TOTAL_REFS_PER_FRAME];

    bool FeatureEnabled[MAX_SEGMENTS][SEG_LVL_MAX];
    int16_t FeatureData[MAX_SEGMENTS][SEG_LVL_MAX];
};

class RefInfo {
public:
    std::vector<RefFrame> m_refs;
    RefInfo()
    {
        m_refs.resize(NUM_REF_FRAMES);
    }
    void resetRefs()
    {
        for (auto& r : m_refs) {
            r.RefValid = false;
            r.RefOrderHint = 0;
        }
    }
};

struct Quantization {
    uint8_t base_q_idx;
    int8_t DeltaQYDc;
    int8_t DeltaQUDc;
    int8_t DeltaQUAc;
    int8_t DeltaQVDc;
    int8_t DeltaQVAc;
    bool using_qmatrix;
    uint8_t qm_y;
    uint8_t qm_u;
    uint8_t qm_v;
    bool parse(BitReader& br, const SequenceHeader& seq);
};

static const int MAX_LOOP_FILTER = 63;
static const int Segmentation_Feature_Bits[SEG_LVL_MAX] = { 8, 6, 6, 6, 6, 3, 0, 0 };
static const int Segmentation_Feature_Signed[SEG_LVL_MAX] = { 1, 1, 1, 1, 1, 0, 0, 0 };
static const int Segmentation_Feature_Max[SEG_LVL_MAX] = {
    255, MAX_LOOP_FILTER, MAX_LOOP_FILTER,
    MAX_LOOP_FILTER, MAX_LOOP_FILTER, 7,
    0, 0
};

struct Segmentation {

    bool parse(BitReader& br);

    bool segmentation_enabled;
    bool FeatureEnabled[MAX_SEGMENTS][SEG_LVL_MAX];
    int16_t FeatureData[MAX_SEGMENTS][SEG_LVL_MAX];
    bool SegIdPreSkip;
    uint8_t LastActiveSegId;
    bool seg_feature_active_idx(int segmentId, SEG_LVL_FEATURE) const;
    void setup_past_independence();
    void load_segmentation_params(const RefFrame& ref);
    void save_segmentation_params(RefFrame& ref);

private:
    void resetFeatures();
};

struct DeltaQ {
    bool delta_q_present;
    uint8_t delta_q_res;
    bool parse(BitReader& br, const Quantization& quant);
};

struct DeltaLf {
    bool delta_lf_present;
    uint8_t delta_lf_res;
    bool delta_lf_multi;
    bool parse(BitReader& br, const DeltaQ& deltaQ);
};

struct LoopFilterParams {
    const static int LOOP_FILTER_LEVEL_COUNT = 4;
    bool loop_filter_delta_enabled;
    uint8_t loop_filter_level[LOOP_FILTER_LEVEL_COUNT];
    int8_t loop_filter_ref_deltas[TOTAL_REFS_PER_FRAME];
    uint8_t loop_filter_sharpness;
    int8_t loop_filter_mode_deltas[2];
    bool parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame);
    void setup_past_independence();
    void load_loop_filter_params(const RefFrame& ref);
    void save_loop_filter_params(RefFrame& ref);

private:
    void resetDeltas();
};

struct CdefParams {
    bool parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame);
    void read_cdef(EntropyDecoder& entropy, int MiRow, int MiCol, uint32_t MiSize);
    const static int CDEF_SIZE = 8;
    uint8_t CdefDamping;
    uint8_t cdef_bits;
    uint8_t cdef_y_pri_strength[CDEF_SIZE];
    uint8_t cdef_y_sec_strength[CDEF_SIZE];
    uint8_t cdef_uv_pri_strength[CDEF_SIZE];
    uint8_t cdef_uv_sec_strength[CDEF_SIZE];

    std::vector<std::vector<int>> cdef_idx;
};

struct LoopRestorationpParams {
    bool parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame);
    void read_lr(Tile& tile, int r, int c, BLOCK_SIZE bSize);
    void resetRefs(int NumPlanes);

    static const int MAX_PLANES = 3;
    static const int MAX_PASSES = 2;
    static const int MAX_WIENER_COEFFS = 3;

    bool UsesLr;
    RestorationType FrameRestorationType[MAX_PLANES];
    int LoopRestorationSize[MAX_PLANES];

    std::vector<std::vector<std::vector<RestorationType>>> LrType;
    std::vector<std::vector<std::vector<std::vector<std::vector<int8_t>>>>> LrWiener;
    std::vector<std::vector<std::vector<int8_t>>> RefLrWiener;

    std::vector<std::vector<std::vector<uint8_t>>> LrSgrSet;
    std::vector<std::vector<std::vector<std::vector<int8_t>>>> LrSgrXqd;
    std::vector<std::vector<int8_t>> RefSgrXqd;

private:
    void read_lr_unit(EntropyDecoder& entropy,
        int plane, int unitRow, int unitCol);
    int unitRows[MAX_PLANES];
    int unitCols[MAX_PLANES];
};

const static uint8_t REF_SCALE_SHIFT = 14;
const static uint8_t SUBPEL_BITS = 4;
const static uint8_t SCALE_SUBPEL_BITS = 10;

struct FrameHeader {
    friend class Block;
    friend class TransformBlock;
    friend class SetFrameRefs;
    bool show_existing_frame;
    uint8_t frame_to_show_map_idx;
    uint32_t display_frame_id;
    uint8_t refresh_frame_flags;
    uint8_t frame_type;
    bool FrameIsIntra;
    bool show_frame;
    bool showable_frame;
    bool error_resilient_mode;
    bool disable_cdf_update;
    bool allow_screen_content_tools;
    bool force_integer_mv;
    uint32_t PrevFrameID;
    uint32_t current_frame_id;
    bool frame_size_override_flag;
    uint8_t OrderHint;
    uint8_t primary_ref_frame;
    bool allow_high_precision_mv;
    bool use_ref_frame_mvs;
    bool allow_intrabc;

    //for inter frame
    bool frame_refs_short_signaling;
    uint8_t last_frame_idx;
    uint8_t gold_frame_idx;
    uint8_t ref_frame_idx[REFS_PER_FRAME];
    uint8_t OrderHints[REFS_PER_FRAME];
    bool RefFrameSignBias[REFS_PER_FRAME];

    //frame_size()
    uint32_t FrameWidth;
    uint32_t FrameHeight;

    //compute_image_size()
    int MiCols;
    int MiRows;

    uint32_t w8;
    uint32_t h8;

    //cols and rows align to super block size
    uint32_t AlignedMiCols;
    uint32_t AlignedMiRows;

    //superres_params()
    bool use_superres;
    uint8_t SuperresDenom;
    uint32_t UpscaledWidth;

    //render_size()
    uint32_t RenderWidth;
    uint32_t RenderHeight;

    InterpFilter interpolation_filter;
    bool is_motion_mode_switchable;

    bool disable_frame_end_update_cdf;

    //tile_info()
    uint32_t TileCols;
    uint32_t TileRows;
    uint32_t NumTiles;
    uint32_t TileColsLog2;
    uint32_t TileRowsLog2;
    uint32_t context_update_tile_id;
    uint8_t TileSizeBytes;

    bool allow_warped_motion;

    bool CodedLossless;
    bool AllLossless;
    bool LosslessArray[MAX_SEGMENTS];
    uint8_t SegQMLevel[3][MAX_SEGMENTS];

    bool reduced_tx_set;

    TXMode TxMode;

    //for inter
    bool reference_select;
    bool skip_mode_present;
    uint8_t SkipModeFrame[2];

    const static int MAX_PLANES = 3;
    GlobalMotionType GmType[NUM_REF_FRAMES];
    int gm_params[NUM_REF_FRAMES][6];
    int PrevGmParams[NUM_REF_FRAMES][6];

    std::vector<uint32_t> MiColStarts;
    std::vector<uint32_t> MiRowStarts;
    std::vector<std::vector<PREDICTION_MODE>> YModes;
    std::vector<std::vector<UV_PREDICTION_MODE>> UVModes;
    std::vector<std::vector<std::vector<int>>> RefFrames;
    std::vector<std::vector<TX_TYPE>> TxTypes;
    std::vector<std::vector<bool>> IsInters;
    std::vector<std::vector<bool>> SkipModes;
    std::vector<std::vector<bool>> Skips;
    std::vector<std::vector<TX_SIZE>> InterTxSizes;
    std::vector<std::vector<TX_SIZE>> TxSizes;
    std::vector<std::vector<BLOCK_SIZE>> MiSizes;
    std::vector<std::vector<TX_SIZE>> LoopfilterTxSizes[MAX_PLANES];
    std::vector<std::vector<uint8_t>> SegmentIds;
    std::vector<std::vector<int8_t>> DeltaLFs[FRAME_LF_COUNT];
    std::vector<std::vector<int8_t>> MfRefFrames;
    std::vector<std::vector<Mv>> MfMvs;
    std::vector<std::vector<std::vector<Mv>>> MotionFieldMvs;
    std::vector<std::vector<std::vector<Mv>>> Mvs;
    std::vector<std::vector<uint8_t>> CompGroupIdxs;
    std::vector<std::vector<uint8_t>> CompoundIdxs;
    std::vector<std::vector<std::vector<InterpFilter>>> InterpFilters;
    std::shared_ptr<Cdfs> m_cdfs;
    std::vector<std::vector<uint8_t>> PrevSegmentIds;
    std::vector<std::vector<uint8_t>> PaletteSizes[2];
    std::vector<std::vector<std::vector<uint8_t>>> PaletteColors[2];

    Quantization m_quant;
    Segmentation m_segmentation;
    DeltaQ m_deltaQ;
    DeltaLf m_deltaLf;
    LoopFilterParams m_loopFilter;
    CdefParams m_cdef;
    LoopRestorationpParams m_loopRestoration;
    ConstSequencePtr m_sequence;
    RefInfo m_refInfo; //a copy of parsers refInfo

    const static uint8_t SUPERRES_NUM = 8;

    FrameHeader(ConstSequencePtr&);
    bool parse(BitReader& br, RefInfo&);
    int16_t get_qindex(int16_t CurrentQIndex, int segmentId) const;
    int16_t get_qindex(int segmentId) const;

    void referenceFrameLoading();
    void motionVectorStorage();

private:
    void setup_past_independence();
    void load_cdfs();
    int8_t get_relative_dist(uint8_t a, uint8_t b) const;
    int8_t get_relative_dist(uint8_t ref) const;
    void mark_ref_frames(uint8_t idLen, RefInfo& refInfo);
    void set_frame_refs(const RefInfo& refInfo);
    bool frame_size(BitReader& br);
    bool superres_params(BitReader& br);
    bool render_size(BitReader& br);
    void computeAlignedSize();
    void compute_image_size();
    bool frame_size_with_refs(BitReader& br, const RefInfo& refInfo);
    bool read_interpolation_filter(BitReader& br);
    bool parseTileInfo(BitReader& br);
    bool parseTileStarts(BitReader& br, std::vector<uint32_t>& starts, uint32_t sbMax, uint32_t sbShift, uint32_t maxTileSb);
    bool parseQuantizationParams(BitReader& br);
    bool loop_filter_params(BitReader& br);
    bool cdef_params(BitReader& br);
    bool lr_params(BitReader& br);
    bool read_tx_mode(BitReader& br);
    bool frame_reference_mode(BitReader& br);
    bool skip_mode_params(BitReader& br, const RefInfo& refInfo);
    void initGeometry();

    //for inter predict
    void motion_field_estimation(const RefInfo& refInfo);
    bool get_block_position(int& PosX8, int& PosY8, int x8, int y8, int dstSign, const Mv& projMv);
    bool mvProject(const RefInfo& refInfo, uint8_t src, int dstSign);
    Mv get_mv_projection(const Mv& mv, int numerator, int denominator);

    bool is_scaled(int refFrame) const;
    void getScale(uint8_t refIdx, uint32_t& xScale, uint32_t& yScale) const;

    bool read_global_param(BitReader& br, GlobalMotionType type, uint8_t ref, int idx);

    bool global_motion_params(BitReader& br);
    void load_cdfs(const RefInfo& refInfo);
    void load_previous(const RefInfo& refInfo);
    void load_previous_segment_ids(const RefInfo& refInfo);

    int16_t get_qindex(bool ignoreDeltaQ, int16_t CurrentQIndex, int segmentId) const;


    const static uint8_t SUPERRES_DENOM_MIN = 9;

    const static uint8_t SUPERRES_DENOM_BITS = 3;
    const static uint8_t TX_MODES = 3;
};

class Parser {
public:
    Parser();
    bool parseSequenceHeader(BitReader& br);
    bool parseTemporalDelimiter(BitReader& br);
    std::shared_ptr<FrameHeader> parseFrameHeader(BitReader& br);
    bool parseTileGroup(BitReader& br, const FramePtr& frame, TileGroup& group);
    bool parseMetadata(BitReader& br);
    bool parsePadding(BitReader& br);
    FramePtr parseFrame(BitReader& br, TileGroup& group);
    bool praseReserved(BitReader& br);
    void finishFrame();
    std::shared_ptr<SequenceHeader> m_sequence;
    std::shared_ptr<FrameHeader> m_frame;

private:
    RefInfo m_refInfo;
    void skipTrailingBits(BitReader& br);
    bool m_seenFrameHeader;
};

const static int FILTER_BITS = 7;

inline static void roundingVariablesDerivation(
    bool isCompound, int BitDepth,
    int& InterRound0, int& InterRound1, int& InterPostRound)
{
    InterRound0 = 3;
    InterRound1 = isCompound ? 7 : 11;
    if (BitDepth == 12) {
        InterRound0 += 2;
        if (!isCompound) {
            InterRound1 -= 2;
        }
    }
    InterPostRound = 2 * FILTER_BITS - (InterRound0 + InterRound1);
}
}
