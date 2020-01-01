#pragma once

#include "../aom/entropy.h"
#include "../aom/entropymode.h"
#include "../aom/enums.h"
#include "../aom/filter.h"
#include "../aom/prob.h"
#include "../aom/seg_common.h"
#include "SymbolDecoder.h"
#include "Av1Common.h"
#include <memory>

namespace YamiAv1 {

class EntropyDecoder {
public:
    EntropyDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update, uint32_t baseQ);
    ~EntropyDecoder();
    PREDICTION_MODE readIntraFrameYMode(uint8_t aboveCtx, uint8_t leftCtx);
    bool readSkip(uint8_t ctx);
    PARTITION_TYPE readPartition(uint8_t ctx, uint8_t bsl);
    bool readSplitOrHorz(uint8_t ctx, uint8_t bsl, BLOCK_SIZE bSize);
    bool readSplitOrVert(uint8_t ctx, uint8_t bsl, BLOCK_SIZE bSize);
    UV_PREDICTION_MODE readUvMode(CFL_ALLOWED_TYPE cfl_allowed, PREDICTION_MODE y_mode);
    int8_t readAngleDeltaY(PREDICTION_MODE YMode);
    int8_t readAngleDeltaUV(UV_PREDICTION_MODE uvMode);
    uint8_t readCflAlphaSigns();
    int8_t readCflAlphaU(uint8_t cfl_alpha_signs);
    int8_t readCflAlphaV(uint8_t cfl_alpha_signs);
    bool readUseFilterIntra(BLOCK_SIZE bSize);
    FILTER_INTRA_MODE readFilterIntraMode();
    uint8_t readTxDepth(int maxTxDepth, uint8_t ctx);
    bool readTxfmSplit(uint8_t ctx);
    uint8_t readInterTxType(TxSet set, TX_SIZE txSzSqr);
    uint8_t readIntraTxType(TxSet set, TX_SIZE txSzSqr, PREDICTION_MODE intraDir);
    bool readAllZero(uint8_t txSzCtx, uint8_t ctx);
    uint8_t readEobPt(uint8_t eobMultisize, PLANE_TYPE planeType, uint8_t ctx);
    bool readEobExtra(TX_SIZE txSzCtx, PLANE_TYPE ptype, int eobPt);
    uint8_t readCoeffBaseEob(uint8_t txSzCtx, PLANE_TYPE planeType, uint8_t ctx);
    uint8_t readCoeffBase(uint8_t txSzCtx, PLANE_TYPE planeType, uint8_t ctx);
    uint8_t readCoeffBr(uint8_t minTx, PLANE_TYPE planeType, uint8_t ctx);
    bool readDcSign(PLANE_TYPE planeType, uint8_t ctx);
    bool readUseWiener();
    bool readUseSgrproj();
    uint8_t readLrSgrSet();
    RestorationType readRestorationType();
    //for inter
    bool readIsInter(uint8_t ctx);
    PREDICTION_MODE readYMode(BLOCK_SIZE MiSize);
    CompMode readCompMode(uint8_t ctx);
    COMP_REFERENCE_TYPE readCompReferenceType(uint8_t ctx);

    bool readUniCompRef(uint8_t ctx);
    bool readUniCompRefP1(uint8_t ctx);
    bool readUniCompRefP2(uint8_t ctx);

    bool readCompRef(uint8_t ctx);
    bool readCompRefP1(uint8_t ctx);
    bool readCompRefP2(uint8_t ctx);
    bool readCompBwdRef(uint8_t ctx);
    bool readCompBwdRefP1(uint8_t ctx);

    bool readSingleRef(uint8_t ctx, int n);

    uint8_t readCompoundMode(uint8_t ctx);
    bool readNewMv(uint8_t ctx);
    bool readZeroMv(uint8_t ctx);
    bool readRefMv(uint8_t ctx);
    bool readDrlMode(uint8_t ctx);

    MV_JOINT_TYPE readMvJoint(uint8_t ctx);
    bool readMvSign(uint8_t ctx, uint8_t comp);
    MV_CLASS_TYPE readMvClass(uint8_t ctx, uint8_t comp);
    int readMvClass0Bit(uint8_t ctx, uint8_t comp);
    int readMvBit(int i, uint8_t ctx, uint8_t comp);
    int readMvClass0Fr(int mv_class0_bit, uint8_t ctx, uint8_t comp);
    int readMvClass0Hp(uint8_t ctx, uint8_t comp);
    int readMvFr(uint8_t ctx, uint8_t comp);
    int readMvHp(uint8_t ctx, uint8_t comp);

    bool readInterIntra(BLOCK_SIZE MiSize);
    INTERINTRA_MODE readInterIntraMode(BLOCK_SIZE MiSize);
    bool readWedgeInterIntra(BLOCK_SIZE MiSize);
    uint8_t readWedgeIndex(BLOCK_SIZE MiSize);

    bool readCompGroupIdx(uint8_t ctx);

    bool readCompoundIdx(uint8_t ctx);

    COMPOUND_TYPE readCompoundType(BLOCK_SIZE MiSize);

    bool readWedgeSign();

    bool readMaskType();

    InterpFilter readInterpFilter(uint8_t ctx);
    bool readUseObmc(BLOCK_SIZE MiSize);
    MOTION_MODE readMotionMode(BLOCK_SIZE MiSize);

    bool readUe(uint32_t& v);
    uint32_t readLiteral(uint32_t n);
    int decode_signed_subexp_with_ref_bool(int low, int high, int k, int r);

private:
    void initCoefCdf(uint32_t baseQ);
    uint8_t getPartitionCdfCount(uint8_t bsl);
    int decode_unsigned_subexp_with_ref_bool(int mx, int k, int r);
    int decode_subexp_bool(int numSyms, int k);
    uint32_t readNS(int n);
    std::unique_ptr<SymbolDecoder> m_symbol;

    aom_cdf_prob txb_skip_cdf[TX_SIZES][TXB_SKIP_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob eob_extra_cdf[TX_SIZES][PLANE_TYPES][EOB_COEF_CONTEXTS]
        [CDF_SIZE(2)];
    aom_cdf_prob dc_sign_cdf[PLANE_TYPES][DC_SIGN_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob eob_flag_cdf16[PLANE_TYPES][2][CDF_SIZE(5)];
    aom_cdf_prob eob_flag_cdf32[PLANE_TYPES][2][CDF_SIZE(6)];
    aom_cdf_prob eob_flag_cdf64[PLANE_TYPES][2][CDF_SIZE(7)];
    aom_cdf_prob eob_flag_cdf128[PLANE_TYPES][2][CDF_SIZE(8)];
    aom_cdf_prob eob_flag_cdf256[PLANE_TYPES][2][CDF_SIZE(9)];
    aom_cdf_prob eob_flag_cdf512[PLANE_TYPES][2][CDF_SIZE(10)];
    aom_cdf_prob eob_flag_cdf1024[PLANE_TYPES][2][CDF_SIZE(11)];
    aom_cdf_prob coeff_base_eob_cdf[TX_SIZES][PLANE_TYPES][SIG_COEF_CONTEXTS_EOB]
        [CDF_SIZE(3)];
    aom_cdf_prob coeff_base_cdf[TX_SIZES][PLANE_TYPES][SIG_COEF_CONTEXTS]
        [CDF_SIZE(4)];
    aom_cdf_prob coeff_br_cdf[TX_SIZES][PLANE_TYPES][LEVEL_CONTEXTS]
        [CDF_SIZE(BR_CDF_SIZE)];

    aom_cdf_prob newmv_cdf[NEWMV_MODE_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob zeromv_cdf[GLOBALMV_MODE_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob refmv_cdf[REFMV_MODE_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob drl_cdf[DRL_MODE_CONTEXTS][CDF_SIZE(2)];

    aom_cdf_prob inter_compound_mode_cdf[INTER_MODE_CONTEXTS]
        [CDF_SIZE(INTER_COMPOUND_MODES)];
    aom_cdf_prob compound_type_cdf[BLOCK_SIZES_ALL][CDF_SIZE(COMPOUND_TYPES)];
    aom_cdf_prob wedge_idx_cdf[BLOCK_SIZES_ALL][CDF_SIZE(16)];
    aom_cdf_prob interintra_cdf[BLOCK_SIZE_GROUPS][CDF_SIZE(2)];
    aom_cdf_prob wedge_interintra_cdf[BLOCK_SIZES_ALL][CDF_SIZE(2)];
    aom_cdf_prob interintra_mode_cdf[BLOCK_SIZE_GROUPS]
        [CDF_SIZE(INTERINTRA_MODES)];
    aom_cdf_prob motion_mode_cdf[BLOCK_SIZES_ALL][CDF_SIZE(MOTION_MODES)];
    aom_cdf_prob obmc_cdf[BLOCK_SIZES_ALL][CDF_SIZE(2)];
    aom_cdf_prob palette_y_size_cdf[PALATTE_BSIZE_CTXS][CDF_SIZE(PALETTE_SIZES)];
    aom_cdf_prob palette_uv_size_cdf[PALATTE_BSIZE_CTXS][CDF_SIZE(PALETTE_SIZES)];
    aom_cdf_prob palette_y_color_index_cdf[PALETTE_SIZES]
        [PALETTE_COLOR_INDEX_CONTEXTS]
    [CDF_SIZE(PALETTE_COLORS)];
    aom_cdf_prob palette_uv_color_index_cdf[PALETTE_SIZES]
        [PALETTE_COLOR_INDEX_CONTEXTS]
    [CDF_SIZE(PALETTE_COLORS)];
    aom_cdf_prob palette_y_mode_cdf[PALATTE_BSIZE_CTXS][PALETTE_Y_MODE_CONTEXTS]
        [CDF_SIZE(2)];
    aom_cdf_prob palette_uv_mode_cdf[PALETTE_UV_MODE_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob comp_inter_cdf[COMP_INTER_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob single_ref_cdf[REF_CONTEXTS][SINGLE_REFS - 1][CDF_SIZE(2)];
    aom_cdf_prob comp_ref_type_cdf[COMP_REF_TYPE_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob uni_comp_ref_cdf[UNI_COMP_REF_CONTEXTS][UNIDIR_COMP_REFS - 1]
        [CDF_SIZE(2)];
    aom_cdf_prob comp_ref_cdf[REF_CONTEXTS][FWD_REFS - 1][CDF_SIZE(2)];
    aom_cdf_prob comp_bwdref_cdf[REF_CONTEXTS][BWD_REFS - 1][CDF_SIZE(2)];
    aom_cdf_prob txfm_partition_cdf[TXFM_PARTITION_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob compound_index_cdf[COMP_INDEX_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob comp_group_idx_cdf[COMP_GROUP_IDX_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob skip_mode_cdfs[SKIP_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob skip_cdfs[SKIP_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob intra_inter_cdf[INTRA_INTER_CONTEXTS][CDF_SIZE(2)];
    aom_cdf_prob intrabc_cdf[CDF_SIZE(2)];
    //struct segmentation_probs seg;
    aom_cdf_prob filter_intra_cdfs[BLOCK_SIZES_ALL][CDF_SIZE(2)];
    aom_cdf_prob filter_intra_mode_cdf[CDF_SIZE(FILTER_INTRA_MODES)];
    aom_cdf_prob switchable_restore_cdf[CDF_SIZE(RESTORE_SWITCHABLE_TYPES)];
    aom_cdf_prob wiener_restore_cdf[CDF_SIZE(2)];
    aom_cdf_prob sgrproj_restore_cdf[CDF_SIZE(2)];
    aom_cdf_prob y_mode_cdf[BLOCK_SIZE_GROUPS][CDF_SIZE(INTRA_MODES)];
    aom_cdf_prob uv_mode_cdf[CFL_ALLOWED_TYPES][INTRA_MODES]
        [CDF_SIZE(UV_INTRA_MODES)];
    aom_cdf_prob partition_cdf[PARTITION_CONTEXTS][CDF_SIZE(EXT_PARTITION_TYPES)];
    aom_cdf_prob switchable_interp_cdf[SWITCHABLE_FILTER_CONTEXTS]
        [CDF_SIZE(SWITCHABLE_FILTERS)];
    /* kf_y_cdf is discarded after use, so does not require persistent storage.
    However, we keep it with the other CDFs in this struct since it needs to
    be copied to each tile to support parallelism just like the others.
    */
    aom_cdf_prob kf_y_cdf[KF_MODE_CONTEXTS][KF_MODE_CONTEXTS]
        [CDF_SIZE(INTRA_MODES)];

    aom_cdf_prob angle_delta_cdf[DIRECTIONAL_MODES]
        [CDF_SIZE(2 * MAX_ANGLE_DELTA + 1)];

    aom_cdf_prob tx_size_cdf[MAX_TX_CATS][TX_SIZE_CONTEXTS]
        [CDF_SIZE(MAX_TX_DEPTH + 1)];
    aom_cdf_prob delta_q_cdf[CDF_SIZE(DELTA_Q_PROBS + 1)];
    aom_cdf_prob delta_lf_multi_cdf[FRAME_LF_COUNT][CDF_SIZE(DELTA_LF_PROBS + 1)];
    aom_cdf_prob delta_lf_cdf[CDF_SIZE(DELTA_LF_PROBS + 1)];
    aom_cdf_prob intra_ext_tx_cdf[EXT_TX_SETS_INTRA][EXT_TX_SIZES][INTRA_MODES]
        [CDF_SIZE(TX_TYPES)];
    aom_cdf_prob inter_ext_tx_cdf[EXT_TX_SETS_INTER][EXT_TX_SIZES]
        [CDF_SIZE(TX_TYPES)];
    aom_cdf_prob cfl_sign_cdf[CDF_SIZE(CFL_JOINT_SIGNS)];
    aom_cdf_prob cfl_alpha_cdf[CFL_ALPHA_CONTEXTS][CDF_SIZE(CFL_ALPHABET_SIZE)];

    NmvContext nmv_context[2];
};
}