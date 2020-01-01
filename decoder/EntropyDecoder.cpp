#include "EntropyDecoder.h"
#include "../aom/entropymode.c"
#include "../aom/token_cdfs.h"
#include "../common/log.h"
#include <iostream>
#include <string.h>

namespace YamiAv1 {

using namespace Yami;

#define av1_copy(a, b) memcpy(a, b, sizeof(b))

EntropyDecoder::EntropyDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update, uint32_t baseQ)
{
    m_symbol.reset(new SymbolDecoder(data, sz, disable_cdf_update));
    av1_copy(palette_y_size_cdf, default_palette_y_size_cdf);
    av1_copy(palette_uv_size_cdf, default_palette_uv_size_cdf);
    av1_copy(palette_y_color_index_cdf, default_palette_y_color_index_cdf);
    av1_copy(palette_uv_color_index_cdf, default_palette_uv_color_index_cdf);
    av1_copy(kf_y_cdf, default_kf_y_mode_cdf);
    av1_copy(angle_delta_cdf, default_angle_delta_cdf);
    av1_copy(comp_inter_cdf, default_comp_inter_cdf);
    av1_copy(comp_ref_type_cdf, default_comp_ref_type_cdf);
    av1_copy(uni_comp_ref_cdf, default_uni_comp_ref_cdf);
    av1_copy(palette_y_mode_cdf, default_palette_y_mode_cdf);
    av1_copy(palette_uv_mode_cdf, default_palette_uv_mode_cdf);
    av1_copy(comp_ref_cdf, default_comp_ref_cdf);
    av1_copy(comp_bwdref_cdf, default_comp_bwdref_cdf);
    av1_copy(single_ref_cdf, default_single_ref_cdf);
    av1_copy(txfm_partition_cdf, default_txfm_partition_cdf);
    av1_copy(compound_index_cdf, default_compound_idx_cdfs);
    av1_copy(comp_group_idx_cdf, default_comp_group_idx_cdfs);
    av1_copy(newmv_cdf, default_newmv_cdf);
    av1_copy(zeromv_cdf, default_zeromv_cdf);
    av1_copy(refmv_cdf, default_refmv_cdf);
    av1_copy(drl_cdf, default_drl_cdf);
    av1_copy(motion_mode_cdf, default_motion_mode_cdf);
    av1_copy(obmc_cdf, default_obmc_cdf);
    av1_copy(inter_compound_mode_cdf, default_inter_compound_mode_cdf);
    av1_copy(compound_type_cdf, default_compound_type_cdf);
    av1_copy(wedge_idx_cdf, default_wedge_idx_cdf);
    av1_copy(interintra_cdf, default_interintra_cdf);
    av1_copy(wedge_interintra_cdf, default_wedge_interintra_cdf);
    av1_copy(interintra_mode_cdf, default_interintra_mode_cdf);
    //av1_copy(seg.pred_cdf, default_segment_pred_cdf);
    //av1_copy(seg.tree_cdf, default_seg_tree_cdf);
    av1_copy(filter_intra_cdfs, default_filter_intra_cdfs);
    av1_copy(filter_intra_mode_cdf, default_filter_intra_mode_cdf);
    av1_copy(switchable_restore_cdf, default_switchable_restore_cdf);
    av1_copy(wiener_restore_cdf, default_wiener_restore_cdf);
    av1_copy(sgrproj_restore_cdf, default_sgrproj_restore_cdf);
    av1_copy(y_mode_cdf, default_if_y_mode_cdf);
    av1_copy(uv_mode_cdf, default_uv_mode_cdf);
    av1_copy(switchable_interp_cdf, default_switchable_interp_cdf);
    av1_copy(partition_cdf, default_partition_cdf);
    av1_copy(intra_ext_tx_cdf, default_intra_ext_tx_cdf);
    av1_copy(inter_ext_tx_cdf, default_inter_ext_tx_cdf);
    av1_copy(skip_mode_cdfs, default_skip_mode_cdfs);
    av1_copy(skip_cdfs, default_skip_cdfs);
    av1_copy(intra_inter_cdf, default_intra_inter_cdf);
    //for (int i = 0; i < SPATIAL_PREDICTION_PROBS; i++)
    //av1_copy(seg.spatial_pred_seg_cdf[i],
    //default_spatial_pred_seg_tree_cdf[i]);
    av1_copy(tx_size_cdf, default_tx_size_cdf);
    av1_copy(delta_q_cdf, default_delta_q_cdf);
    av1_copy(delta_lf_cdf, default_delta_lf_cdf);
    av1_copy(delta_lf_multi_cdf, default_delta_lf_multi_cdf);
    av1_copy(cfl_sign_cdf, default_cfl_sign_cdf);
    av1_copy(cfl_alpha_cdf, default_cfl_alpha_cdf);
    av1_copy(intrabc_cdf, default_intrabc_cdf);
    memcpy(&nmv_context[0], &default_nmv_context, sizeof(default_nmv_context));
    memcpy(&nmv_context[1], &default_nmv_context, sizeof(default_nmv_context));

    initCoefCdf(baseQ);
}

static uint8_t getQctx(uint32_t q)
{
    if (q <= 20)
        return 0;
    if (q <= 60)
        return 1;
    if (q <= 120)
        return 2;
    return 3;
}

void EntropyDecoder::initCoefCdf(uint32_t baseQ)
{
    const uint8_t index = getQctx(baseQ);

    av1_copy(txb_skip_cdf, av1_default_txb_skip_cdfs[index]);
    av1_copy(eob_extra_cdf, av1_default_eob_extra_cdfs[index]);
    av1_copy(dc_sign_cdf, av1_default_dc_sign_cdfs[index]);
    av1_copy(coeff_br_cdf, av1_default_coeff_lps_multi_cdfs[index]);
    av1_copy(coeff_base_cdf, av1_default_coeff_base_multi_cdfs[index]);
    av1_copy(coeff_base_eob_cdf,
        av1_default_coeff_base_eob_multi_cdfs[index]);
    av1_copy(eob_flag_cdf16, av1_default_eob_multi16_cdfs[index]);
    av1_copy(eob_flag_cdf32, av1_default_eob_multi32_cdfs[index]);
    av1_copy(eob_flag_cdf64, av1_default_eob_multi64_cdfs[index]);
    av1_copy(eob_flag_cdf128, av1_default_eob_multi128_cdfs[index]);
    av1_copy(eob_flag_cdf256, av1_default_eob_multi256_cdfs[index]);
    av1_copy(eob_flag_cdf512, av1_default_eob_multi512_cdfs[index]);
    av1_copy(eob_flag_cdf1024, av1_default_eob_multi1024_cdfs[index]);
}

EntropyDecoder::~EntropyDecoder()
{
}

PREDICTION_MODE EntropyDecoder::readIntraFrameYMode(uint8_t aboveCtx, uint8_t leftCtx)
{
    return (PREDICTION_MODE)m_symbol->read(kf_y_cdf[aboveCtx][leftCtx], INTRA_MODES);
}

bool EntropyDecoder::readSkip(uint8_t ctx)
{
    return (bool)m_symbol->read(skip_cdfs[ctx], 2);
}

uint8_t EntropyDecoder::getPartitionCdfCount(uint8_t bsl)
{
    if (bsl == 1)
        return PARTITION_TYPES;
    if (bsl == 5)
        return EXT_PARTITION_TYPES - 2;
    //ASSERT(bsl > 1 && bsl < 5);
    return EXT_PARTITION_TYPES;
}

PARTITION_TYPE EntropyDecoder::readPartition(uint8_t ctx, uint8_t bsl)
{
    uint8_t idx = (bsl - 1) * 4 + ctx;
    return (PARTITION_TYPE)m_symbol->read(partition_cdf[idx], getPartitionCdfCount(bsl));
}

bool EntropyDecoder::readSplitOrHorz(uint8_t ctx, uint8_t bsl, BLOCK_SIZE bSize)
{
    uint8_t idx = (bsl - 1) * 4 + ctx;
    aom_cdf_prob* pcdf = partition_cdf[idx];
    int psum = (pcdf[PARTITION_VERT] - pcdf[PARTITION_VERT - 1] + pcdf[PARTITION_SPLIT] - pcdf[PARTITION_SPLIT - 1] + pcdf[PARTITION_HORZ_A] - pcdf[PARTITION_HORZ_A - 1] + pcdf[PARTITION_VERT_A] - pcdf[PARTITION_VERT_A - 1] + pcdf[PARTITION_VERT_B] - pcdf[PARTITION_VERT_B - 1]);
    if (bSize != BLOCK_128X128)
        psum += pcdf[PARTITION_VERT_4] - pcdf[PARTITION_VERT_4 - 1];
    //we use reverted cdf so it we need + psum instead - psum
    uint16_t icdf[] = { AOM_CDF2(CDF_PROB_TOP + psum) };
    return (bool)m_symbol->read(icdf, 2, true);
}

bool EntropyDecoder::readSplitOrVert(uint8_t ctx, uint8_t bsl, BLOCK_SIZE bSize)
{
    uint8_t idx = (bsl - 1) * 4 + ctx;
    aom_cdf_prob* pcdf = partition_cdf[idx];
    int psum = (pcdf[PARTITION_HORZ] - pcdf[PARTITION_HORZ - 1] + pcdf[PARTITION_SPLIT] - pcdf[PARTITION_SPLIT - 1] + pcdf[PARTITION_HORZ_A] - pcdf[PARTITION_HORZ_A - 1] + pcdf[PARTITION_HORZ_B] - pcdf[PARTITION_HORZ_B - 1] + pcdf[PARTITION_VERT_A] - pcdf[PARTITION_VERT_A - 1]);
    if (bSize != BLOCK_128X128)
        psum += pcdf[PARTITION_HORZ_4] - pcdf[PARTITION_HORZ_4 - 1];
    //we use reverted cdf so it we need + psum instead - psum
    uint16_t icdf[] = { AOM_CDF2(CDF_PROB_TOP + psum) };
    return (bool)m_symbol->read(icdf, 2, true);
}

UV_PREDICTION_MODE EntropyDecoder::readUvMode(CFL_ALLOWED_TYPE cflAllowed, PREDICTION_MODE yMode)
{
    return (UV_PREDICTION_MODE)m_symbol->read(uv_mode_cdf[cflAllowed][yMode], UV_INTRA_MODES - !cflAllowed);
}

int8_t EntropyDecoder::readAngleDeltaY(PREDICTION_MODE YMode)
{
    uint8_t angle_delta_y = (uint8_t)m_symbol->read(angle_delta_cdf[YMode - V_PRED], 2 * MAX_ANGLE_DELTA + 1);
    return angle_delta_y - MAX_ANGLE_DELTA;
}

int8_t EntropyDecoder::readAngleDeltaUV(UV_PREDICTION_MODE uvMode)
{
    uint8_t angle_delta_uv = (uint8_t)m_symbol->read(angle_delta_cdf[uvMode - UV_V_PRED], 2 * MAX_ANGLE_DELTA + 1);
    return angle_delta_uv - MAX_ANGLE_DELTA;
}

uint8_t EntropyDecoder::readCflAlphaSigns()
{
    return (uint8_t)m_symbol->read(cfl_sign_cdf, CFL_JOINT_SIGNS);
}

int8_t EntropyDecoder::readCflAlphaU(uint8_t cfl_alpha_signs)
{
    int ctx = cfl_alpha_signs - 2;
    return (int8_t)m_symbol->read(cfl_alpha_cdf[ctx], CFL_ALPHABET_SIZE) + 1;
}
int8_t EntropyDecoder::readCflAlphaV(uint8_t cfl_alpha_signs)
{
    int contexts[] = { 0, 3, 0, 1, 4, 0, 2, 5 };
    int ctx = contexts[cfl_alpha_signs];
    return (int8_t)m_symbol->read(cfl_alpha_cdf[ctx], CFL_ALPHABET_SIZE) + 1;
}

bool EntropyDecoder::readUseFilterIntra(BLOCK_SIZE bSize)
{
    return m_symbol->read(filter_intra_cdfs[bSize], 2);
}

FILTER_INTRA_MODE EntropyDecoder::readFilterIntraMode()
{
    return (FILTER_INTRA_MODE)m_symbol->read(filter_intra_mode_cdf, FILTER_INTRA_MODES);
}

uint8_t EntropyDecoder::readTxDepth(int maxTxDepth, uint8_t ctx)
{
    static const int MaxDepthToCat[] = { 0, 0, 1, 2, 3 };
    static const int Sizes[] = { 2, 2, 3, 3, 3 };
    int cat = MaxDepthToCat[maxTxDepth];
    int size = Sizes[maxTxDepth];
    return (uint8_t)m_symbol->read(tx_size_cdf[cat][ctx], size);
}
bool EntropyDecoder::readTxfmSplit(uint8_t ctx)
{
    return (bool)m_symbol->read(txfm_partition_cdf[ctx], 2);
}
uint8_t EntropyDecoder::readInterTxType(TxSet set, TX_SIZE txSzSqr)
{
    ASSERT(set == TX_SET_INTER_1 || set == TX_SET_INTER_2 || set == TX_SET_INTER_3);
    const int cdfSize[] = { 0, 16, 12, 2 };
    return (uint8_t)m_symbol->read(inter_ext_tx_cdf[set][txSzSqr], cdfSize[set]);
}
uint8_t EntropyDecoder::readIntraTxType(TxSet set, TX_SIZE txSzSqr, PREDICTION_MODE intraDir)
{
    ASSERT(set == TX_SET_INTRA_1 || set == TX_SET_INTRA_2);
    if (set == TX_SET_INTRA_1)
        return (uint8_t)m_symbol->read(intra_ext_tx_cdf[1][txSzSqr][intraDir], 7);
    return (uint8_t)m_symbol->read(intra_ext_tx_cdf[2][txSzSqr][intraDir], 5);
}

bool EntropyDecoder::readAllZero(uint8_t txSzCtx, uint8_t ctx)
{
    return (bool)m_symbol->read(txb_skip_cdf[txSzCtx][ctx], 2);
}

uint8_t EntropyDecoder::readEobPt(uint8_t eobMultisize, PLANE_TYPE planeType, uint8_t ctx)
{
    aom_cdf_prob* cdf;
    if (eobMultisize == 0) {
        cdf = eob_flag_cdf16[planeType][ctx];
    } else if (eobMultisize == 1) {
        cdf = eob_flag_cdf32[planeType][ctx];
    } else if (eobMultisize == 2) {
        cdf = eob_flag_cdf64[planeType][ctx];
    } else if (eobMultisize == 3) {
        cdf = eob_flag_cdf128[planeType][ctx];
    } else if (eobMultisize == 4) {
        cdf = eob_flag_cdf256[planeType][ctx];
    } else if (eobMultisize == 5) {
        cdf = eob_flag_cdf512[planeType][ctx];
    } else if (eobMultisize == 6) {
        cdf = eob_flag_cdf1024[planeType][ctx];
    } else {
        return 0;
    }
    return m_symbol->read(cdf, eobMultisize + 5) + 1;
}
bool EntropyDecoder::readEobExtra(TX_SIZE txSzCtx, PLANE_TYPE ptype, int eobPt)
{
    return (bool)m_symbol->read(eob_extra_cdf[txSzCtx][ptype][eobPt - 3], 2);
}

uint8_t EntropyDecoder::readCoeffBaseEob(uint8_t txSzCtx, PLANE_TYPE planeType, uint8_t ctx)
{
    return m_symbol->read(coeff_base_eob_cdf[txSzCtx][planeType][ctx], 3);
}

uint8_t EntropyDecoder::readCoeffBase(uint8_t txSzCtx, PLANE_TYPE planeType, uint8_t ctx)
{
    return m_symbol->read(coeff_base_cdf[txSzCtx][planeType][ctx], 4);
}

uint8_t EntropyDecoder::readCoeffBr(uint8_t minTx, PLANE_TYPE planeType, uint8_t ctx)
{
    return m_symbol->read(coeff_br_cdf[minTx][planeType][ctx], BR_CDF_SIZE);
}

bool EntropyDecoder::readDcSign(PLANE_TYPE planeType, uint8_t ctx)
{
    return (bool)m_symbol->read(dc_sign_cdf[planeType][ctx], 2);
}

bool EntropyDecoder::readUseWiener()
{
    return (bool)m_symbol->read(wiener_restore_cdf, 2);
}

bool EntropyDecoder::readUseSgrproj()
{
    return (bool)m_symbol->read(sgrproj_restore_cdf, 2);
}

RestorationType EntropyDecoder::readRestorationType()
{
    return (RestorationType)m_symbol->read(switchable_restore_cdf, RESTORE_SWITCHABLE_TYPES);
}

uint8_t EntropyDecoder::readLrSgrSet()
{
    return readLiteral(SGRPROJ_PARAMS_BITS);
}

bool EntropyDecoder::readIsInter(uint8_t ctx)
{
    return (bool)m_symbol->read(intra_inter_cdf[ctx], 2);
}

const static uint8_t Size_Group[BLOCK_SIZES_ALL] = {
    0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3,
    3, 3, 3, 3, 3, 0, 0, 1, 1, 2, 2
};

PREDICTION_MODE EntropyDecoder::readYMode(BLOCK_SIZE MiSize)
{
    uint8_t ctx = Size_Group[MiSize];
    return (PREDICTION_MODE)m_symbol->read(y_mode_cdf[ctx], INTRA_MODES);
}

CompMode EntropyDecoder::readCompMode(uint8_t ctx)
{
    return (CompMode)m_symbol->read(comp_inter_cdf[ctx], 2);
}

COMP_REFERENCE_TYPE EntropyDecoder::readCompReferenceType(uint8_t ctx)
{
    return (COMP_REFERENCE_TYPE)m_symbol->read(comp_ref_type_cdf[ctx], 2);
}

bool EntropyDecoder::readUniCompRef(uint8_t ctx)
{
    return (bool)m_symbol->read(uni_comp_ref_cdf[ctx][0], 2);
}

bool EntropyDecoder::readUniCompRefP1(uint8_t ctx)
{
    return (bool)m_symbol->read(uni_comp_ref_cdf[ctx][1], 2);
}

bool EntropyDecoder::readUniCompRefP2(uint8_t ctx)
{
    return (bool)m_symbol->read(uni_comp_ref_cdf[ctx][2], 2);
}

bool EntropyDecoder::readCompRef(uint8_t ctx)
{
    return (bool)m_symbol->read(comp_ref_cdf[ctx][0], 2);
}

bool EntropyDecoder::readCompRefP1(uint8_t ctx)
{
    return (bool)m_symbol->read(comp_ref_cdf[ctx][1], 2);
}

bool EntropyDecoder::readCompRefP2(uint8_t ctx)
{
    return (bool)m_symbol->read(comp_ref_cdf[ctx][2], 2);
}

bool EntropyDecoder::readCompBwdRef(uint8_t ctx)
{
    return (bool)m_symbol->read(comp_bwdref_cdf[ctx][0], 2);
}

bool EntropyDecoder::readCompBwdRefP1(uint8_t ctx)
{
    return (bool)m_symbol->read(comp_bwdref_cdf[ctx][1], 2);
}

bool EntropyDecoder::readSingleRef(uint8_t ctx, int n)
{
    return (bool)m_symbol->read(single_ref_cdf[ctx][n - 1], 2);
}

uint8_t EntropyDecoder::readCompoundMode(uint8_t ctx)
{
    return (uint8_t)m_symbol->read(inter_compound_mode_cdf[ctx], INTER_COMPOUND_MODES);
}

bool EntropyDecoder::readNewMv(uint8_t ctx)
{
    return (bool)m_symbol->read(newmv_cdf[ctx], 2);
}

bool EntropyDecoder::readZeroMv(uint8_t ctx)
{
    return (bool)m_symbol->read(zeromv_cdf[ctx], 2);
}

bool EntropyDecoder::readRefMv(uint8_t ctx)
{
    return (bool)m_symbol->read(refmv_cdf[ctx], 2);
}

bool EntropyDecoder::readDrlMode(uint8_t ctx)
{
    return (bool)m_symbol->read(drl_cdf[ctx], 2);
}

MV_JOINT_TYPE EntropyDecoder::readMvJoint(uint8_t ctx)
{
    return (MV_JOINT_TYPE)m_symbol->read(nmv_context[ctx].joints_cdf, MV_JOINTS);
}

bool EntropyDecoder::readMvSign(uint8_t ctx, uint8_t comp)
{
    return (bool)m_symbol->read(nmv_context[ctx].comps[comp].sign_cdf, 2);
}

MV_CLASS_TYPE EntropyDecoder::readMvClass(uint8_t ctx, uint8_t comp)
{
    return (MV_CLASS_TYPE)m_symbol->read(nmv_context[ctx].comps[comp].classes_cdf, MV_CLASSES);
}

int EntropyDecoder::readMvClass0Bit(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(nmv_context[ctx].comps[comp].class0_cdf, 2);
}

int EntropyDecoder::readMvBit(int i, uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(nmv_context[ctx].comps[comp].bits_cdf[i], 2);
}

int EntropyDecoder::readMvClass0Fr(int mv_class0_bit, uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(nmv_context[ctx].comps[comp].class0_fp_cdf[mv_class0_bit], MV_FP_SIZE);
}

int EntropyDecoder::readMvClass0Hp(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(nmv_context[ctx].comps[comp].class0_hp_cdf, 2);
}

int EntropyDecoder::readMvFr(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(nmv_context[ctx].comps[comp].fp_cdf, MV_FP_SIZE);
}

int EntropyDecoder::readMvHp(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(nmv_context[ctx].comps[comp].hp_cdf, 2);
}

bool EntropyDecoder::readInterIntra(BLOCK_SIZE MiSize)
{
    uint8_t ctx = Size_Group[MiSize] - 1;
    return (bool)m_symbol->read(interintra_cdf[ctx], 2);
}

INTERINTRA_MODE EntropyDecoder::readInterIntraMode(BLOCK_SIZE MiSize)
{
    uint8_t ctx = Size_Group[MiSize] - 1;
    return (INTERINTRA_MODE)m_symbol->read(interintra_mode_cdf[ctx], INTERINTRA_MODES);
}

bool EntropyDecoder::readWedgeInterIntra(BLOCK_SIZE MiSize)
{
    return (bool)m_symbol->read(wedge_interintra_cdf[MiSize], 2);
}

uint8_t EntropyDecoder::readWedgeIndex(BLOCK_SIZE MiSize)
{
    return (uint8_t)m_symbol->read(wedge_idx_cdf[MiSize], 16);
}

bool EntropyDecoder::readCompGroupIdx(uint8_t ctx)
{
    return (bool)m_symbol->read(comp_group_idx_cdf[ctx], 2);
}

bool EntropyDecoder::readCompoundIdx(uint8_t ctx)
{
    return (bool)m_symbol->read(compound_index_cdf[ctx], 2);
}

COMPOUND_TYPE EntropyDecoder::readCompoundType(BLOCK_SIZE MiSize)
{
    return (COMPOUND_TYPE)m_symbol->read(compound_type_cdf[MiSize], COMPOUND_TYPES);
}

bool EntropyDecoder::readWedgeSign()
{
    return (bool)m_symbol->readBool();
}

bool EntropyDecoder::readMaskType()
{
    return (bool)m_symbol->readBool();
}

InterpFilter EntropyDecoder::readInterpFilter(uint8_t ctx)
{
    return (InterpFilter)m_symbol->read(switchable_interp_cdf[ctx], SWITCHABLE_FILTERS);
}

bool EntropyDecoder::readUseObmc(BLOCK_SIZE MiSize)
{
    return (bool)m_symbol->read(obmc_cdf[MiSize], 2);
}

MOTION_MODE EntropyDecoder::readMotionMode(BLOCK_SIZE MiSize)
{
    return (MOTION_MODE)m_symbol->read(motion_mode_cdf[MiSize], MOTION_MODES);
}

bool EntropyDecoder::readUe(uint32_t& v)
{
    uint8_t len = 0;
    uint8_t bit = 0;
    do {
        len++;
        if (len >= 32) {
            ERROR("invalid lenth");
            return false;
        }
        bit = m_symbol->readBool();
    } while (!bit);
    std::cout << len;
    v = 1;
    for (int i = len - 2; i >= 0; i--) {
        bit = m_symbol->readBool();
        v = (v << 1) | bit;
    }
    return true;
}

uint32_t EntropyDecoder::readLiteral(uint32_t n)
{
    uint32_t x = 0;
    for (uint32_t i = 0; i < n; i++) {
        x = 2 * x + m_symbol->readBool();
    }
    return x;
}

static int inverse_recenter(int r, int v)
{
    if (v > 2 * r)
        return v;
    else if (v & 1)
        return r - ((v + 1) >> 1);
    else
        return r + (v >> 1);
}
int EntropyDecoder::decode_signed_subexp_with_ref_bool(int low, int high, int k, int r)
{
    int x = decode_unsigned_subexp_with_ref_bool(high - low, k, r - low);
    return x + low;
}

int EntropyDecoder::decode_unsigned_subexp_with_ref_bool(int mx, int k, int r)
{
    int v = decode_subexp_bool(mx, k);
    if ((r << 1) <= mx) {
        return inverse_recenter(r, v);
    } else {
        return mx - 1 - inverse_recenter(mx - 1 - r, v);
    }
}

int EntropyDecoder::decode_subexp_bool(int numSyms, int k)
{
    int i = 0;
    int mk = 0;
    while (1) {
        int b2 = i ? k + i - 1 : k;
        int a = 1 << b2;
        if (numSyms <= mk + 3 * a) {
            uint32_t subexp_unif_bools;
            subexp_unif_bools = readNS(numSyms - mk);
            return subexp_unif_bools + mk;
        } else {
            uint32_t subexp_more_bools = readLiteral(1);
            if (subexp_more_bools) {
                i++;
                mk += a;
            } else {
                uint32_t subexp_bools = readLiteral(b2);
                return subexp_bools + mk;
            }
        }
    }
}

uint32_t EntropyDecoder::readNS(int n)
{
    int w = FloorLog2(n) + 1;
    int m = (1 << w) - n;
    uint32_t v = readLiteral(w - 1);
    if (v < m)
        return v;
    return (v << 1) - m + readLiteral(1);
}

}