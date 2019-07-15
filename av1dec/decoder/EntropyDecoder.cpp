#include "EntropyDecoder.h"
#include "../aom/entropymode.c"
#include "../aom/token_cdfs.h"
#include "../common/log.h"
#include <iostream>
#include <string.h>

#define av1_copy(a, b) memcpy(a, b, sizeof(b))

EntropyDecoder::EntropyDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update, uint32_t baseQ)
{
    m_symbol.reset(new YamiParser::Av1::SymbolDecoder(data, sz, disable_cdf_update));
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
    ;
    //ASSERT(bsl > 1 && bsl < 5);
    return EXT_PARTITION_TYPES;
}

PARTITION_TYPE EntropyDecoder::readPartition(uint8_t ctx, uint8_t bsl)
{
    uint8_t idx = (bsl - 1) * 4 + ctx;
    return (PARTITION_TYPE)m_symbol->read(partition_cdf[idx], getPartitionCdfCount(bsl));
}

UV_PREDICTION_MODE EntropyDecoder::readUvMode(CFL_ALLOWED_TYPE cflAllowed, PREDICTION_MODE yMode)
{
    return (UV_PREDICTION_MODE)m_symbol->read(uv_mode_cdf[cflAllowed][yMode], UV_INTRA_MODES - !cflAllowed);
}

uint8_t EntropyDecoder::readAngleDeltaUV(UV_PREDICTION_MODE uvMode)
{
    uint8_t angle_delta_uv = (uint8_t)m_symbol->read(angle_delta_cdf[uvMode - UV_V_PRED], 2 * MAX_ANGLE_DELTA + 1);
    return angle_delta_uv - MAX_ANGLE_DELTA;
}

bool EntropyDecoder::readUseFilterIntra(BLOCK_SIZE bSize)
{
    return m_symbol->read(filter_intra_cdfs[bSize], 2);
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