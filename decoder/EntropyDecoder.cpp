#include "EntropyDecoder.h"
#include "../aom/entropymode.c"
#include "../common/log.h"
#include "prob.h"
#include <iostream>
#include <string.h>

namespace YamiAv1 {

using namespace Yami;
using std::vector;

EntropyDecoder::EntropyDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update, Cdfs& cdfs)
    : m_cdfs(cdfs)
{
    m_symbol.reset(new SymbolDecoder(data, sz, disable_cdf_update));
}

EntropyDecoder::~EntropyDecoder()
{
}

PREDICTION_MODE EntropyDecoder::readIntraFrameYMode(uint8_t aboveCtx, uint8_t leftCtx)
{
    return (PREDICTION_MODE)m_symbol->read(m_cdfs.kf_y_cdf[aboveCtx][leftCtx]);
}

bool EntropyDecoder::readSkip(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.skip_cdfs[ctx]);
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
    return (PARTITION_TYPE)m_symbol->read(m_cdfs.partition_cdf[idx]);
}

bool EntropyDecoder::readSplitOrHorz(uint8_t ctx, uint8_t bsl, BLOCK_SIZE bSize)
{
    uint8_t idx = (bsl - 1) * 4 + ctx;
    const vector<aom_cdf_prob> pcdf = m_cdfs.partition_cdf[idx];
    int psum = (pcdf[PARTITION_VERT] - pcdf[PARTITION_VERT - 1] + pcdf[PARTITION_SPLIT] - pcdf[PARTITION_SPLIT - 1] + pcdf[PARTITION_HORZ_A] - pcdf[PARTITION_HORZ_A - 1] + pcdf[PARTITION_VERT_A] - pcdf[PARTITION_VERT_A - 1] + pcdf[PARTITION_VERT_B] - pcdf[PARTITION_VERT_B - 1]);
    if (bSize != BLOCK_128X128)
        psum += pcdf[PARTITION_VERT_4] - pcdf[PARTITION_VERT_4 - 1];
    //we use reverted cdf so it we need + psum instead - psum
    vector<aom_cdf_prob> icdf = { (aom_cdf_prob)AOM_ICDF(CDF_PROB_TOP + psum), AOM_ICDF(CDF_PROB_TOP), 0 };
    return (bool)m_symbol->read(icdf, true);
}

bool EntropyDecoder::readSplitOrVert(uint8_t ctx, uint8_t bsl, BLOCK_SIZE bSize)
{
    uint8_t idx = (bsl - 1) * 4 + ctx;
    const vector<aom_cdf_prob> pcdf = m_cdfs.partition_cdf[idx];
    int psum = (pcdf[PARTITION_HORZ] - pcdf[PARTITION_HORZ - 1] + pcdf[PARTITION_SPLIT] - pcdf[PARTITION_SPLIT - 1] + pcdf[PARTITION_HORZ_A] - pcdf[PARTITION_HORZ_A - 1] + pcdf[PARTITION_HORZ_B] - pcdf[PARTITION_HORZ_B - 1] + pcdf[PARTITION_VERT_A] - pcdf[PARTITION_VERT_A - 1]);
    if (bSize != BLOCK_128X128)
        psum += pcdf[PARTITION_HORZ_4] - pcdf[PARTITION_HORZ_4 - 1];
    //we use reverted cdf so it we need + psum instead - psum
    vector<aom_cdf_prob> icdf = { (aom_cdf_prob)AOM_ICDF(CDF_PROB_TOP + psum), AOM_ICDF(CDF_PROB_TOP), 0 };
    return (bool)m_symbol->read(icdf, true);
}

UV_PREDICTION_MODE EntropyDecoder::readUvMode(CFL_ALLOWED_TYPE cflAllowed, PREDICTION_MODE yMode)
{
    return (UV_PREDICTION_MODE)m_symbol->read(m_cdfs.uv_mode_cdf[cflAllowed][yMode]);
}

int8_t EntropyDecoder::readAngleDeltaY(PREDICTION_MODE YMode)
{
    uint8_t angle_delta_y = (uint8_t)m_symbol->read(m_cdfs.angle_delta_cdf[YMode - V_PRED]);
    return angle_delta_y - MAX_ANGLE_DELTA;
}

int8_t EntropyDecoder::readAngleDeltaUV(UV_PREDICTION_MODE uvMode)
{
    uint8_t angle_delta_uv = (uint8_t)m_symbol->read(m_cdfs.angle_delta_cdf[uvMode - UV_V_PRED]);
    return angle_delta_uv - MAX_ANGLE_DELTA;
}

uint8_t EntropyDecoder::readCflAlphaSigns()
{
    return (uint8_t)m_symbol->read(m_cdfs.cfl_sign_cdf);
}

int8_t EntropyDecoder::readCflAlphaU(uint8_t cfl_alpha_signs)
{
    int ctx = cfl_alpha_signs - 2;
    return (int8_t)m_symbol->read(m_cdfs.cfl_alpha_cdf[ctx]) + 1;
}
int8_t EntropyDecoder::readCflAlphaV(uint8_t cfl_alpha_signs)
{
    int contexts[] = { 0, 3, 0, 1, 4, 0, 2, 5 };
    int ctx = contexts[cfl_alpha_signs];
    return (int8_t)m_symbol->read(m_cdfs.cfl_alpha_cdf[ctx]) + 1;
}

bool EntropyDecoder::readUseFilterIntra(BLOCK_SIZE bSize)
{
    return m_symbol->read(m_cdfs.filter_intra_cdfs[bSize]);
}

FILTER_INTRA_MODE EntropyDecoder::readFilterIntraMode()
{
    return (FILTER_INTRA_MODE)m_symbol->read(m_cdfs.filter_intra_mode_cdf);
}

uint8_t EntropyDecoder::readTxDepth(int maxTxDepth, uint8_t ctx)
{
    static const int MaxDepthToCat[] = { 0, 0, 1, 2, 3 };
    static const int Sizes[] = { 2, 2, 3, 3, 3 };
    int cat = MaxDepthToCat[maxTxDepth];
    int size = Sizes[maxTxDepth];
    return (uint8_t)m_symbol->read(m_cdfs.tx_size_cdf[cat][ctx]);
}
bool EntropyDecoder::readTxfmSplit(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.txfm_partition_cdf[ctx]);
}
uint8_t EntropyDecoder::readInterTxType(TxSet set, TX_SIZE txSzSqr)
{
    ASSERT(set == TX_SET_INTER_1 || set == TX_SET_INTER_2 || set == TX_SET_INTER_3);
    const int cdfSize[] = { 0, 16, 12, 2 };
    return (uint8_t)m_symbol->read(m_cdfs.inter_ext_tx_cdf[set][txSzSqr]);
}
uint8_t EntropyDecoder::readIntraTxType(TxSet set, TX_SIZE txSzSqr, PREDICTION_MODE intraDir)
{
    ASSERT(set == TX_SET_INTRA_1 || set == TX_SET_INTRA_2);
    if (set == TX_SET_INTRA_1)
        return (uint8_t)m_symbol->read(m_cdfs.intra_ext_tx_cdf[1][txSzSqr][intraDir]);
    return (uint8_t)m_symbol->read(m_cdfs.intra_ext_tx_cdf[2][txSzSqr][intraDir]);
}

bool EntropyDecoder::readAllZero(uint8_t txSzCtx, uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.txb_skip_cdf[txSzCtx][ctx]);
}

uint8_t EntropyDecoder::readEobPt(uint8_t eobMultisize, PLANE_TYPE planeType, uint8_t ctx)
{
    vector<aom_cdf_prob> *cdf;
    if (eobMultisize == 0) {
        cdf = &m_cdfs.eob_flag_cdf16[planeType][ctx];
    } else if (eobMultisize == 1) {
        cdf = &m_cdfs.eob_flag_cdf32[planeType][ctx];
    } else if (eobMultisize == 2) {
        cdf = &m_cdfs.eob_flag_cdf64[planeType][ctx];
    } else if (eobMultisize == 3) {
        cdf = &m_cdfs.eob_flag_cdf128[planeType][ctx];
    } else if (eobMultisize == 4) {
        cdf = &m_cdfs.eob_flag_cdf256[planeType][ctx];
    } else if (eobMultisize == 5) {
        cdf = &m_cdfs.eob_flag_cdf512[planeType][ctx];
    } else if (eobMultisize == 6) {
        cdf = &m_cdfs.eob_flag_cdf1024[planeType][ctx];
    } else {
        return 0;
    }
    return m_symbol->read(*cdf) + 1;
}
bool EntropyDecoder::readEobExtra(TX_SIZE txSzCtx, PLANE_TYPE ptype, int eobPt)
{
    return (bool)m_symbol->read(m_cdfs.eob_extra_cdf[txSzCtx][ptype][eobPt - 3]);
}

uint8_t EntropyDecoder::readCoeffBaseEob(uint8_t txSzCtx, PLANE_TYPE planeType, uint8_t ctx)
{
    return m_symbol->read(m_cdfs.coeff_base_eob_cdf[txSzCtx][planeType][ctx]);
}

uint8_t EntropyDecoder::readCoeffBase(uint8_t txSzCtx, PLANE_TYPE planeType, uint8_t ctx)
{
    return m_symbol->read(m_cdfs.coeff_base_cdf[txSzCtx][planeType][ctx]);
}

uint8_t EntropyDecoder::readCoeffBr(uint8_t minTx, PLANE_TYPE planeType, uint8_t ctx)
{
    return m_symbol->read(m_cdfs.coeff_br_cdf[minTx][planeType][ctx]);
}

bool EntropyDecoder::readDcSign(PLANE_TYPE planeType, uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.dc_sign_cdf[planeType][ctx]);
}

bool EntropyDecoder::readUseWiener()
{
    return (bool)m_symbol->read(m_cdfs.wiener_restore_cdf);
}

bool EntropyDecoder::readUseSgrproj()
{
    return (bool)m_symbol->read(m_cdfs.sgrproj_restore_cdf);
}

RestorationType EntropyDecoder::readRestorationType()
{
    return (RestorationType)m_symbol->read(m_cdfs.switchable_restore_cdf);
}

uint8_t EntropyDecoder::readLrSgrSet()
{
    return readLiteral(SGRPROJ_PARAMS_BITS);
}

bool EntropyDecoder::readIsInter(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.intra_inter_cdf[ctx]);
}

bool EntropyDecoder::readSkipMode(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.skip_mode_cdfs[ctx]);
}

const static uint8_t Size_Group[BLOCK_SIZES_ALL] = {
    0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3,
    3, 3, 3, 3, 3, 0, 0, 1, 1, 2, 2
};

PREDICTION_MODE EntropyDecoder::readYMode(BLOCK_SIZE MiSize)
{
    uint8_t ctx = Size_Group[MiSize];
    return (PREDICTION_MODE)m_symbol->read(m_cdfs.y_mode_cdf[ctx]);
}

CompMode EntropyDecoder::readCompMode(uint8_t ctx)
{
    return (CompMode)m_symbol->read(m_cdfs.comp_inter_cdf[ctx]);
}

COMP_REFERENCE_TYPE EntropyDecoder::readCompReferenceType(uint8_t ctx)
{
    return (COMP_REFERENCE_TYPE)m_symbol->read(m_cdfs.comp_ref_type_cdf[ctx]);
}

bool EntropyDecoder::readUniCompRef(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.uni_comp_ref_cdf[ctx][0]);
}

bool EntropyDecoder::readUniCompRefP1(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.uni_comp_ref_cdf[ctx][1]);
}

bool EntropyDecoder::readUniCompRefP2(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.uni_comp_ref_cdf[ctx][2]);
}

bool EntropyDecoder::readCompRef(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.comp_ref_cdf[ctx][0]);
}

bool EntropyDecoder::readCompRefP1(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.comp_ref_cdf[ctx][1]);
}

bool EntropyDecoder::readCompRefP2(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.comp_ref_cdf[ctx][2]);
}

bool EntropyDecoder::readCompBwdRef(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.comp_bwdref_cdf[ctx][0]);
}

bool EntropyDecoder::readCompBwdRefP1(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.comp_bwdref_cdf[ctx][1]);
}

bool EntropyDecoder::readSingleRef(uint8_t ctx, int n)
{
    return (bool)m_symbol->read(m_cdfs.single_ref_cdf[ctx][n - 1]);
}

uint8_t EntropyDecoder::readCompoundMode(uint8_t ctx)
{
    return (uint8_t)m_symbol->read(m_cdfs.inter_compound_mode_cdf[ctx]);
}

bool EntropyDecoder::readNewMv(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.newmv_cdf[ctx]);
}

bool EntropyDecoder::readZeroMv(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.zeromv_cdf[ctx]);
}

bool EntropyDecoder::readRefMv(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.refmv_cdf[ctx]);
}

bool EntropyDecoder::readDrlMode(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.drl_cdf[ctx]);
}

MV_JOINT_TYPE EntropyDecoder::readMvJoint(uint8_t ctx)
{
    return (MV_JOINT_TYPE)m_symbol->read(m_cdfs.nmv_context[ctx].joints_cdf);
}

bool EntropyDecoder::readMvSign(uint8_t ctx, uint8_t comp)
{
    return (bool)m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].sign_cdf);
}

MV_CLASS_TYPE EntropyDecoder::readMvClass(uint8_t ctx, uint8_t comp)
{
    return (MV_CLASS_TYPE)m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].classes_cdf);
}

int EntropyDecoder::readMvClass0Bit(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].class0_cdf);
}

int EntropyDecoder::readMvBit(int i, uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].bits_cdf[i]);
}

int EntropyDecoder::readMvClass0Fr(int mv_class0_bit, uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].class0_fp_cdf[mv_class0_bit]);
}

int EntropyDecoder::readMvClass0Hp(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].class0_hp_cdf);
}

int EntropyDecoder::readMvFr(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].fp_cdf);
}

int EntropyDecoder::readMvHp(uint8_t ctx, uint8_t comp)
{
    return m_symbol->read(m_cdfs.nmv_context[ctx].comps[comp].hp_cdf);
}

bool EntropyDecoder::readInterIntra(BLOCK_SIZE MiSize)
{
    uint8_t ctx = Size_Group[MiSize];
    return (bool)m_symbol->read(m_cdfs.interintra_cdf[ctx]);
}

INTERINTRA_MODE EntropyDecoder::readInterIntraMode(BLOCK_SIZE MiSize)
{
    uint8_t ctx = Size_Group[MiSize];
    return (INTERINTRA_MODE)m_symbol->read(m_cdfs.interintra_mode_cdf[ctx]);
}

bool EntropyDecoder::readWedgeInterIntra(BLOCK_SIZE MiSize)
{
    return (bool)m_symbol->read(m_cdfs.wedge_interintra_cdf[MiSize]);
}

uint8_t EntropyDecoder::readWedgeIndex(BLOCK_SIZE MiSize)
{
    return (uint8_t)m_symbol->read(m_cdfs.wedge_idx_cdf[MiSize]);
}

bool EntropyDecoder::readCompGroupIdx(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.comp_group_idx_cdf[ctx]);
}

bool EntropyDecoder::readCompoundIdx(uint8_t ctx)
{
    return (bool)m_symbol->read(m_cdfs.compound_index_cdf[ctx]);
}

COMPOUND_TYPE EntropyDecoder::readCompoundType(BLOCK_SIZE MiSize)
{
    return (COMPOUND_TYPE)m_symbol->read(m_cdfs.compound_type_cdf[MiSize]);
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
    return (InterpFilter)m_symbol->read(m_cdfs.switchable_interp_cdf[ctx]);
}

bool EntropyDecoder::readUseObmc(BLOCK_SIZE MiSize)
{
    return (bool)m_symbol->read(m_cdfs.obmc_cdf[MiSize]);
}

MOTION_MODE EntropyDecoder::readMotionMode(BLOCK_SIZE MiSize)
{
    return (MOTION_MODE)m_symbol->read(m_cdfs.motion_mode_cdf[MiSize]);
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