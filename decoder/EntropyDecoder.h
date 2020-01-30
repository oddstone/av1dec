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

#include "../aom/entropy.h"
#include "../aom/entropymode.h"
#include "../aom/enums.h"
#include "../aom/filter.h"
#include "../aom/prob.h"
#include "../aom/seg_common.h"
#include "Av1Common.h"
#include "Cdfs.h"
#include "SymbolDecoder.h"
#include <memory>

namespace YamiAv1 {

class EntropyDecoder {
public:
    EntropyDecoder(const uint8_t* data, uint32_t sz, bool disable_cdf_update, Cdfs& cdfs);
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

    uint8_t readDeltaLfAbs(bool delta_lf_multi, int i);
    uint8_t readDeltaQAbs();

    //for palette
    bool readHasPaletteY(uint8_t bsizeCtx, uint8_t ctx);
    uint8_t readPaletteSizeY(uint8_t bsizeCtx);
    bool readHasPaletteUV(uint8_t ctx);
    uint8_t readPaletteSizeUV(uint8_t bsizeCtx);
    uint8_t readPaletteColorIdxY(uint8_t PaletteSize, uint8_t ColorContextHash);
    uint8_t readPaletteColorIdxUV(uint8_t PaletteSize, uint8_t ColorContextHash);

    //for inter
    bool readIsInter(uint8_t ctx);
    bool readSkipMode(uint8_t ctx);
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

    uint32_t readNS(int n);

private:
    uint8_t getPartitionCdfCount(uint8_t bsl);
    int decode_unsigned_subexp_with_ref_bool(int mx, int k, int r);
    int decode_subexp_bool(int numSyms, int k);
    std::unique_ptr<SymbolDecoder> m_symbol;
    Cdfs& m_cdfs;
};

}