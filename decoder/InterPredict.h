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
#include "Block.h"

#include <vector>

namespace YamiAv1 {

class Block::InterPredict {
public:
    InterPredict(Block& block, int plane, YuvFrame& yuv, const FrameStore& frameStore, std::vector<std::vector<uint8_t>>& mask);
    void predict_inter(int x, int y, int w, int h, int candRow, int candCol);

private:
    uint8_t getUseWarp(int x, int y, int refFrame);
    void motionVectorScaling(int8_t refIdx, int x, int y, const Mv& mv);
    int getFilterIdx(int size, int candRow, int candCol, int dir);
    void blockInterPrediction(int8_t refIdx, int refList, int w, int h, int candRow, int candCol);
    void blockWarp(int useWarp, uint8_t refIdx, int refList, int x, int y, int i8, int j8, int w, int h);
    void intraModeVariantMask(int w, int h);
    void maskBlend(int x, int y, int w, int h);
    void predict_overlap(int pass, int candRow, int candCol, int x4, int y4, int predW, int predH, const uint8_t* mask);
    void overlappedMotionCompensation(int w, int h);
    void wedgeMask(int w, int h);
    void differenceWeightMask(int w, int h);
    void getDistanceWeights(int candRow, int candCol, int& FwdWeight, int& BckWeight);
    const Block& m_block;
    const FrameHeader& m_frame;
    const SequenceHeader& m_sequence;
    const int plane;
    const int subX;
    const int subY;
    const int MiRow;
    const int MiCol;
    LocalWarp& m_localWarp;
    YuvFrame& m_yuv;
    const FrameStore& m_frameStore;
    bool isCompound = 0;
    int InterRound0 = 0;
    int InterRound1 = 0;
    int InterPostRound = 0;
    bool globaValid = false;

    int startX = 0;
    int startY = 0;
    int xStep = 0;
    int yStep = 0;
    std::vector<std::vector<int16_t>> preds[2];
    std::vector<std::vector<uint8_t>>& Mask;
};

class Block::FindMvStack {
public:
    FindMvStack(Block& block);

    void find_mv_stack();
    uint8_t getCompoundModeCtx() const;
    uint8_t getNewMvCtx() const;
    uint8_t getZeroMvCtx() const;
    uint8_t getRefMvCtx() const;
    int getNumMvFound() const;
    uint8_t getDrlModeCtx(uint8_t idx) const;
    Mv RefStackMv[MAX_REF_MV_STACK_SIZE][2];
    Mv GlobalMvs[2];

private:
    int16_t clamp_mv_row(int16_t mvec, int border);
    int16_t clamp_mv_col(int16_t mvec, int border);
    void clampMv();
    void generateRefAndNewMvContext(uint32_t CloseMatches, int numNew, uint32_t TotalMatches);
    void generateDrlCtxStack();
    void sort(int start, int end);
    void swap_stack(int i, int j);
    static bool has_newmv(PREDICTION_MODE mode);
    void searchStack(int mvRow, int mvCol, int candList, uint32_t weight);
    void searchCompoundStack(int mvRow, int mvCol, uint32_t weight);
    void add_ref_mv_candidate(int mvRow, int mvCol, uint32_t weight);
    void scanRow(int deltaRow);
    void scanCol(int deltaCol);
    void scanPoint(int deltaRow, int deltaCol);
    void lower_mv_precision(Mv& mv);
    void setupGlobalMV(uint8_t refList);

    bool check_sb_border(int deltaRow, int deltaCol) const;

    void temporalScan();
    void extraSearch();
    void add_tpl_ref_mv(int deltaRow, int deltaCol);
    void add_extra_mv_candidate(int mvRow, int mvCol);

    const Block& m_block;
    const FrameHeader& m_frame;
    const Tile& m_tile;

    uint32_t WeightStack[MAX_REF_MV_STACK_SIZE];
    int NumMvFound = 0;
    int NewMvCount = 0;

    const int MiCol;
    const int MiRow;
    const int bw4;
    const int bh4;
    const bool isCompound;
    bool FoundMatch;
    std::vector<uint8_t> DrlCtxStack;
    uint8_t NewMvContext;
    uint8_t RefMvContext;
    uint8_t ZeroMvContext;
    std::vector<Mv> RefIdMvs[2];
    std::vector<Mv> RefDiffMvs[2];
};

}