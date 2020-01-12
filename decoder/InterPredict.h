#pragma once
#include "Block.h"

#include <vector>


namespace YamiAv1 {

class Block::InterPredict {
public:
    InterPredict(Block& block, int plane, YuvFrame& yuv, const FrameStore& frameStore);
    void predict_inter(int x, int y, uint32_t w, uint32_t h, int candRow, int candCol);

private:
    uint8_t getUseWarp(int x, int y, int refFrame);
    void motionVectorScaling(uint8_t refIdx, int x, int y, const Mv& mv);
    int getFilterIdx(int size, int candRow, int candCol, int dir);
    void blockInterPrediction(uint8_t refIdx, int refList, uint32_t w, uint32_t h, int candRow, int candCol);
    void blockWarp(int useWarp, uint8_t refIdx, int refList, int x, int y, int i8, int j8, int w, int h);
    void intraModeVariantMask(int w, int h);
    void maskBlend(int x, int y, int w, int h);
    void predict_overlap(int pass, int candRow, int candCol, int x4, int y4, int predW, int predH, const uint8_t* mask);
    void overlappedMotionCompensation(int w, int h);
    void wedgeMask(int w, int h);
    const Block& m_block;
    const FrameHeader& m_frame;
    const SequenceHeader& m_sequence;
    const int plane;
    const int subX;
    const int subY;
    const uint32_t MiRow;
    const uint32_t MiCol;
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
    std::vector<std::vector<uint8_t>> Mask;
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
    void searchStack(uint32_t mvRow, uint32_t mvCol, int candList, uint32_t weight);
    void searchCompoundStack(uint32_t mvRow, uint32_t mvCol, uint32_t weight);
    void add_ref_mv_candidate(uint32_t mvRow, uint32_t mvCol, uint32_t weight);
    void scanRow(int deltaRow);
    void scanCol(int deltaCol);
    void scanPoint(int deltaRow, int deltaCol);
    void lower_mv_precision(Mv& mv);
    void setupGlobalMV(uint8_t refList);

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

    const uint32_t MiCol;
    const uint32_t MiRow;
    const uint32_t bw4;
    const uint32_t bh4;
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