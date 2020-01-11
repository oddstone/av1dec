#pragma once

#include "Av1Common.h"
#include "VideoFrame.h"
#include <memory>

namespace YamiAv1 {
class Parser;
struct LoopFilterParams;
struct DeltaLf;

class LoopFilter {
public:
    LoopFilter(const ConstFramePtr&);
    void filter(const std::shared_ptr<YuvFrame>& frame) const;

private:
    void loop_filter_edge(const std::shared_ptr<YuvFrame>& frame,
        int plane, int pass, int row, int col) const;
    void sampleFilter(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane,
        int limit, int blimit, int thresh,
        int dx, int dy, int filterSize) const;
    void narrowFilter(const std::shared_ptr<YuvFrame>& frame,
        int hevMask, int x, int y, int plane, int dx, int dy) const;
    void wideFilter(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane, int dx, int dy, int log2Size) const;
    void getFilterMask(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane, int limit, int blimit, int thresh,
        int dx, int dy, int filterSize, int& hevMask, int& filterMask, int& flatMask, int& flatMask2) const;

    int getFilterSize(TX_SIZE txSz, TX_SIZE prevTxSz, int pass, int plane) const;
    void getFilterStrength(int row, int col, int plane, int pass, int& lvl, int& limit, int& blimit, int& thresh) const;
    int getLimit(int lvl) const;
    int8_t getLvl(uint8_t segment, int ref, int modeType, int8_t deltaLF, int plane, int pass) const;
    int8_t getDeltaLF(int row, int col, int plane, int pass) const;

    bool isOnScreen(int x, int y, int pass) const;

    static bool getIsBlockEdge(int xP, int yP, BLOCK_SIZE planeSize, int pass);

    static bool getIsTxEdge(int xP, int yP, TX_SIZE txSize, int pass);

    static bool getApplyFilter(bool isTxEdge, bool isBlockEdge, bool skip, bool isIntra);

    ConstFramePtr m_frame;
    const SequenceHeader& m_sequence;
    const LoopFilterParams& m_filter;
    const DeltaLf& m_deltaLF;
};
}
