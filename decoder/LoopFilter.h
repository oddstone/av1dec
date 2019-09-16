#ifndef Av1LoopFilter_h
#define Av1LoopFilter_h
#include <memory>
#include "enums.h"
#include "VideoFrame.h"

namespace YamiParser {
    namespace Av1 {
        class Parser;
        struct FrameHeader;
        struct SequenceHeader;
        struct LoopFilterParams;
        struct DeltaLf;
    }
}
using namespace YamiParser::Av1;

class LoopFilter {
public:
    LoopFilter(const Parser& parser);
    void filter(const std::shared_ptr<YuvFrame>& frame);
private:
    void loop_filter_edge(const std::shared_ptr<YuvFrame>& frame,
        int plane, int pass, int row, int col);
    void sampleFilter(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane,
        int limit, int blimit, int thresh,
        int dx, int dy, int filterSize);
    void narrowFilter(const std::shared_ptr<YuvFrame>& frame,
        int hevMask, int x, int y, int plane, int dx, int dy);
    void wideFilter(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane, int dx, int dy, int log2Size);
    void getFilterMask(const std::shared_ptr<YuvFrame>& frame,
        int x, int y, int plane, int limit, int blimit, int thresh,
        int dx, int dy, int filterSize, int& hevMask, int& filterMask, int& flatMask, int& flatMask2);

    int getFilterSize(TX_SIZE txSz, TX_SIZE prevTxSz, int pass, int plane);
    void getFilterStrength(int row, int col, int plane, int pass, int& lvl, int& limit, int& blimit, int& thresh);
    int getLimit(int lvl);
    int8_t getLvl(uint8_t segment, int ref, int modeType, int8_t deltaLF, int plane, int pass);
    int8_t getDeltaLF(int row, int col, int plane, int pass);

    bool isOnScreen(int x, int y, int pass);

    static bool getIsBlockEdge(int xP, int yP, BLOCK_SIZE planeSize, int pass);

    static bool getIsTxEdge(int xP, int yP, TX_SIZE txSize, int pass);

    static bool getApplyFilter(bool isTxEdge, bool isBlockEdge, bool skip, bool isIntra);

    const FrameHeader& m_frame;
    const SequenceHeader& m_sequence;
    const LoopFilterParams& m_filter;
    const DeltaLf& m_deltaLF;
};

#endif