#ifndef LoopRestoration_h
#define LoopRestoration_h
#include <memory>
#include "enums.h"
#include "VideoFrame.h"
#include "Av1Parser.h"

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

class LoopRestoration {
public:
    LoopRestoration(const Parser& parser,
        const std::shared_ptr<YuvFrame>& upscaledCdefFrame,
        const std::shared_ptr<YuvFrame>& upscaledCurrFrame);
    std::shared_ptr<YuvFrame> filter();
private:
    uint8_t get_source_sample(int plane, int x, int y,
        int StripeStartY, int StripeEndY);
    void wienerFilter(const std::shared_ptr<YuvFrame>& LrFrame,
        int plane, int unitRow, int unitCol,
        int x, int y, int w, int h, int StripeStartY, int StripeEndY);
    void loop_restore_block(const         std::shared_ptr<YuvFrame>& LrFrame,
        int plane, int row, int col );
    const FrameHeader& m_frame;
    const SequenceHeader& m_sequence;
    const LoopRestorationpParams& m_loopRestoration;
    static const int MAX_PLANES = 3;
    int unitSize[MAX_PLANES];
    int unitRows[MAX_PLANES];
    int unitCols[MAX_PLANES];
    int PlaneEndX[MAX_PLANES];
    int PlaneEndY[MAX_PLANES];
    const std::shared_ptr<YuvFrame>& UpscaledCdefFrame;
    const std::shared_ptr<YuvFrame>& UpscaledCurrFrame;
    int InterRound0;
    int InterRound1;
    int InterPostRound;
};

#endif
