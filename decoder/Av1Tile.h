#ifndef Av1Tile_h
#define Av1Tile_h
#include "../aom/enums.h"
#include "EntropyDecoder.h"
#include <memory>
#include <stdint.h>
#include <vector>
#include <deque>

class Partition;

namespace YamiParser {
namespace Av1 {
    class SymbolDecoder;
    struct FrameHeader;
    struct SequenceHeader;
}
}
using namespace YamiParser::Av1;

struct YuvFrame;

const static uint8_t Mi_Width_Log2[BLOCK_SIZES_ALL] = {
    0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3,
    4, 4, 4, 5, 5, 0, 2, 1, 3, 2, 4
};

const static uint8_t Mi_Height_Log2[BLOCK_SIZES_ALL] = {
    0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 4,
    3, 4, 5, 4, 5, 2, 0, 3, 1, 4, 2
};

/*
		enum BlockType {
			BLOCK_4X4,
			BLOCK_4X8,
			BLOCK_8X4,
			BLOCK_8X8,
			BLOCK_8X16,
			BLOCK_16X8,
			BLOCK_16X16,
			BLOCK_16X32,
			BLOCK_32X16,
			BLOCK_32X32,
			BLOCK_32X64,
			BLOCK_64X32,
			BLOCK_64X64,
			BLOCK_64X128,
			BLOCK_128X64,
			BLOCK_128X128,
			BLOCK_4X16,
			BLOCK_16X4,
			BLOCK_8X32,
			BLOCK_32X8,
			BLOCK_16X64,
			BLOCK_64X16,
			BLOCK_SIZES,
			BLOCK_INVALID = BLOCK_SIZES,
		};

		enum PARTITION_TYPE {
			PARTITION_NONE,
			PARTITION_HORZ,
			PARTITION_VERT,
			PARTITION_SPLIT,
			PARTITION_HORZ_A,
			PARTITION_HORZ_B,
			PARTITION_VERT_A,
			PARTITION_VERT_B,
			PARTITION_HORZ_4,
			PARTITION_VERT_4,
			EXT_PARTITION_TYPES,
			PARTITION_TYPES = PARTITION_SPLIT + 1,
			PARTITION_INVALID = 255,
		};

		const uint8_t PARTITION_CONTEXTS = 4;
		const uint8_t PARTITION_WIDTH_TYPES = 6;
		const uint8_t SKIP_CONTEXTS = 3;

		// Note: All directional predictors must be between V_PRED and D67_PRED (both
		// inclusive).
		enum PREDICTION_MODE {
			DC_PRED,        // Average of above and left pixels
			V_PRED,         // Vertical
			H_PRED,         // Horizontal
			D45_PRED,       // Directional 45  degree
			D135_PRED,      // Directional 135 degree
			D113_PRED,      // Directional 113 degree
			D157_PRED,      // Directional 157 degree
			D203_PRED,      // Directional 203 degree
			D67_PRED,       // Directional 67  degree
			SMOOTH_PRED,    // Combination of horizontal and vertical interpolation
			SMOOTH_V_PRED,  // Vertical interpolation
			SMOOTH_H_PRED,  // Horizontal interpolation
			PAETH_PRED,     // Predict from the direction of smallest gradient
			NEARESTMV,
			NEARMV,
			GLOBALMV,
			NEWMV,
			// Compound ref compound modes
			NEAREST_NEARESTMV,
			NEAR_NEARMV,
			NEAREST_NEWMV,
			NEW_NEARESTMV,
			NEAR_NEWMV,
			NEW_NEARMV,
			GLOBAL_GLOBALMV,
			NEW_NEWMV,
			MB_MODE_COUNT,
			INTRA_MODES = PAETH_PRED + 1,  // PAETH_PRED has to be the last intra mode.
			INTRA_INVALID = MB_MODE_COUNT  // For uv_mode in inter blocks
		};
		const uint8_t INTRA_MODE_CONTEXTS = 5;

		enum  UV_PREDICTION_MODE  {
			UV_DC_PRED,        // Average of above and left pixels
			UV_V_PRED,         // Vertical
			UV_H_PRED,         // Horizontal
			UV_D45_PRED,       // Directional 45  degree
			UV_D135_PRED,      // Directional 135 degree
			UV_D113_PRED,      // Directional 113 degree
			UV_D157_PRED,      // Directional 157 degree
			UV_D203_PRED,      // Directional 203 degree
			UV_D67_PRED,       // Directional 67  degree
			UV_SMOOTH_PRED,    // Combination of horizontal and vertical interpolation
			UV_SMOOTH_V_PRED,  // Vertical interpolation
			UV_SMOOTH_H_PRED,  // Horizontal interpolation
			UV_PAETH_PRED,     // Predict from the direction of smallest gradient
			UV_CFL_PRED,       // Chroma-from-Luma
			UV_INTRA_MODES,
			UV_MODE_INVALID,  // For uv_mode in inter blocks
		} ;
		*/
class Tile;
class SuperBlock;

class BlockDecoded {
public:
    BlockDecoded();
    void init(Tile& tile);
    void clearFlags(int r, int c, int sbSize4);
    void setFlag(int plane, int r, int c);
    bool getFlag(int plane, int r, int c);

private:
    static const int SIZE = 128 / 4 + 2;
    static const int OFFSET = 1;
    bool m_decoded[3][SIZE][SIZE];
    bool subsampling_x;
    bool subsampling_y;
    int NumPlanes;
    int MiColEnd;
    int MiRowEnd;
};

struct BlockContext {
    std::vector<std::vector<int16_t>> LevelContext;
    std::vector<std::vector<uint8_t>> DcContext;
    std::vector<bool> SegPredContext;
    void clear(uint32_t size);
    void reset(uint32_t plane, uint32_t start, uint32_t count);
    void set(uint32_t plane, uint32_t start, uint32_t count, int16_t culLevel, uint8_t dcCategory);
};

class Tile {
    uint32_t MiRowStart;
    uint32_t MiRowEnd;
    uint32_t MiColStart;
    uint32_t MiColEnd;
    uint32_t CurrentQIndex;
    friend class Block;
    friend class BlockDecoded;
    friend class TransformBlock;
    friend class Partition;
    friend class SuperBlock;

public:
    Tile(std::shared_ptr<const SequenceHeader> sequence, std::shared_ptr<FrameHeader> frame, uint32_t TileNum);
    bool parse(const uint8_t* data, uint32_t size);
    bool decode(std::shared_ptr<YuvFrame>& frame);

    std::shared_ptr<FrameHeader> m_frame;
    std::shared_ptr<const SequenceHeader> m_sequence;
    std::unique_ptr<EntropyDecoder> m_entropy;

private:
    //bool decodePartition(uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
    bool decodeBlock(uint32_t r, uint32_t c, BLOCK_SIZE bSize);
//    PARTITION_TYPE readPartition(uint32_t r, uint32_t c, bool AvailU, bool AvailL, BLOCK_SIZE bSize);

private:
    bool is_inside(uint32_t r, uint32_t c);
    void clear_above_context();
    void clear_left_context();


    BlockDecoded m_decoded;

    BlockContext m_above;
    BlockContext m_left;
    std::deque<std::shared_ptr<SuperBlock>> m_sbs;

    //uint16_t m_partitionCdf[PARTITION_WIDTH_TYPES][PARTITION_CONTEXTS][EXT_PARTITION_TYPES + 1];
    //uint16_t m_skipCdf[SKIP_CONTEXTS][3];
    //uint16_t m_intraFrameYModeCdf[INTRA_MODE_CONTEXTS][INTRA_MODE_CONTEXTS][INTRA_MODES + 1];
};
#endif
