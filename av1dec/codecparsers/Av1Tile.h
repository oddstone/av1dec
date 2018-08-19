#pragma once
#include <stdint.h>
#include <memory>
#include "EntropyDecoder.h"
#include "../aom/enums.h"

namespace YamiParser {
	namespace Av1 {
		class SymbolDecoder;
		struct FrameHeader;
		struct SequenceHeader;

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


		struct Tile {
			uint32_t MiRowStart;
			uint32_t MiRowEnd;
			uint32_t MiColStart;
			uint32_t MiColEnd;
			uint32_t CurrentQIndex;
			Tile(const SequenceHeader& sequence, const FrameHeader& frame, uint32_t TileNum);

			bool decode(const uint8_t* data, uint32_t size);
			bool decodePartition(uint32_t r, uint32_t c, BLOCK_SIZE sbSize);
			bool decodeBlock(uint32_t r, uint32_t c, BLOCK_SIZE bSize);
			PARTITION_TYPE readPartition(uint32_t r, uint32_t c, bool AvailU, bool AvailL, BLOCK_SIZE bSize);
			void readModeInfo(uint32_t r, uint32_t c, BLOCK_SIZE bSize);
			void readIntraFrameModeInfo(uint32_t r, uint32_t c, BLOCK_SIZE bSize);
			void readInterFrameModeInfo();

			uint8_t getSkipCtx();
			bool readSkip();
			

			void readCdef();
			void readDeltaQindex();
			void readDeltaLf();

			PREDICTION_MODE readIntraFrameYMode();
			void intraAngleInfoY();
			UV_PREDICTION_MODE readUvMode(PREDICTION_MODE yMode);
			void intra_angle_info_uv(UV_PREDICTION_MODE uvMode);
			void filter_intra_mode_info(BLOCK_SIZE bSize);
		private:
			bool isInside(uint32_t r, uint32_t c);
			const FrameHeader& m_frame;
			const SequenceHeader& m_sequence;
			std::unique_ptr<EntropyDecoder> m_entropy;

			//uint16_t m_partitionCdf[PARTITION_WIDTH_TYPES][PARTITION_CONTEXTS][EXT_PARTITION_TYPES + 1];
			//uint16_t m_skipCdf[SKIP_CONTEXTS][3];
			//uint16_t m_intraFrameYModeCdf[INTRA_MODE_CONTEXTS][INTRA_MODE_CONTEXTS][INTRA_MODES + 1];

		};
	}
}