#ifndef Block_h
#define Block_h

#include <stdint.h>
#include <memory>
#include "EntropyDecoder.h"
#include "../aom/enums.h"
#include "Av1Tile.h"

namespace YamiParser {
	namespace Av1 {
		class SymbolDecoder;
		struct FrameHeader;
		struct SequenceHeader;
		struct FrameHeader;
	}	
}

using namespace YamiParser::Av1;

const int Num_4x4_Blocks_Wide[BLOCK_SIZES_ALL] = {
	1, 1, 2, 2, 2, 4, 4, 4, 8, 8, 8,
	16, 16, 16, 32, 32, 1, 4, 2, 8, 4, 16
};

const int Num_4x4_Blocks_High[BLOCK_SIZES_ALL] = {
	1, 2, 1, 2, 4, 2, 4, 8, 4, 8, 16,
	8, 16, 32, 16, 32, 4, 1, 8, 2, 16, 4
};


const static int Tx_Width[TX_SIZES_ALL] = {
	4, 8, 16, 32, 64, 4, 8, 8, 16, 16, 32, 32, 64, 4, 16, 8, 32, 16, 64
};

const static int Tx_Height[TX_SIZES_ALL] = {
	4, 8, 16, 32, 64, 8, 4, 16, 8, 32, 16, 64, 32, 16, 4, 32, 8, 64, 16
};

const static TX_SIZE Max_Tx_Size_Rect[BLOCK_SIZES_ALL] = {
	TX_4X4, TX_4X8, TX_8X4, TX_8X8,
	TX_8X16, TX_16X8, TX_16X16, TX_16X32,
	TX_32X16, TX_32X32, TX_32X64, TX_64X32,
	TX_64X64, TX_64X64, TX_64X64, TX_64X64,
	TX_4X16, TX_16X4, TX_8X32, TX_32X8,
	TX_16X64, TX_64X16
};

const static int Max_Tx_Depth[BLOCK_SIZES_ALL] = {
	0, 1, 1, 1,
	2, 2, 2, 3,
	3, 3, 4, 4,
	4, 4, 4, 4,
	2, 2, 3, 3,
	4, 4
};

const static TX_SIZE Split_Tx_Size[TX_SIZES_ALL] = {
	TX_4X4,
	TX_4X4,
	TX_8X8,
	TX_16X16,
	TX_32X32,
	TX_4X4,
	TX_4X4,
	TX_8X8,
	TX_8X8,
	TX_16X16,
	TX_16X16,
	TX_32X32,
	TX_32X32,
	TX_4X8,
	TX_8X4,
	TX_8X16,
	TX_16X8,
	TX_16X32,
	TX_32X16
};

static const int Tx_Width_Log2[ TX_SIZES_ALL ] = {
	2, 3, 4, 5, 6, 2, 3, 3, 4, 4, 5, 5, 6, 2, 4, 3, 5, 4, 6
};

static const int Tx_Height_Log2[ TX_SIZES_ALL ] = {
	2, 3, 4, 5, 6, 3, 2, 4, 3, 5, 4, 6, 5, 4, 2, 5, 3, 6, 4
};

enum TxSet {
	TX_SET_DCTONLY = 0,
	TX_SET_INTRA_1,
	TX_SET_INTRA_2,
	TX_SET_INTER_1 = 1,
	TX_SET_INTER_2,
	TX_SET_INTER_3,
};

class Block {
	friend class TransformBlock;
public:
	Block(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE bSize);
	bool decode();
private:
	void intra_segment_id();
	void mode_info();
	void intra_frame_mode_info();
	void inter_frame_mode_info();

	uint8_t getSkipCtx();
	bool readSkip();
	

	void readCdef();
	void read_delta_qindex(bool readDeltas);
	void read_delta_lf(bool readDeltas);

	PREDICTION_MODE intra_frame_y_mode();
	void intra_angle_info_y();
	UV_PREDICTION_MODE uv_mode();
	void intra_angle_info_uv();
	void filter_intra_mode_info();

	void palette_tokens();
	void read_block_tx_size();
	void read_tx_size(bool allowSelect);
	void compute_prediction();

	void reset_block_context();
	BLOCK_SIZE get_plane_residual_size(int subsize, int plane);
	bool is_tx_type_in_set(TxSet txSet, TX_TYPE txType);
	TX_TYPE compute_tx_type(int plane, TX_SIZE txSz, int blockX, int blockY);
	TX_SIZE get_tx_size(int plane, TX_SIZE txSz);
	TxSet get_tx_set(TX_SIZE txSz);

	void predict_intra(int plane, int startX, int startY,
		int availL, int availU, bool decodedUpRight, bool decodedBottomLeft,
		int mode, int log2W, int log2H);
	uint8_t getAllZeroCtx(int plane, int x4, int y4, int w4, int h4, TX_SIZE txSz);
	int16_t get_q_idx();

	void transform_block(int plane, int baseX, int baseY, TX_SIZE txSz, int x, int y);
	void residual();


	FrameHeader& m_frame;
	const SequenceHeader& m_sequence;
	Tile& m_tile;
	BlockDecoded m_decoded;
	

	//from spec
	uint32_t MiRow;
	uint32_t MiCol;
	BLOCK_SIZE MiSize;
	uint32_t bw4;
	uint32_t bh4;
	bool HasChroma;
	bool AvailU;
	bool AvailL;
	bool AvailUChroma;
	bool AvailLChroma;
	int subsampling_x;
	int subsampling_y;
	bool is_inter;
	uint8_t segment_id;
	bool Lossless;
	bool skip;
	PREDICTION_MODE YMode;
	UV_PREDICTION_MODE UVMode;
	uint32_t PaletteSizeY = 0;
	uint32_t PaletteSizeUV = 0;
	bool use_filter_intra;
	TX_SIZE TxSize;
	TX_SIZE InterTxSizes[BLOCK_SIZES][BLOCK_SIZES];
	int RefFrame[2];
	uint32_t sbMask;
	std::vector<std::vector<uint8_t>> TxTypes;

	EntropyDecoder& m_entropy;
};
#endif
