#include "Av1Tile.h"
#include "codecparsers/Av1Parser.h"
#include "SymbolDecoder.h"
#include "common/log.h"
#include "Block.h"
#include "TransformBlock.h"

#include <limits>

Block::Block(Tile& tile, uint32_t r, uint32_t c, BLOCK_SIZE subSize)
	: m_frame(tile.m_frame)
	, m_sequence(tile.m_sequence)
	, m_entropy(*tile.m_entropy)
	, m_tile(tile)
	, m_decoded(tile.m_decoded)
	, MiRow(r)
	, MiCol(c)
	, MiSize(subSize)
	, bw4(Num_4x4_Blocks_Wide[subSize])
	, bh4(Num_4x4_Blocks_High[subSize])
	, sbMask(m_sequence.use_128x128_superblock ? 31 : 15)
	, subsampling_x(m_sequence.subsampling_x)
	, subsampling_y(m_sequence.subsampling_y)
{
	if (bh4 == 1 && subsampling_y && (MiRow & 1) == 0)
		HasChroma = true;
	else if (bw4 == 1 && subsampling_x && (MiCol & 1) == 0)
		HasChroma = true;
	else
		HasChroma = m_sequence.NumPlanes > 1;
	AvailU = tile.is_inside( r - 1, c );
	AvailL = tile.is_inside( r, c - 1 );
	AvailUChroma = AvailU;
	AvailLChroma = AvailL;
	if ( HasChroma ) {
		if ( subsampling_y && bh4 == 1 )
			AvailUChroma = tile.is_inside( r - 2, c );
		if ( subsampling_x && bw4 == 1 )
			AvailLChroma = tile.is_inside( r, c - 2 );
	} else {
		AvailUChroma = false;
		AvailLChroma = false;
	}
}

BLOCK_SIZE Subsampled_Size[BLOCK_SIZES_ALL][ 2 ][ 2 ] = {
	{ { BLOCK_4X4, BLOCK_4X4}, {BLOCK_4X4, BLOCK_4X4} },
	{ { BLOCK_4X8, BLOCK_4X4}, {BLOCK_INVALID, BLOCK_4X4} },
	{ { BLOCK_8X4, BLOCK_INVALID}, {BLOCK_4X4, BLOCK_4X4} },
	{ { BLOCK_8X8, BLOCK_8X4}, {BLOCK_4X8, BLOCK_4X4} },
	{ {BLOCK_8X16, BLOCK_8X8}, {BLOCK_INVALID, BLOCK_4X8} },
	{ {BLOCK_16X8, BLOCK_INVALID}, {BLOCK_8X8, BLOCK_8X4} },
	{ {BLOCK_16X16, BLOCK_16X8}, {BLOCK_8X16, BLOCK_8X8} },
	{ {BLOCK_16X32, BLOCK_16X16}, {BLOCK_INVALID, BLOCK_8X16} },
	{ {BLOCK_32X16, BLOCK_INVALID}, {BLOCK_16X16, BLOCK_16X8} },
	{ {BLOCK_32X32, BLOCK_32X16}, {BLOCK_16X32, BLOCK_16X16} },
	{ {BLOCK_32X64, BLOCK_32X32}, {BLOCK_INVALID, BLOCK_16X32} },
	{ {BLOCK_64X32, BLOCK_INVALID}, {BLOCK_32X32, BLOCK_32X16} },
	{ {BLOCK_64X64, BLOCK_64X32}, {BLOCK_32X64, BLOCK_32X32} },
	{ {BLOCK_64X128, BLOCK_64X64}, {BLOCK_INVALID, BLOCK_32X64} },
	{ {BLOCK_128X64, BLOCK_INVALID}, {BLOCK_64X64, BLOCK_64X32} },
	{ {BLOCK_128X128, BLOCK_128X64}, {BLOCK_64X128, BLOCK_64X64} },
	{ {BLOCK_4X16, BLOCK_4X8}, {BLOCK_INVALID, BLOCK_4X8} },
	{ {BLOCK_16X4, BLOCK_INVALID}, {BLOCK_8X4, BLOCK_8X4} },
	{ {BLOCK_8X32, BLOCK_8X16}, {BLOCK_INVALID, BLOCK_4X16} },
	{ {BLOCK_32X8, BLOCK_INVALID}, {BLOCK_16X8, BLOCK_16X4} },
	{ {BLOCK_16X64, BLOCK_16X32}, {BLOCK_INVALID, BLOCK_8X32} },
	{ {BLOCK_64X16, BLOCK_INVALID}, {BLOCK_32X16, BLOCK_32X8} },
};

BLOCK_SIZE Block::get_plane_residual_size(int subsize, int plane ) {
	int subx = plane > 0 ? subsampling_x : 0;
	int suby = plane > 0 ? subsampling_y : 0;
	return Subsampled_Size[subsize][subx][suby];
}

int16_t Block::get_q_idx()
{
	return m_frame.m_segmentation.segmentation_enabled ?
		m_frame.get_qindex(true, segment_id) : m_frame.m_quant.base_q_idx;
}

void Block::compute_prediction()
{

	uint32_t subBlockMiRow = MiRow & sbMask;
	uint32_t subBlockMiCol = MiCol & sbMask;
	for (int plane = 0; plane < 1 + HasChroma * 2; plane++ ) {
		int planeSz = get_plane_residual_size( MiSize, plane );
		int num4x4W = Num_4x4_Blocks_Wide[ planeSz ];
		int num4x4H = Num_4x4_Blocks_High[ planeSz ];
		int log2W = MI_SIZE_LOG2 + Mi_Width_Log2[ planeSz ];
		int log2H = MI_SIZE_LOG2 + Mi_Height_Log2[ planeSz ];
		int subX = (plane > 0) ? subsampling_x : 0;
		int subY = (plane > 0) ? subsampling_y : 0;
		int baseX = (MiCol >> subX) * MI_SIZE;
		int baseY = (MiRow >> subY) * MI_SIZE;
		int candRow = (MiRow >> subY) << subY;
		int candCol = (MiCol >> subX) << subX;
		bool IsInterIntra = ( is_inter && RefFrame[ 1 ] == INTRA_FRAME );
		if ( IsInterIntra ) {
			ASSERT(0 && "IsInterIntra");
		}
		if (is_inter) {
			ASSERT(0 && "is_inter");
		}
		
	}
}

void Block::predict_intra(int plane, int startX, int startY,
	int availL, int availU, bool decodedUpRight, bool decodedBottomLeft,
	int mode, int log2W, int log2H)
{
	if (mode != PAETH_PRED) {
		ASSERT(0 && "not PAETH_PRED");
	}
	
}
	
void Block::transform_block(int plane, int baseX, int baseY, TX_SIZE txSz, int x, int y)
{
	int startX = baseX + 4 * x;
	int startY = baseY + 4 * y;
	int subX = (plane > 0) ? subsampling_x : 0;
	int subY = (plane > 0) ? subsampling_y : 0;
	int row = ( startY << subY ) >> MI_SIZE_LOG2;
	int col = ( startX << subX ) >> MI_SIZE_LOG2;
	int subBlockMiRow = row & sbMask;
	int subBlockMiCol = col & sbMask;
	int stepX = Tx_Width[ txSz ] >> MI_SIZE_LOG2;
	int stepY = Tx_Height[ txSz ] >> MI_SIZE_LOG2;
	int maxX = (m_frame.MiCols * MI_SIZE) >> subX;
	int maxY = (m_frame.MiRows * MI_SIZE) >> subY;
	if ( startX >= maxX || startY >= maxY ) {
		return;
	}
	if (!is_inter) {
		if ( ( ( plane == 0 ) && PaletteSizeY ) ||
			( ( plane != 0 ) && PaletteSizeUV ) ) {
			ASSERT(0 && "predict_palette");
			//predict_palette( plane, startX, startY, x, y, txSz )
		} else {
			bool isCfl = (plane > 0 && UVMode == UV_CFL_PRED);
			int mode;
			if ( plane == 0 ) {
				mode = YMode;
			} else {
				mode = ( isCfl ) ? DC_PRED : UVMode;
			}
			int log2W = Tx_Width_Log2[ txSz ];
			int log2H = Tx_Height_Log2[txSz];
			predict_intra( plane, startX, startY,
				( plane == 0 ? AvailL : AvailLChroma ) || x > 0,
				( plane == 0 ? AvailU : AvailUChroma ) || y > 0,
				m_decoded.getFlag(plane, (subBlockMiRow >> subY) - 1, (subBlockMiCol >> subX) + stepX),
				m_decoded.getFlag(plane, (subBlockMiRow >> subY) + stepY, (subBlockMiCol >> subX) - 1),
				mode,
				log2W, log2H );
			if ( isCfl ) {
				ASSERT(0 && "predict_chroma_from_luma");
				//predict_chroma_from_luma( plane, startX, startY, txSz);
			}
		}
		if ( plane == 0 ) {
			int MaxLumaW = startX + stepX * 4;
			int MaxLumaH = startY + stepY * 4;
		}
	}
	if (!skip) {
		TransformBlock tb(*this, plane, startX, startY, txSz);

		tb.decode();

	}
	for (int  i = 0; i < stepY; i++ ) {
		for (int  j = 0; j < stepX; j++ ) {
/*		LoopfilterTxSizes[ plane ]
		[ (row >> subY) + i ]
		[ (col >> subX) + j ] = txSz*/
			m_decoded.setFlag(plane, ( subBlockMiRow >> subY ) + i, ( subBlockMiCol >> subX ) + j );
		}
	}
}

TX_SIZE Block::get_tx_size(int plane, TX_SIZE txSz) {
	if (plane == 0)
		return txSz;
	TX_SIZE uvTx = Max_Tx_Size_Rect[get_plane_residual_size(MiSize, plane)];
	if (Tx_Width[uvTx] == 64 || Tx_Height[uvTx] == 64) {
		if (Tx_Width[uvTx] == 16) {
			return TX_16X32;
		}
		if (Tx_Height[uvTx] == 16) {
			return TX_32X16;
		}
		return TX_32X32;
	}
	return uvTx;
}


void Block::residual()
{
	int Block_Width = Num_4x4_Blocks_Wide[MiSize] << 2;
	int Block_Height = Num_4x4_Blocks_High[MiSize] << 2;
	int widthChunks = std::max( 1, Block_Width >> 6 );
	int heightChunks = std::max( 1, Block_Height >> 6 );
	int miSizeChunk = ( widthChunks > 1 || heightChunks > 1 ) ? BLOCK_64X64 : MiSize;
	for (int chunkY = 0; chunkY < heightChunks; chunkY++ ) {
		for (int chunkX = 0; chunkX < widthChunks; chunkX++ ) {
			int miRowChunk = MiRow + ( chunkY << 4 );
			int miColChunk = MiCol + ( chunkX << 4 );
			int subBlockMiRow = miRowChunk & sbMask;
			for (int plane = 0; plane < 1 + HasChroma * 2; plane++ ) {
				TX_SIZE txSz = Lossless ? TX_4X4 : get_tx_size( plane, TxSize );
				int stepX = Tx_Width[ txSz ] >> 2;
				int stepY = Tx_Height[ txSz ] >> 2;
				int planeSz = get_plane_residual_size( miSizeChunk, plane );
				int num4x4W = Num_4x4_Blocks_Wide[ planeSz ];
				int num4x4H = Num_4x4_Blocks_High[ planeSz ];
				int subX = (plane > 0) ? subsampling_x : 0;
				int subY = (plane > 0) ? subsampling_y : 0;
				int baseX = (miColChunk >> subX) * MI_SIZE;
				int baseY = (miRowChunk >> subY) * MI_SIZE;
				if ( is_inter && !Lossless && !plane ) {
					ASSERT(0 && "transform_tree");
					//transform_tree( baseX, baseY, num4x4W * 4, num4x4H * 4 )
				} else {
					int baseXBlock = (MiCol >> subX) * MI_SIZE;
					int baseYBlock = (MiRow >> subY) * MI_SIZE;
					for (int y = 0; y < num4x4H; y += stepY )
						for (int x = 0; x < num4x4W; x += stepX )
						transform_block( plane, baseXBlock, baseYBlock, txSz,
							x + ( ( chunkX << 4 ) >> subX ),
							y + ( ( chunkY << 4 ) >> subY ) );
				}
			}
		}
	}
}

void Block::reset_block_context()
{
	for (int plane = 0; plane < 1 + 2 * HasChroma; plane++ ) {
		int subX = (plane > 0) ? subsampling_x : 0;
		int subY = (plane > 0) ? subsampling_y : 0;
		m_tile.m_above.reset(plane, MiCol >> subX, bw4);
		m_tile.m_left.reset(plane, MiRow >> subY, bh4);
	}
}

bool Block::decode()
{
	mode_info();
	palette_tokens();
	read_block_tx_size();
	if (skip)
		reset_block_context();
	bool isCompund = RefFrame[1] > INTRA_FRAME;
	/*
		for ( y = 0; y < bh4; y++ ) {
		for ( x = 0; x < bw4; x++ ) {
		YModes [ r + y ][ c + x ] = YMode
		if ( RefFrame[ 0 ] == INTRA_FRAME && HasChroma )
		UVModes [ r + y ][ c + x ] = UVMode
		for ( refList = 0; refList < 2; refList++ )
		RefFrames[ r + y ][ c + x ][ refList ] = RefFrame[ refList ]
		if ( is_inter ) {
		if ( !use_intrabc ) {
		CompGroupIdxs[ r + y ][ c + x ] = comp_group_idx
		CompoundIdxs[ r + y ][ c + x ] = compound_idx
		}
		for ( dir = 0; dir < 2; dir++ ) {
		InterpFilters[ r + y ][ c + x ][ dir ] = interp_filter[ dir ]
		}
		for ( refList = 0; refList < 1 + isCompound; refList++ ) {
		Mvs[ r + y ][ c + x ][ refList ] = Mv[ refList ]
		}
		}
		}
	}
	*/
	compute_prediction();
	residual();

	return true;
}

uint8_t Block::getSkipCtx()
{
	return 0;
}

bool Block::readSkip()
{
	uint8_t ctx = getSkipCtx();
	return m_entropy.readSkip(ctx);
}

void Block::readCdef()
{
	if (m_frame.CodedLossless)
		return;
	ASSERT(0 && "readCdef");

}

void Block::read_delta_qindex(bool readDeltas)
{
	BLOCK_SIZE sbSize = m_sequence.use_128x128_superblock ? BLOCK_128X128 : BLOCK_64X64;
	if (MiSize == sbSize && skip)
		return;
	if (readDeltas) {
		ASSERT(0);
	}
}

void Block::read_delta_lf(bool ReadDeltas)
{
	if (!(ReadDeltas && m_frame.m_deltaLf.delta_lf_present))
		return;
	ASSERT(0);
}

PREDICTION_MODE Block::intra_frame_y_mode()
{
	uint8_t Intra_Mode_Context[INTRA_MODES] = {
		0, 1, 2, 3, 4, 4, 4, 4, 3, 0, 1, 2, 0
	};
	PREDICTION_MODE above = DC_PRED;
	PREDICTION_MODE	left = DC_PRED;
	uint8_t aboveCtx = Intra_Mode_Context[above];
	uint8_t leftCtx = Intra_Mode_Context[left];
	return m_entropy.readIntraFrameYMode(aboveCtx, leftCtx);
}

bool is_directional_mode( PREDICTION_MODE mode ) {
	return ( mode >= V_PRED ) && ( mode <= D67_PRED );
}

bool is_directional_mode(UV_PREDICTION_MODE mode) {
	return (mode >= UV_V_PRED) && (mode <= UV_D67_PRED);
}

void Block::intra_angle_info_y()
{
	if (MiSize >= BLOCK_8X8) {
		if (is_directional_mode(YMode)) {
			ASSERT(0);
		}
	}
}

UV_PREDICTION_MODE Block::uv_mode()
{

	CFL_ALLOWED_TYPE cflAllowed = (std::max(bw4, bh4) <= 8) ? CFL_ALLOWED : CFL_DISALLOWED;
	return m_entropy.readUvMode(cflAllowed, YMode);

}

void Block::intra_angle_info_uv()
{
	if (MiSize > BLOCK_8X8) {
		if (is_directional_mode(UVMode)) {
			ASSERT(0);
			uint8_t angle_delta_uv = m_entropy.readAngleDeltaUV(UVMode);
		}
	}
}

void Block::filter_intra_mode_info()
{
	use_filter_intra = false;
	if (m_sequence.enable_filter_intra &&
		YMode == DC_PRED && PaletteSizeY == 0 && 
		std::max(bw4, bh4) <= 8) {
		use_filter_intra = m_entropy.readUseFilterIntra(MiSize);
		if (use_filter_intra) {
			ASSERT(0);
		}
	}

}

void Block::palette_tokens()
{
	if ( PaletteSizeY ) {
		ASSERT(0);
	}
}

void Block::intra_segment_id()
{
	if (m_frame.m_segmentation.segmentation_enabled)
		ASSERT(0);
	else
		segment_id = 0;
	Lossless = m_frame.LosslessArray[segment_id];
}

void Block::intra_frame_mode_info()
{
	bool SegIdPreSkip = m_frame.m_segmentation.SegIdPreSkip;
	if (SegIdPreSkip)
		intra_segment_id();
	skip = readSkip();
	if (!SegIdPreSkip)
		intra_segment_id();
	readCdef();
	bool ReadDeltas = m_frame.m_deltaQ.delta_q_present;
	read_delta_qindex(ReadDeltas);
	read_delta_lf(ReadDeltas);

	RefFrame[ 0 ] = INTRA_FRAME;
	RefFrame[ 1 ] = NONE_FRAME;
	
	bool use_intrabc;
	if (m_frame.allow_intrabc) {
		ASSERT(0 && "intrabc");
	} else {
		use_intrabc = false;
	}
	if (use_intrabc) {
		is_inter = true;
		YMode = DC_PRED;
		ASSERT(0);
	} else {
		is_inter = false;
		YMode = intra_frame_y_mode();
		intra_angle_info_y();
		if (HasChroma) {
			UV_PREDICTION_MODE uvMode = uv_mode();
			if (uvMode == UV_CFL_PRED) {
				ASSERT(0);
				//read_cfl_alphas()
			}
			intra_angle_info_uv();

		}

		PaletteSizeY = 0;
		PaletteSizeUV = 0;
		if ( MiSize >= BLOCK_8X8 &&
			bw4 <= 16 &&
			bh4 <= 16 &&
			m_frame.allow_screen_content_tools)
		{
			ASSERT(0);
			//palette_mode_info( )
		}

		filter_intra_mode_info();
	}

}

void Block::inter_frame_mode_info()
{
	ASSERT(0);

}

void Block::mode_info()
{
	if (m_frame.FrameIsIntra)
		intra_frame_mode_info();
	else
		inter_frame_mode_info();
	
}

void Block::read_tx_size(bool allowSelect)
{
	if (Lossless) {
		TxSize = TX_4X4;
		return;
	}
	TX_SIZE maxRectTxSize = Max_Tx_Size_Rect[ MiSize ];
	int maxTxDepth = Max_Tx_Depth[MiSize];
	TxSize = maxRectTxSize;
	if ( MiSize > BLOCK_4X4 && allowSelect && m_frame.TxMode == TX_MODE_SELECT ) {
		ASSERT(0);
		//tx_depth;
		/*for (int i = 0; i < tx_depth; i++ )
			TxSize = Split_Tx_Size[ TxSize ];
		}*/
	}
	
}

void Block::read_block_tx_size()
{
	if (m_frame.TxMode == TX_MODE_SELECT &&
		MiSize > BLOCK_4X4 && is_inter &&
		!skip && !m_frame.CodedLossless) {
		ASSERT(0);
	} else {
		read_tx_size(!skip || !is_inter);
		for (int row = MiRow; row < MiRow + bh4; row++) {
			for (int col = MiCol; col < MiCol + bw4; col++ )
				InterTxSizes[ row ][ col ] = TxSize;
		}
	}
}

