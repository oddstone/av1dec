#include "Av1Tile.h"
#include "codecparsers/Av1Parser.h"
#include "SymbolDecoder.h"
#include "common/log.h"


namespace YamiParser {
	namespace Av1 {
		

		const int Num_4x4_Blocks_Wide[BLOCK_SIZES_ALL] = {
			1, 1, 2, 2, 2, 4, 4, 4, 8, 8, 8,
			16, 16, 16, 32, 32, 1, 4, 2, 8, 4, 16
		};

		BLOCK_SIZE Partition_Subsize[10][BLOCK_SIZES_ALL] = {
			{
				BLOCK_4X4,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X128,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X4,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X128,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X4,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X4,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X4,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_128X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X128,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X128,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X4,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_32X8,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_64X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			},{
				BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_4X16,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_8X32,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_16X64,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID,
				BLOCK_INVALID, BLOCK_INVALID, BLOCK_INVALID
			}
		};

		

		static void CopyAndReverse(void* dest, const void* src, uint32_t size)
		{
			#define CDF_PROB_BITS 15
			#define CDF_PROB_TOP (1 << CDF_PROB_BITS)
			const uint16_t* cdf = (const uint16_t*)src;
			uint16_t* icdf = (uint16_t*)dest;
			for (uint32_t i = 0; i < size; i++) {
				*icdf = CDF_PROB_TOP - *cdf;
				icdf++;
				cdf++;
			}

		}
		Tile::Tile(const SequenceHeader& sequence, const FrameHeader& frame, uint32_t TileNum)
			: m_sequence(sequence)
			, m_frame(frame)
		{
			uint32_t TileRow = TileNum / frame.TileCols;
			uint32_t TileCol = TileNum % frame.TileCols;
			MiRowStart = frame.MiRowStarts[TileRow];
			MiRowEnd = frame.MiRowStarts[TileRow + 1];
			MiColStart = frame.MiColStarts[TileCol];
			MiColEnd = frame.MiColStarts[TileCol + 1];
			CurrentQIndex = frame.m_quant.base_q_idx;

			//CopyAndReverse(m_partitionCdf, Default_Partition_Cdf, sizeof(Default_Partition_Cdf));
			//CopyAndReverse(m_skipCdf, Default_Skip_Cdf, sizeof(Default_Skip_Cdf));
			//CopyAndReverse(m_intraFrameYModeCdf, Default_Intra_Frame_Y_Mode_Cdf, sizeof(Default_Intra_Frame_Y_Mode_Cdf));

		}

		bool Tile::isInside(uint32_t r, uint32_t c)
		{
			return (c >= MiColStart &&
				c < MiColEnd &&
				r >= MiRowStart &&
				r < MiRowEnd);
		}

		uint8_t Tile::getSkipCtx()
		{
			return 0;
		}

		bool Tile::readSkip()
		{
			uint8_t ctx = getSkipCtx();
			return m_entropy->readSkip(ctx);
		}

		void Tile::readCdef()
		{
			if (m_frame.CodedLossless)
				return;
			ASSERT(0 && "readCdef");

		}
		void Tile::readDeltaQindex()
		{

		}

		void Tile::readDeltaLf()
		{

		}

		PREDICTION_MODE Tile::readIntraFrameYMode()
		{
			uint8_t Intra_Mode_Context[INTRA_MODES] = {
				0, 1, 2, 3, 4, 4, 4, 4, 3, 0, 1, 2, 0
			};
			PREDICTION_MODE above = DC_PRED;
			PREDICTION_MODE	left = DC_PRED;
			uint8_t aboveCtx = Intra_Mode_Context[above];
			uint8_t leftCtx = Intra_Mode_Context[left];
			return m_entropy->readIntraFrameYMode(aboveCtx, leftCtx);
		}

		void Tile::intraAngleInfoY()
		{

		}

		UV_PREDICTION_MODE Tile::readUvMode(PREDICTION_MODE yMode)
		{

			CFL_ALLOWED_TYPE cflAllowed = CFL_ALLOWED;
			/* TODO: get right allowed type*/
			return m_entropy->readUvMode(cflAllowed, yMode);

		}

		void Tile::intra_angle_info_uv(UV_PREDICTION_MODE uvMode)
		{
			uint8_t angle_delta_uv = m_entropy->readAngleDeltaUV(uvMode);


		}

		void Tile::filter_intra_mode_info(BLOCK_SIZE bSize)
		{
			bool use_filter_intra = m_entropy->readUseFilterIntra(bSize);
			if (use_filter_intra) {
				///todo
			}

		}

		void Tile::readIntraFrameModeInfo(uint32_t r, uint32_t c, BLOCK_SIZE bSize)
		{
			bool SegIdPreSkip = false;
			if (SegIdPreSkip)
				ASSERT(0);
			bool skip = readSkip();
			/*
			if ( !SegIdPreSkip )
			intra_segment_id( )
			*/
			readCdef();
			readDeltaQindex();
			readDeltaLf();
			bool use_intrabc;
			if (m_frame.allow_intrabc) {
				ASSERT(0 && "intrabc");
			} else {
				use_intrabc = false;
			}
			if (use_intrabc) {
				ASSERT(0);
			} else {
				PREDICTION_MODE intra_frame_y_mode = readIntraFrameYMode();
				intraAngleInfoY();
				bool HasChroma = true;
				if (HasChroma) {
					UV_PREDICTION_MODE uvMode = readUvMode(intra_frame_y_mode);
					if (uvMode == UV_CFL_PRED) {
						//TODO
						//read_cfl_alphas()
					}
					intra_angle_info_uv(uvMode);

				}
				/*
				PaletteSizeY = 0
				PaletteSizeUV = 0
				if ( MiSize >= BLOCK_8X8 &&
				Block_Width[ MiSize ] <= 64 &&
				Block_Height[ MiSize ] <= 64 &&
				allow_screen_content_tools ) {
					palette_mode_info( )
				}
				*/
				filter_intra_mode_info(bSize);
			}

		}

		void Tile::readInterFrameModeInfo()
		{

		}

		void Tile::readModeInfo(uint32_t r, uint32_t c, BLOCK_SIZE bSize)
		{
			if (m_frame.FrameIsIntra)
				readIntraFrameModeInfo(r, c, bSize);
			else
				readInterFrameModeInfo();
			
		}

		bool Tile::decodeBlock(uint32_t r, uint32_t c, BLOCK_SIZE bSize)
		{
			/* MiRow = r
				MiCol = c
				MiSize = subSize
				bw4 = Num_4x4_Blocks_Wide[ subSize ]
				bh4 = Num_4x4_Blocks_High[ subSize ]
				if ( bh4 == 1 && subsampling_y && (MiRow & 1) == 0 )
				HasChroma = 0
				else if ( bw4 == 1 && subsampling_x && (MiCol & 1) == 0 )
				HasChroma = 0
				else
				HasChroma = NumPlanes > 1
				AvailU = is_inside( r - 1, c )
				AvailL = is_inside( r, c - 1 )
				AvailUChroma = AvailU
				AvailLChroma = AvailL
				if ( HasChroma ) {
				if ( subsampling_y && bh4 == 1 )
				AvailUChroma = is_inside( r - 2, c )
				if ( subsampling_x && bw4 == 1 )
				AvailLChroma = is_inside( r, c - 2 )
				} else {
				AvailUChroma = 0
				AvailLChroma = 0
				}
				*/
			readModeInfo(r, c, bSize);

			return true;
		}

		bool Tile::decodePartition(uint32_t r, uint32_t c, BLOCK_SIZE bSize)
		{
			uint32_t MiRows = m_frame.MiRows;
			uint32_t MiCols = m_frame.MiCols;
			if (r >= MiRows || c >= MiCols)
				return true;
			bool AvailU = isInside(r - 1, c);
			bool AvailL = isInside(r, c - 1);
			int num4x4 = Num_4x4_Blocks_Wide[bSize];
			int halfBlock4x4 = num4x4 >> 1;
			int quarterBlock4x4 = halfBlock4x4 >> 1;
			bool hasRows = (r + halfBlock4x4) < MiRows;
			bool hasCols = (c + halfBlock4x4) < MiCols;

			PARTITION_TYPE partition;
			if (bSize < BLOCK_8X8) {
				partition = PARTITION_NONE;
			} else if (hasRows && hasCols) {
				partition = readPartition(r, c, AvailU, AvailL, bSize);
			} else if (hasCols) {
				//split_or_horz S()
				//			partition = split_or_horz ? PARTITION_SPLIT : PARTITION_HORZ
				ASSERT(0);
			} else if (hasRows) {
				//split_or_vert S()
				//partition = split_or_vert ? PARTITION_SPLIT : PARTITION_VERT
				ASSERT(0);
			} else {
				partition = PARTITION_SPLIT;
			}
			BLOCK_SIZE subSize = Partition_Subsize[partition][bSize];
			BLOCK_SIZE splitSize = Partition_Subsize[PARTITION_SPLIT][bSize];

			if (partition == PARTITION_NONE) {
				decodeBlock(r, c, subSize);
			} else if (partition == PARTITION_HORZ) {
				decodeBlock(r, c, subSize);
				if (hasRows)
					decodeBlock(r + halfBlock4x4, c, subSize);
			} else if (partition == PARTITION_VERT) {
				decodeBlock(r, c, subSize);
				if (hasCols)
					decodeBlock(r, c + halfBlock4x4, subSize);
			} else if (partition == PARTITION_SPLIT) {
				decodePartition(r, c, subSize);
				decodePartition(r, c + halfBlock4x4, subSize);
				decodePartition(r + halfBlock4x4, c, subSize);
				decodePartition(r + halfBlock4x4, c + halfBlock4x4, subSize);
			} else if (partition == PARTITION_HORZ_A) {
				decodeBlock(r, c, splitSize);
				decodeBlock(r, c + halfBlock4x4, splitSize);
				decodeBlock(r + halfBlock4x4, c, subSize);
			} else if (partition == PARTITION_HORZ_B) {
				decodeBlock(r, c, subSize);
				decodeBlock(r + halfBlock4x4, c, splitSize);
				decodeBlock(r + halfBlock4x4, c + halfBlock4x4, splitSize);
			} else if (partition == PARTITION_VERT_A) {
				decodeBlock(r, c, splitSize);
				decodeBlock(r + halfBlock4x4, c, splitSize);
				decodeBlock(r, c + halfBlock4x4, subSize);
			} else if (partition == PARTITION_VERT_B) {
				decodeBlock(r, c, subSize);
				decodeBlock(r, c + halfBlock4x4, splitSize);
				decodeBlock(r + halfBlock4x4, c + halfBlock4x4, splitSize);
			} else if (partition == PARTITION_HORZ_4) {
				decodeBlock(r + quarterBlock4x4 * 0, c, subSize);
				decodeBlock(r + quarterBlock4x4 * 1, c, subSize);
				decodeBlock(r + quarterBlock4x4 * 2, c, subSize);
				if (r + quarterBlock4x4 * 3 < MiRows)
					decodeBlock(r + quarterBlock4x4 * 3, c, subSize);
			} else {
				decodeBlock(r, c + quarterBlock4x4 * 0, subSize);
				decodeBlock(r, c + quarterBlock4x4 * 1, subSize);
				decodeBlock(r, c + quarterBlock4x4 * 2, subSize);
				if (c + quarterBlock4x4 * 3 < MiCols)
					decodeBlock(r, c + quarterBlock4x4 * 3, subSize);
			}
			return true;
		}



		bool Tile::decode(const uint8_t* data, uint32_t size)
		{
			m_entropy.reset(new EntropyDecoder(data, size, m_frame.disable_cdf_update, m_frame.m_quant.base_q_idx));
			/*
			clear_above_context( )
			for ( i = 0; i < FRAME_LF_COUNT; i++ )
			DeltaLF[ i ] = 0
			for ( plane = 0; plane < NumPlanes; plane++ ) {
			for ( pass = 0; pass < 2; pass++ ) {
			RefSgrXqd[ plane ][ pass ] = Sgrproj_Xqd_Mid[ pass ]
			for ( i = 0; i < WIENER_COEFFS; i++ ) {
			RefLrWiener[ plane ][ pass ][ i ] = Wiener_Taps_Mid[ i ]
			}
			}
			}
			*/
			BLOCK_SIZE sbSize = m_sequence.use_128x128_superblock ? BLOCK_128X128 : BLOCK_64X64;
			int sbSize4 = Num_4x4_Blocks_Wide[sbSize];
			for (uint32_t r = MiRowStart; r < MiRowEnd; r += sbSize4) {
				//clear_left_context()
				for (uint32_t c = MiColStart; c < MiColEnd; c += sbSize4) {
					//ReadDeltas = delta_q_present
					//clear_cdef(r, c)
					//clear_block_decoded_flags(r, c, sbSize4)
					//read_lr(r, c, sbSize)
					//decode_partition(r, c, sbSize)
					decodePartition(r, c, sbSize);
				}
			}
			return true;
		}

		uint8_t Mi_Width_Log2[BLOCK_SIZES_ALL] = {
			0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3,
			4, 4, 4, 5, 5, 0, 2, 1, 3, 2, 4
		};

		uint8_t Mi_Height_Log2[BLOCK_SIZES_ALL] = {
			0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 4,
			3, 4, 5, 4, 5, 2, 0, 3, 1, 4, 2
		};

		

		


		PARTITION_TYPE Tile::readPartition(uint32_t r, uint32_t c, bool AvailU, bool AvailL, BLOCK_SIZE bSize)
		{	
			uint8_t bsl = Mi_Width_Log2[bSize];
			/*
			uint8_t above = AvailU && (Mi_Width_Log2[MiSizes[r - 1][c]] < bsl);
			uint8_t left = AvailL && (Mi_Height_Log2[MiSizes[r][c - 1]] < bsl);
			uint8_t ctx = left * 2 + above;
			*/
			uint8_t ctx = 0;
			return m_entropy->readPartition(ctx, bsl);
		}
	}
}