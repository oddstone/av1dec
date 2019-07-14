#ifndef Av1Parser_h
#define Av1Parser_h

#include <stdint.h>
#include <string.h>
#include <vector>
#include <list>
#include <memory>
#include "bitReader.h"
#include "Av1Tile.h"

#define CLIP3(min, max, v) (v>max?max:((v < min)?min:v))
class Block;
class TransformBlock;

namespace YamiParser {
	namespace Av1 {

		struct obu_extension_header
		{
			uint8_t temporal_id;
			uint8_t spatial_id;
			uint8_t quality_id;
			bool parse(BitReader& br);
		};

		enum ObuType {
			OBU_SEQUENCE_HEADER = 1,
			OBU_TD = 2,
			OBU_FRAME_HEADER = 3,
			OBU_TILE_GROUP = 4,
			OBU_METADATA = 5,
			OBU_FRAME = 6,
			OBU_REDUNDANT_FRAME_HEADER = 7,
			OBU_PADDING = 15,
		};
		const char* obuType2String(uint8_t type);
		
		struct obu_header
		{
			bool parse(BitReader& br);
			ObuType obu_type;
			bool obu_extension_flag;
			bool obu_has_size_field;
			obu_extension_header extension;
			uint64_t obu_size;
		private:
			bool parseLeb128(BitReader& br, uint64_t& v);
		};


		enum ColorPrimaries {
			CP_BT_709 = 1,
			CP_UNSPECIFIED = 2,
			CP_BT_470_M = 4,
			CP_BT_470_B_G = 5,
			CP_BT_601 = 6,
			CP_SMPTE_240 = 7,
			CP_GENERIC_FILM = 8,
			CP_BT_2020 = 9,
			CP_XYZ = 10,
			CP_SMPTE_431 = 11,
			CP_SMPTE_432 = 12,
			CP_EBU_3213 = 22,
		};

		enum TransferCharacteristic {
			TC_RESERVED_0 = 0,
			TC_BT_709 = 1,
			TC_UNSPECIFIED = 2,
			TC_RESERVED_3 = 3,
			TC_BT_470_M = 4,
			TC_BT_470_B_G = 5,
			TC_BT_601 = 6,
			TC_SMPTE_240 = 7,
			TC_LINEAR = 8,
			TC_LOG_100 = 9,
			TC_LOG_100_SQRT10 = 10,
			TC_IEC_61966 = 11,
			TC_BT_1361 = 12,
			TC_SRGB = 13,
			TC_BT_2020_10_BIT = 14,
			TC_BT_2020_12_BIT = 15,
			TC_SMPTE_2084 = 16,
			TC_SMPTE_428 = 17,
			TC_HLG = 18,
		};

		enum MatrixCoefficient {
			MC_IDENTITY = 0,
			MC_BT_709 = 1,
			MC_UNSPECIFIED = 2,
			MC_RESERVED_3 = 3,
			MC_FCC = 4,
			MC_BT_470_B_G = 5,
			MC_BT_601 = 6,
			MC_SMPTE_240 = 7,
			MC_SMPTE_YCGCO = 8,
			MC_BT_2020_NCL = 9,
			MC_BT_2020_CL = 10,
			MC_SMPTE_2085 = 11,
			MC_CHROMAT_NCL = 12,
			MC_CHROMAT_CL = 13,
			MC_ICTCP = 14,
		};

		enum ChromaSamplePosition{
			CSP_UNKNOWN = 0,
			CSP_VERTICAL = 1,
			CSP_COLOCATED = 2,
			CSP_RESERVED = 3,
		};

		const static uint8_t SELECT_SCREEN_CONTENT_TOOLS = 2;
		const static uint8_t SELECT_INTEGER_MV = 2;

		struct SequenceHeader
		{
			uint8_t seq_profile;
			bool still_picture;
			bool reduced_still_picture_header;
			bool timing_info_present_flag;
			bool decoder_model_info_present_flag;
			bool initial_display_delay_present_flag;

			uint8_t frame_width_bits_minus_1;
			uint8_t frame_height_bits_minus_1;
			uint32_t max_frame_width_minus_1;
			uint32_t max_frame_height_minus_1;

			bool frame_id_numbers_present_flag;
			uint8_t delta_frame_id_length_minus2;
			uint8_t additional_frame_id_length_minus1;

			bool use_128x128_superblock;

			bool enable_filter_intra;
			bool enable_intra_edge_filter;
			bool enable_interintra_compound;
			bool enable_masked_compound;
			bool enable_warped_motion;
			bool enable_dual_filter;

			bool enable_order_hint;
			bool enable_jnt_comp;
			bool enable_ref_frame_mvs;

			//bool seq_choose_screen_content_tools;
			uint8_t seq_force_screen_content_tools;
			bool seq_choose_integer_mv;
			uint8_t seq_force_integer_mv;

			//uint8_t order_hint_bits_minus1;
			uint8_t OrderHintBits;
			bool enable_superres;
			bool enable_cdef;
			bool enable_restoration;

			//clolor_config()
			uint8_t BitDepth;
			bool mono_chrome;
			uint8_t NumPlanes;
			uint8_t color_primaries;
			uint8_t transfer_characteristics;
			uint8_t matrix_coefficients;

			uint8_t color_space;
			uint8_t transfer_function;
			bool color_range;
			int subsampling_x;
			int subsampling_y;
			uint8_t chroma_sample_position;
			bool  separate_uv_delta_q;

			uint32_t num_units_in_tick;
			uint32_t time_scale;
			bool equal_picture_interval;
			uint32_t num_ticks_per_picture_minus1;
			
			bool film_grain_params_present;


			struct OperatingPoint
			{
				OperatingPoint()
				{
					memset(this, 0, sizeof(*this));
				}
				uint16_t operating_point_idc;
				uint8_t seq_level_idx;
				bool seq_tier;
				bool decoder_model_present_for_this_op;
				bool initial_display_delay_present_for_this_op;
				//depends on decoder_rate_model_param_present_flag;
				//uint16_t decode_to_display_rate_ratio;
				//uint32_t initial_display_delay;
				//uint8_t extra_frame_buffers;
			};
			std::vector<OperatingPoint> operating_points;

			bool parse(BitReader& br);
		private:
			bool parseColorConfig(BitReader& br);
			bool parseTimingInfo(BitReader& br);
		};

		class Parser;
		struct FrameHeader;
		enum FrameType
		{
			KEY_FRAME = 0,
			INTER_FRAME = 1,
			INTRA_ONLY_FRAME = 2,
			SWITCH_FRAME = 3,
		};
		
		enum TXMode {
			ONLY_4X4 = 0,
			TX_MODE_LARGEST = 1,
			TX_MODE_SELECT = 2,
		};
		const static uint8_t NUM_REF_FRAMES = 8;
		const static uint8_t PRIMARY_REF_NONE = 7;
		const static uint16_t MAX_TILE_WIDTH = 4096;
		const static uint32_t MAX_TILE_AREA = 4906 * 2304;
		const static uint32_t MAX_TILE_COLS = 64;
		const static uint32_t MAX_TILE_ROWS = 64;

		struct Quantization
		{
			uint32_t base_q_idx;
			int8_t DeltaQYDc;
			int8_t DeltaQUDc;
			int8_t DeltaQUAc;
			int8_t DeltaQVDc;
			int8_t DeltaQVAc;
			bool using_qmatrix;
			uint8_t qm_y;
			uint8_t qm_u;
			uint8_t qm_v;
			bool parse(BitReader& br, const SequenceHeader& seq);
		};
		enum SEG_LVL_FEATURE {
			SEG_LVL_ALT_Q,
			SEG_LVL_ALT_LF_Y_V,
			SEG_LVL_REF_FRAME = 5,
			SEG_LVL_SKIP,
			SEG_LVL_GLOBALMV,
			SEG_LVL_MAX,
		};
		static const int MAX_LOOP_FILTER = 63;
		static const int Segmentation_Feature_Bits[SEG_LVL_MAX] = { 8, 6, 6, 6, 6, 3, 0, 0 };
		static const int Segmentation_Feature_Signed[SEG_LVL_MAX] = { 1, 1, 1, 1, 1, 0, 0, 0 };
		static const int Segmentation_Feature_Max[SEG_LVL_MAX] = {
			255, MAX_LOOP_FILTER, MAX_LOOP_FILTER,
			MAX_LOOP_FILTER, MAX_LOOP_FILTER, 7,
			0, 0 };

		struct Segmentation
		{

			bool parse(BitReader& br);

			bool segmentation_enabled;
			bool FeatureEnabled[MAX_SEGMENTS][SEG_LVL_MAX];
			int16_t FeatureData[MAX_SEGMENTS][SEG_LVL_MAX];
			bool 	SegIdPreSkip;
			uint8_t LastActiveSegId;


;
			bool seg_feature_active_idx(int segmentId, SEG_LVL_FEATURE);
		private:
		};

		struct DeltaQ
		{
			bool delta_q_present;
			uint8_t delta_q_res;
			bool parse(BitReader& br, const Quantization& quant);
		};

		struct DeltaLf
		{
			bool delta_lf_present;
			uint8_t delta_lf_res;
			bool delta_lf_multi;
			bool parse(BitReader& br, const DeltaQ& deltaQ);
		};

		struct LoopFilter
		{
			const static int LOOP_FILTER_LEVEL_COUNT = 4;
			const static int LOOP_FILTER_MODE_DELTA_COUNT = 2;
			uint8_t loop_filter_level[LOOP_FILTER_LEVEL_COUNT];
			int8_t loop_filter_ref_deltas[TOTAL_REFS_PER_FRAME];
			uint8_t loop_filter_sharpness;
			int8_t loop_filter_mode_deltas[LOOP_FILTER_MODE_DELTA_COUNT];
			bool parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame);

		};
		struct Cdef
		{
			bool parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame);
		};

		struct FrameHeader
		{
			friend class Block;
			friend class TransformBlock;
			bool show_existing_frame;
			uint8_t frame_to_show_map_idx;
			uint8_t refresh_frame_flags;
			uint8_t frame_type;
			bool FrameIsIntra;
			bool show_frame;
			bool showable_frame;
			bool RefValid[NUM_REF_FRAMES];
			uint8_t RefOrderHint[NUM_REF_FRAMES];
			bool error_resilient_mode;
			bool disable_cdf_update;
			bool allow_screen_content_tools;
			bool force_integer_mv;
			uint32_t PrevFrameID;
			uint32_t current_frame_id;
			uint32_t RefFrameId[NUM_REF_FRAMES];
			bool frame_size_override_flag;
			uint8_t order_hint;
			uint8_t primary_ref_frame;
			bool allow_high_precision_mv;
			bool use_ref_frame_mvs;
			bool allow_intrabc;

			//frame_size()
			uint32_t FrameWidth;
			uint32_t FrameHeight;

			//compute_image_size()
			uint32_t MiCols;
			uint32_t MiRows;

			//superres_params()
			bool use_superres;
			uint8_t SuperresDenom;
			uint32_t UpscaledWidth;

			//render_size()
			uint32_t RenderWidth;
			uint32_t RenderHeight;

			bool disable_frame_end_update_cdf;

			//tile_info()
			uint32_t TileCols;
			uint32_t TileRows;
			uint32_t NumTiles;
			uint32_t TileColsLog2;
			uint32_t TileRowsLog2;
			uint8_t TileSizeBytes;

			Quantization m_quant;
			Segmentation m_segmentation;
			DeltaQ m_deltaQ;
			DeltaLf m_deltaLf;
			LoopFilter m_loopFilter;

			bool enable_warped_motion;
			bool allow_warped_motion;

			bool CodedLossless;
			bool AllLossless;
			bool LosslessArray[MAX_SEGMENTS];
			uint8_t SegQMLevel[3][MAX_SEGMENTS];

			bool reduced_tx_set;

			TXMode TxMode;

			std::vector<uint32_t> MiColStarts;
			std::vector<uint32_t> MiRowStarts;
			std::vector<std::vector<TX_TYPE>> TxTypes;


			FrameHeader();
			bool parse(BitReader& br, const SequenceHeader& sequence);
			int16_t get_qindex(bool ignoreDeltaQ, int segmentId) const;
		private:
			void mark_ref_frames(const SequenceHeader& sequence, uint8_t idLen);
			bool parseFrameSize(BitReader& br, const SequenceHeader& sequence);
			bool parseSuperresParams(BitReader& br, const SequenceHeader& sequence);
			bool parseRenderSize(BitReader& br);
			bool parseTileInfo(BitReader& br, const SequenceHeader& sequence);
			bool parseTileStarts(BitReader& br, std::vector<uint32_t>& starts, uint32_t sbMax, uint32_t sbShift, uint32_t maxTileSb);
			bool parseQuantizationParams(BitReader& br, const SequenceHeader& sequence);
			bool parseTxMode(BitReader& br, const FrameHeader& frame);
			void initGeometry();
			const static uint8_t SUPERRES_DENOM_MIN = 9;
			const static uint8_t SUPERRES_NUM = 8;
			const static uint8_t SUPERRES_DENOM_BITS = 3;
			const static uint8_t TX_MODES = 3;

		};

		
		struct TileGroup {
			//std::list<Tile> m_group;
		};
		typedef std::shared_ptr<TileGroup> TileGroupPtr;
		class Parser
		{
		public:

			Parser();
			bool parseSequenceHeader(BitReader& br);
			bool parseTemporalDelimiter(BitReader& br);
			bool parseFrameHeader(BitReader& br);
			bool parseTileGroup(BitReader& br, TileGroupPtr& group);
			bool parseMetadata(BitReader& br);
			bool parsePadding(BitReader& br);
			bool parseFrame(BitReader& br, TileGroupPtr& group);
			bool praseReserved(BitReader& br);
		private:
			void skipTrailingBits(BitReader& br);
			bool m_seenFrameHeader;
			SequenceHeader m_sequence;
			FrameHeader m_frameHeader;
			TileGroup m_group;
			
		};


	}
}

#endif