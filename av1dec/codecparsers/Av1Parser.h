#ifndef Av1Parser_h
#define Av1Parser_h

#include <stdint.h>
#include <vector>
#include "bitReader.h"

namespace YamiParser {
	namespace Av1 {
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

		struct SequenceHeader
		{
			uint8_t seq_profile;
			//uint8_t frame_width_bits_minus_1;
			//uint8_t frame_height_bits_minus_1;
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

			//bool enable_order_hint;
			bool enable_jnt_comp;
			bool enable_ref_frame_mvs;

			//bool seq_choose_screen_content_tools;
			uint8_t seq_force_screen_content_tools;
			bool seq_choose_integer_mv;
			bool seq_force_integer_mv;

			//uint8_t order_hint_bits_minus1;
			uint8_t OrderHintBits;
			bool enable_superres;
			bool enable_cdef;
			bool enable_restoration;

			//clolor_config()
			uint8_t BitDepth;
			bool mono_chrome;
			uint8_t color_primaries;
			uint8_t transfer_characteristics;
			uint8_t matrix_coefficients;

			uint8_t color_space;
			uint8_t transfer_function;
			bool color_range;
			bool subsampling_x;
			bool subsampling_y;
			uint8_t chroma_sample_position;
			bool  separate_uv_delta_q;

			const uint8_t SELECT_SCREEN_CONTENT_TOOLS = 2;
			const uint8_t SELECT_INTEGER_MV = 2;
			struct OperatingPoint
			{
				OperatingPoint()
				{
					memset(this, 0, sizeof(*this));
				}
				uint16_t operating_point_idc;
				uint8_t level;
				bool decoder_rate_model_param_present_flag;
				//depends on decoder_rate_model_param_present_flag;
				uint16_t decode_to_display_rate_ratio;
				uint32_t initial_display_delay;
				uint8_t extra_frame_buffers;
			};
			std::vector<OperatingPoint> operating_points;

			bool parse(BitReader& br);
		private:
			bool parseColorConfig(BitReader& br);
		};
		class Parser
		{
		public:
			bool parse(uint8_t* data, size_t size);
			Parser();
		private:
			bool parseSequenceHeader(BitReader& br);
			bool parseTemporalDelimiter(BitReader& br);
			bool parseFrameHeader(BitReader& br);
			bool praseTileGroup(BitReader& br);
			bool parseMetadata(BitReader& br);
			bool parsePadding(BitReader& br);
			bool praseReserved(BitReader& br);
			void skipTrailingBits(BitReader& br);
			bool m_seenFrameHeader;
			SequenceHeader m_sequence;



		};


	}
}

#endif