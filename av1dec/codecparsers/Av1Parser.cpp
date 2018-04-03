#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include "Av1Parser.h"
#include "common/log.h"

#define READ(f)                             \
    do {                                    \
        if (!br.readT(f)) {                 \
            ERROR("failed to read %s", #f); \
            return false;                   \
        }                                   \
    } while (0)

#define READ_BITS(f, bits)                              \
    do {                                                \
        if (!br.readT(f, bits)) {                       \
            ERROR("failed to read %d to %s", bits, #f); \
            return false;                               \
        }                                               \
    } while (0)

#define CHECK_EQ(f, v)									\
	do {												\
		if (f != v) {									\
			ERROR("%s not equals to %d", #f, v);		\
			return false;								\
		}												\
	} while(0)


namespace YamiParser {
namespace Av1 {
	struct obu_extension_header
	{
		uint8_t temporal_id;
		uint8_t spatial_id;
		uint8_t quality_id;
		bool parse(BitReader& br)
		{
			READ_BITS(temporal_id, 3);
			READ_BITS(spatial_id, 2);
			READ_BITS(quality_id, 2);

			bool reserved_flag;
			READ(reserved_flag);
			return true;
		}
	};
	enum {
		OBU_SEQUENCE_HEADER = 1,
		OBU_TD = 2,
		OBU_FRAME_HEADER = 3,
		OBU_TILE_GROUP = 4,
		OBU_METADATA = 5,
		OBU_FRAME = 6,
		OBU_REDUNDANT_FRAME_HEADER = 7,
		OBU_PADDING = 15,
	};
	const char* obuType2String(uint8_t type)
	{
		switch (type) {
		case OBU_SEQUENCE_HEADER:
			return "OBU_SEQUENCE_HEADER";
		case OBU_TD:
			return "OBU_TD";
		case OBU_FRAME_HEADER:
			return "OBU_FRAME_HEADER";
		case OBU_TILE_GROUP:
			return "OBU_TILE_GROUP";
		case OBU_METADATA:
			return "OBU_METADATA";
		case OBU_FRAME:
			return "OBU_FRAME";
		case OBU_PADDING:
			return "OBU_PADDING";
		default:
			return "OBU_Unknow";
		}
	}

	struct obu_header
	{
		bool parse(BitReader& br)
		{
			memset(this, 0, sizeof(*this));
			bool obu_forbidden_bit;
			READ(obu_forbidden_bit);
			CHECK_EQ(obu_forbidden_bit, 0);

			READ_BITS(obu_type, 4);

			

			READ(obu_extension_flag);

			bool obu_has_size_field;
			READ(obu_has_size_field);

			bool obu_reserved_1bits;
			READ(obu_reserved_1bits);

			if (obu_extension_flag) {
				if (!extension.parse(br))
					return false;
			}
			//obu_has_size_field = true;
			if (obu_has_size_field) {
				if (!parseLeb128(br, obu_size)) {
					ERROR("parse Leb 128 failed");
					return false;
				}

			}
			else {
				obu_size = br.getRemainingBitsCount() >> 3;
			}
			uint64_t left = br.getRemainingBitsCount() >> 3;
			if (obu_size > left) {
				ERROR("obu_size(%d) > left (%d)", (int)obu_size, (int)left);
				return false;
			}


			return true;
		}
		uint8_t obu_type;
		bool obu_extension_flag;
		bool obu_has_size_field;
		obu_extension_header extension;
		uint64_t obu_size;
	private:
		bool parseLeb128(BitReader& br, uint64_t& v)
		{
			const int kMaxSize = 8;
			v = 0;
			uint8_t leb128_byte;
			for (int i = 0; i < kMaxSize; i++)
			{
				READ(leb128_byte);
				v |= ((leb128_byte & 0x7f) << (i * 7));
				if (!(leb128_byte & 0x80))
				{
					break;
				}
				
			}
			return true;


		}
	};

	bool SequenceHeader::parseColorConfig(BitReader& br)
	{
		//color_config()
		bool high_bitdepth;
		READ(high_bitdepth);

		if (seq_profile == 2 && high_bitdepth) {
			bool twelve_bit;
			READ(twelve_bit);
			BitDepth = twelve_bit ? 12 : 10;
		} else if (seq_profile <= 2) {
			BitDepth = high_bitdepth ? 10 : 8;
		} else {
			ERROR("Unsupported profile %d", seq_profile);
			return false;
		}
		if (seq_profile == 1) {
			mono_chrome = false;
		} else {
			READ(mono_chrome);
		}

		bool color_description_present_flag;
		READ(color_description_present_flag);
		if (color_description_present_flag) {
			READ(color_primaries);
			READ(transfer_characteristics);
			READ(matrix_coefficients);
		} else {
			color_primaries = CP_UNSPECIFIED;
			transfer_characteristics = TC_UNSPECIFIED;
			matrix_coefficients = MC_UNSPECIFIED;

		}

		if (mono_chrome) {
			color_range = true;
			subsampling_x = true;
			subsampling_y = true;
			chroma_sample_position = CSP_UNKNOWN;
			separate_uv_delta_q = 0;
			return true;
		}
		if ((color_primaries == CP_BT_709 &&
			transfer_characteristics == TC_SRGB &&
			matrix_coefficients == MC_IDENTITY)) {
			color_range = true;
			subsampling_x = false;
			subsampling_y = false;
		} else {
			READ(color_range);
			if (seq_profile == 0) {
				subsampling_x = true;
				subsampling_y = true;
			} else if (seq_profile == 1) {
				subsampling_x = false;
				subsampling_y = false;
			} else {
				if (BitDepth == 12) {
					READ(subsampling_x);
					if (subsampling_x)
						READ(subsampling_y);
					else
						subsampling_y = false;
				} else {
					subsampling_x = true;
					subsampling_y = false;
				}
			}
			if (subsampling_x && subsampling_y) {
				READ_BITS(chroma_sample_position, 2);
			}
		}
		READ(separate_uv_delta_q);
		return true;


	}

	bool SequenceHeader::parseTimingInfo(BitReader& br)
	{
		bool timing_info_present_flag;
		READ(timing_info_present_flag);
		if (timing_info_present_flag) {
			READ(num_units_in_tick);
			READ(time_scale);
			READ(equal_picture_interval);
			ASSERT(0);
			if (equal_picture_interval) {

			}
		}
		return true;
		


	}

	bool SequenceHeader::parse(BitReader& br)
	{
		READ_BITS(seq_profile, 2);
		uint8_t operating_points_minus1_cnt;
		READ_BITS(operating_points_minus1_cnt, 5);
		operating_points.resize(operating_points_minus1_cnt + 1);
		for (int i = 0; i <= operating_points_minus1_cnt; i++) {
			OperatingPoint& p = operating_points[i];
			READ_BITS(p.operating_point_idc, 12);
			READ_BITS(p.level, 4);
			READ(p.decoder_rate_model_param_present_flag);
			if (p.decoder_rate_model_param_present_flag) {
				READ_BITS(p.decode_to_display_rate_ratio, 12);
				READ_BITS(p.initial_display_delay, 24);
				READ_BITS(p.extra_frame_buffers, 4);
			}

		}

		READ_BITS(frame_width_bits_minus_1, 4);
		READ_BITS(frame_height_bits_minus_1, 4);
		READ_BITS(max_frame_width_minus_1, frame_width_bits_minus_1 + 1);
		READ_BITS(max_frame_height_minus_1, frame_height_bits_minus_1 + 1);
		READ(frame_id_numbers_present_flag);
		if (frame_id_numbers_present_flag) {
			READ_BITS(delta_frame_id_length_minus2, 4);
			READ_BITS(additional_frame_id_length_minus1, 3);
		}

		READ(use_128x128_superblock);
		READ(enable_filter_intra);
		READ(enable_intra_edge_filter);
		READ(enable_interintra_compound);
		READ(enable_masked_compound);
		READ(enable_warped_motion);
		READ(enable_dual_filter);

		bool enable_order_hint;
		READ(enable_order_hint);

		if (enable_order_hint) {
			READ(enable_jnt_comp);
			READ(enable_ref_frame_mvs);
		}
		else {
			enable_jnt_comp = false;
			enable_ref_frame_mvs = false;
		}
		bool seq_choose_screen_content_tools;
		READ(seq_choose_screen_content_tools);
		if (seq_choose_screen_content_tools) {
			seq_force_screen_content_tools = SELECT_SCREEN_CONTENT_TOOLS;
		} else {
			READ_BITS(seq_force_screen_content_tools, 1);
		}
		if (seq_force_screen_content_tools > 0) {
			bool seq_choose_integer_mv;
			READ(seq_choose_integer_mv);
			if (seq_choose_integer_mv) {
				seq_force_integer_mv = SELECT_INTEGER_MV;
			} else {
				READ_BITS(seq_force_integer_mv, 1);
			}
		} else {
			seq_force_integer_mv = SELECT_INTEGER_MV;
		}

		if (enable_order_hint) {
			uint8_t order_hint_bits_minus1;
			READ_BITS(order_hint_bits_minus1, 3);
			OrderHintBits = order_hint_bits_minus1 + 1;
		} else {
			OrderHintBits = 0;
		}
		READ(enable_superres);
		READ(enable_cdef);
		READ(enable_restoration);
		if (!parseColorConfig(br))
			return false;
		if (!parseTimingInfo(br))
			return false;
		READ(film_grain_params_present);
		return true;
	}
	Parser::Parser()
		: m_seenFrameHeader(false)
	{

	}


	bool Parser::parse(uint8_t* data, size_t size)

	{
		BitReader reader(data, size);
		while (reader.getRemainingBitsCount() > 0) {
			obu_header header;
			if (!header.parse(reader))
				return false;
			bool ret;
			uint8_t type = header.obu_type;
			uint64_t sz = header.obu_size;
			printf("type = %s, extension = %d, obu_size = %d\r\n", obuType2String(type), header.obu_extension_flag, (int)sz);

			BitReader br(data + (reader.getPos() >> 3), sz);
			if (type == OBU_SEQUENCE_HEADER) {
				ret = parseSequenceHeader(br);
			} else if (type == OBU_TD) {
				ret = parseTemporalDelimiter(br);
			} else if (type == OBU_FRAME_HEADER) {
				ret = parseFrameHeader(br);
			} else if (type == OBU_TILE_GROUP) {
				ret = praseTileGroup(br);
			} else if (type == OBU_METADATA) {
				ret = parseMetadata(br);
			} else if (type == OBU_PADDING) {
				ret = parsePadding(br);
			} else if (type == OBU_FRAME) {
				ret = parseFrame(br);
			}
			else {
				ret = praseReserved(br);
			}
			if (!ret) {
				return false;
			}
			reader.skip(sz<<3);
		}
		return true;

	}
	bool Parser::parseSequenceHeader(BitReader& br)
	{
		return m_sequence.parse(br);
	}
	bool Parser::parseTemporalDelimiter(BitReader&)
	{
		m_seenFrameHeader = false;
		
		return true;
	}
	bool Parser::parseFrameHeader(BitReader& br)
	{
		if (m_seenFrameHeader) {
			ASSERT(0);
		} else {
			m_seenFrameHeader = true;
			if (!m_frameHeader.parse(br, m_sequence))
				return false;
		}
		return true;
	}
	bool Parser::praseTileGroup(BitReader& br)
	{
		return true;
	}
	bool Parser::parseMetadata(BitReader& br)
	{
		return true;
	}
	bool Parser::parsePadding(BitReader& br)
	{
		return true;
	}

	FrameHeader::FrameHeader()
	{
		memset(this, 0, sizeof(*this));		
	}

	bool FrameHeader::parse(BitReader& br, const SequenceHeader& sequence)
	{

		uint32_t idLen = (sequence.additional_frame_id_length_minus1 + sequence.delta_frame_id_length_minus2 + 3);
		READ(show_existing_frame);
		if (show_existing_frame == 1) {
			READ_BITS(frame_to_show_map_idx, 3);
			refresh_frame_flags = 0;
			/*if (sequence.frame_id_numbers_present_flag) {
				READ_BITS(display_frame_id, idLen);
			}*/
			ASSERT(0);
			return true;
		}
		READ_BITS(frame_type, 2);
		FrameIsIntra = (frame_type == INTRA_ONLY_FRAME || frame_type == KEY_FRAME);
		READ(show_frame);
		if (show_frame) {
			showable_frame = false;
		} else {
			READ(showable_frame);
		}
		if (frame_type == KEY_FRAME && show_frame) {
			memset(RefValid, 0, sizeof(RefValid));
		}
		if (frame_type == SWITCH_FRAME) {
			error_resilient_mode = true;
		} else {
			READ(error_resilient_mode);
		}
		READ(disable_cdf_update);
		if (sequence.seq_force_screen_content_tools == SELECT_SCREEN_CONTENT_TOOLS) {
			READ(allow_screen_content_tools);
		} else {
			allow_screen_content_tools = (bool)sequence.seq_force_screen_content_tools;
		}

		if (allow_screen_content_tools) {
			if (sequence.seq_force_integer_mv == SELECT_INTEGER_MV) {
				READ(force_integer_mv);
			} else {
				force_integer_mv = (bool)sequence.seq_force_integer_mv;
			}
		} else {
			force_integer_mv = false;
		}
		if (sequence.frame_id_numbers_present_flag) {
			PrevFrameID = current_frame_id;
			READ_BITS(current_frame_id, idLen);
			mark_ref_frames(sequence, idLen);
		}
		if (frame_type == SWITCH_FRAME) {
			frame_size_override_flag = true;
		} else {
			READ(frame_size_override_flag);
		}
		READ(order_hint);
		if (FrameIsIntra || error_resilient_mode) {
			primary_ref_frame = PRIMARY_REF_NONE;
		} else {
			READ_BITS(primary_ref_frame, 3);
		}
		allow_high_precision_mv = false;
		use_ref_frame_mvs = false;
		allow_intrabc = false;
		if (frame_type == KEY_FRAME) {
			if (show_frame) {
				refresh_frame_flags = (1 << NUM_REF_FRAMES) - 1;
			}
			else {
				READ(refresh_frame_flags);
			}
			if (!parseFrameSize(br, sequence))
				return false;
			if (!parseRenderSize(br))
				return false;
			if (allow_screen_content_tools && UpscaledWidth == FrameWidth) {
				READ(allow_intrabc);
			}
	
		} else {

		}
		return true;

	}
	void FrameHeader::mark_ref_frames(const SequenceHeader& sequence, uint8_t idLen)
	{
		uint8_t diffLen = sequence.delta_frame_id_length_minus2 + 2;
		for (int i = 0; i < NUM_REF_FRAMES; i++) {
			if (current_frame_id > (1 << diffLen)) {
				if (RefFrameId[i] > current_frame_id ||
					RefFrameId[i] < (current_frame_id - (1 << diffLen)))
					RefValid[i] = false;
			} else {
				if (RefFrameId[i] > current_frame_id ||
					RefFrameId[i] < ((1 << idLen) +
						current_frame_id -
						(1 << diffLen)))
					RefValid[i] = false;
			}
		}

	}
	bool FrameHeader::parseFrameSize(BitReader& br, const SequenceHeader& sequence)
	{
		if (frame_size_override_flag) {
			uint32_t frame_width_minus_1, frame_height_minus_1;
			READ_BITS(frame_width_minus_1, sequence.frame_width_bits_minus_1 + 1);
			READ_BITS(frame_height_minus_1, sequence.frame_height_bits_minus_1 + 1);
			FrameWidth = frame_width_minus_1 + 1;
			FrameHeight = frame_height_minus_1 + 1;
		} else{
			FrameWidth = sequence.max_frame_width_minus_1 + 1;
			FrameHeight = sequence.max_frame_height_minus_1 + 1;
		}
		MiCols = 2 * ((FrameWidth + 7) >> 3);
		MiRows = 2 * ((FrameHeight + 7) >> 3);
		return parseSuperresParams(br, sequence);
	}
	bool FrameHeader::parseSuperresParams(BitReader& br, const SequenceHeader& sequence)
	{

		if (sequence.enable_superres) {
			READ(use_superres);
		} else {
			use_superres = false;
		}
		if (use_superres) {
			uint8_t coded_denom;
			READ_BITS(coded_denom, SUPERRES_DENOM_BITS);
			SuperresDenom = coded_denom + SUPERRES_DENOM_MIN;
		} else {
			SuperresDenom = SUPERRES_NUM;
		}
		UpscaledWidth = FrameWidth;
		FrameWidth = (UpscaledWidth * SUPERRES_NUM +
			(SuperresDenom / 2)) / SuperresDenom;
		return true;
	}

	bool FrameHeader::parseRenderSize(BitReader& br)
	{
		bool render_and_frame_size_different;
		READ(render_and_frame_size_different);
		if (render_and_frame_size_different) {
			uint16_t render_width_minus_1, render_height_minus_1;
			READ(render_width_minus_1);
			READ(render_height_minus_1);
			RenderWidth = render_width_minus_1 + 1;
			RenderHeight = render_height_minus_1 + 1;
		} else {
			RenderWidth = UpscaledWidth;
			RenderHeight = FrameHeight;
		}
		return true;
	}

	bool Parser::parseFrame(BitReader& br)
	{
		if (!parseFrameHeader(br))
			return false;
		skipTrailingBits(br);

		return true;

	}
	bool Parser::praseReserved(BitReader& br)
	{
		return true;
	}
	void Parser::skipTrailingBits(BitReader& br)
	{
		while (br.getPos() & 7)
			br.skip(1);
		return;
	}

}
}