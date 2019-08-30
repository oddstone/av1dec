#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <algorithm>
#include <string.h>

#include "Av1Parser.h"
#include "Av1Common.h"
#include "SymbolDecoder.h"
#include "log.h"

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

#define READ_NS(f, n)                                   \
    do {                                                \
        if (!br.readNs(f, n)) {                         \
            ERROR("failed to readNs(%d) to %s", n, #f); \
            return false;                               \
        }                                               \
    } while (0)

#define READ_DELTA_Q(f)                                  \
    do {                                                 \
        bool delta_coded;                                \
        if (!br.readT(delta_coded)) {                    \
            ERROR("read delta_coded failed for %s", #f); \
            return false;                                \
        }                                                \
        if (delta_coded) {                               \
            if (!br.readSu(f, 7)) {                      \
                ERROR("read %s failed", #f);             \
                return false;                            \
            }                                            \
        } else {                                         \
            f = 0;                                       \
        }                                                \
    } while (0)

#define READ_SU(f, n)                                   \
    do {                                                \
        if (!br.readSu(f, n)) {                         \
            ERROR("failed to readSu(%d) to %s", n, #f); \
            return false;                               \
        }                                               \
    } while (0)

#define READ_LE(f, n)                                   \
    do {                                                \
        if (!br.readLe(f, n)) {                         \
            ERROR("failed to readLe(%d) to %s", n, #f); \
            return false;                               \
        }                                               \
    } while (0)

#define CHECK_EQ(f, v)                           \
    do {                                         \
        if (f != v) {                            \
            ERROR("%s not equals to %d", #f, v); \
            return false;                        \
        }                                        \
    } while (0)
#ifndef ROUND2
#define ROUND2(x, n) ((n == 0) ? x : ((x + (1 << (n - 1))) >> n))
#endif

namespace YamiParser {
namespace Av1 {
    bool obu_extension_header::parse(BitReader& br)
    {
        READ_BITS(temporal_id, 3);
        READ_BITS(spatial_id, 2);
        READ_BITS(quality_id, 2);

        bool reserved_flag;
        READ(reserved_flag);
        return true;
    }

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

    bool obu_header::parse(BitReader& br)
    {
        memset(this, 0, sizeof(*this));
        bool obu_forbidden_bit;
        READ(obu_forbidden_bit);
        CHECK_EQ(obu_forbidden_bit, 0);

        uint8_t type;
        READ_BITS(type, 4);
        obu_type = (ObuType)type;
        ///READ_BITS(obu_type, 4);

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

        } else {
            obu_size = br.getRemainingBitsCount() >> 3;
        }
        uint64_t left = br.getRemainingBitsCount() >> 3;
        if (obu_size > left) {
            ERROR("obu_size(%d) > left (%d)", (int)obu_size, (int)left);
            return false;
        }

        return true;
    }

    bool obu_header::parseLeb128(BitReader& br, uint64_t& v)
    {
        const int kMaxSize = 8;
        v = 0;
        uint8_t leb128_byte;
        for (int i = 0; i < kMaxSize; i++) {
            READ(leb128_byte);
            v |= ((leb128_byte & 0x7f) << (i * 7));
            if (!(leb128_byte & 0x80)) {
                break;
            }
        }
        return true;
    }

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
        NumPlanes = mono_chrome ? 1 : 3;
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
        if ((color_primaries == CP_BT_709 && transfer_characteristics == TC_SRGB && matrix_coefficients == MC_IDENTITY)) {
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
        READ_BITS(seq_profile, 3);
        READ(still_picture);
        READ(reduced_still_picture_header);
        if (reduced_still_picture_header) {
            timing_info_present_flag = false;
            decoder_model_info_present_flag = false;
            initial_display_delay_present_flag = false;
            operating_points.resize(1);
            OperatingPoint& p = operating_points[0];
            READ_BITS(p.seq_level_idx, 5);
            p.seq_tier = false;
            p.decoder_model_present_for_this_op = false;
            p.initial_display_delay_present_for_this_op = false;
        } else {
            READ(timing_info_present_flag);
            if (timing_info_present_flag) {
                parseTimingInfo(br);
                READ(decoder_model_info_present_flag);
                if (decoder_model_info_present_flag) {
                    ASSERT(0 && "decoder_model_info");
                }
            }
            READ(initial_display_delay_present_flag);
            uint8_t operating_points_minus1_cnt;
            READ_BITS(operating_points_minus1_cnt, 5);
            operating_points.resize(operating_points_minus1_cnt + 1);
            for (int i = 0; i <= operating_points_minus1_cnt; i++) {
                OperatingPoint& p = operating_points[i];
                READ_BITS(p.operating_point_idc, 12);
                READ_BITS(p.seq_level_idx, 5);
                if (p.seq_level_idx > 7)
                    READ(p.seq_tier);
                if (decoder_model_info_present_flag) {
                    READ(p.decoder_model_present_for_this_op);
                    if (p.decoder_model_present_for_this_op) {
                        ASSERT(0 && "decoder_model_present_for_this_op");
                    }
                    if (initial_display_delay_present_flag) {
                        ASSERT(0 && "initial_display_delay_present_flag");
                    }
                }
            }
        }
        READ_BITS(frame_width_bits_minus_1, 4);
        READ_BITS(frame_height_bits_minus_1, 4);
        READ_BITS(max_frame_width_minus_1, frame_width_bits_minus_1 + 1);
        READ_BITS(max_frame_height_minus_1, frame_height_bits_minus_1 + 1);
        if (reduced_still_picture_header)
            frame_id_numbers_present_flag = false;
        else
            READ(frame_id_numbers_present_flag);
        if (frame_id_numbers_present_flag) {
            READ_BITS(delta_frame_id_length_minus2, 4);
            READ_BITS(additional_frame_id_length_minus1, 3);
        }

        READ(use_128x128_superblock);
        READ(enable_filter_intra);
        READ(enable_intra_edge_filter);
        if (reduced_still_picture_header) {
            enable_interintra_compound = false;
            enable_interintra_compound = false;
            enable_masked_compound = false;
            enable_warped_motion = false;
            enable_dual_filter = false;
            enable_order_hint = false;
            enable_jnt_comp = false;
            enable_ref_frame_mvs = false;
            seq_force_screen_content_tools = SELECT_SCREEN_CONTENT_TOOLS;
            seq_force_integer_mv = SELECT_INTEGER_MV;
            OrderHintBits = 0;
        } else {
            READ(enable_interintra_compound);
            READ(enable_masked_compound);
            READ(enable_warped_motion);
            READ(enable_dual_filter);

            READ(enable_order_hint);

            if (enable_order_hint) {
                READ(enable_jnt_comp);
                READ(enable_ref_frame_mvs);
            } else {
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
        }
        READ(enable_superres);
        READ(enable_cdef);
        READ(enable_restoration);
        if (!parseColorConfig(br))
            return false;
        READ(film_grain_params_present);
        return true;
    }

    Parser::Parser()
        : m_seenFrameHeader(false)
    {
    }

    bool Parser::parseSequenceHeader(BitReader& br)
    {
        m_sequence = std::make_shared<SequenceHeader>();
        return m_sequence->parse(br);
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
            m_frame.reset(new FrameHeader);
            if (!m_frame->parse(br, *m_sequence))
                return false;
            /* if (show_existing_frame) */
        }
        return true;
    }
    bool Parser::parseTileGroup(BitReader& br, TileGroup& group)
    {
        bool tile_start_and_end_present_flag = false;
        ;
        if (m_frame->NumTiles > 1)
            READ(tile_start_and_end_present_flag);
        uint32_t tg_start, tg_end;
        if (m_frame->NumTiles == 1 || !tile_start_and_end_present_flag) {
            tg_start = 0;
            tg_end = m_frame->NumTiles - 1;
        } else {
            ASSERT(0);
        }
        skipTrailingBits(br);

        //group.reset(new TileGroup);

        for (uint32_t TileNum = tg_start; TileNum <= tg_end; TileNum++) {

            bool lastTile = (TileNum == tg_end);
            uint32_t tileSize;
            if (lastTile) {
                tileSize = br.getRemainingBitsCount() / 8;
            } else {
                uint32_t tile_size_minus_1;
                READ_LE(tile_size_minus_1, m_frame->TileSizeBytes);
                tileSize = tile_size_minus_1 + 1;
            }
            std::shared_ptr<Tile> tile(new Tile(m_sequence, m_frame, TileNum));
            if (!tile->parse(br.getCurrent(), br.getRemainingBitsCount() >> 3)) {
                ERROR("decode tile failed");
                return false;
            }
            group.push_back(tile);
        }
        //br.
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
        memset(this, 0, offsetof(FrameHeader, MiColStarts));
    }

    void FrameHeader::initGeometry()
    {
        
        YModes.assign(AlignedMiRows, std::vector<PREDICTION_MODE>(AlignedMiCols));
        UVModes.assign(AlignedMiRows, std::vector<UV_PREDICTION_MODE>(AlignedMiCols));
        TxTypes.assign(AlignedMiRows, std::vector<TX_TYPE>(AlignedMiCols));
        IsInters.assign(AlignedMiRows, std::vector<bool>(AlignedMiCols));
        InterTxSizes.assign(AlignedMiRows, std::vector<TX_SIZE>(AlignedMiCols));
        TxSizes.assign(AlignedMiRows, std::vector<TX_SIZE>(AlignedMiCols));
        MiSizes.assign(AlignedMiRows, std::vector<BLOCK_SIZE>(AlignedMiCols));
        SegmentIds.assign(AlignedMiRows, std::vector<uint8_t>(AlignedMiCols));
        Skips.assign(AlignedMiRows, std::vector<bool>(AlignedMiCols));
    }

    bool FrameHeader::loop_filter_params(BitReader& br, const SequenceHeader& seq)
    {
        return m_loopFilter.parse(br, seq, *this);
    }

    bool FrameHeader::cdef_params(BitReader& br, const SequenceHeader& seq)
    {
        return m_cdef.parse(br, seq, *this);
    }

    bool FrameHeader::lr_params(BitReader& br, const SequenceHeader& seq)
    {
        return m_loopRestoration.parse(br, seq, *this);
    }

    bool FrameHeader::parse(BitReader& br, const SequenceHeader& sequence)
    {
        const static uint8_t allFrames = (1 << NUM_REF_FRAMES) - 1;
        uint32_t idLen = 0;
        if (sequence.frame_id_numbers_present_flag)
            idLen = (sequence.additional_frame_id_length_minus1 + sequence.delta_frame_id_length_minus2 + 3);

        if (sequence.reduced_still_picture_header) {
            show_existing_frame = false;
            frame_type = KEY_FRAME;
            FrameIsIntra = true;
            show_frame = true;
            showable_frame = false;
        } else {
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
        }
        if (frame_type == KEY_FRAME && show_frame) {
            for (int i = 0; i < NUM_REF_FRAMES; i++) {
                RefValid[i] = false;
                RefOrderHint[i] = 0;
            }
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
        if (FrameIsIntra)
            force_integer_mv = true;

        if (sequence.frame_id_numbers_present_flag) {
            PrevFrameID = current_frame_id;
            READ_BITS(current_frame_id, idLen);
            mark_ref_frames(sequence, idLen);
        }
        if (frame_type == SWITCH_FRAME) {
            frame_size_override_flag = true;
        } else if (sequence.reduced_still_picture_header) {
            frame_size_override_flag = false;
        } else {
            READ(frame_size_override_flag);
        }
        READ_BITS(order_hint, sequence.OrderHintBits);
        if (FrameIsIntra || error_resilient_mode) {
            primary_ref_frame = PRIMARY_REF_NONE;
        } else {
            READ_BITS(primary_ref_frame, 3);
        }
        if (sequence.decoder_model_info_present_flag) {
            ASSERT(0 && "decoder_model_info_present_flag");
        }
        allow_high_precision_mv = false;
        use_ref_frame_mvs = false;
        allow_intrabc = false;
        if (frame_type == SWITCH_FRAME || (frame_type == KEY_FRAME && show_frame)) {
            refresh_frame_flags = allFrames;
        } else {
            READ(refresh_frame_flags);
        }
        if (FrameIsIntra) {
            if (!parseFrameSize(br, sequence))
                return false;
            if (!parseRenderSize(br))
                return false;
            if (allow_screen_content_tools && UpscaledWidth == FrameWidth) {
                READ(allow_intrabc);
            }
        } else {
            ASSERT(0);
        }
        if (sequence.reduced_still_picture_header || disable_cdf_update) {
            disable_frame_end_update_cdf = true;
        } else {
            READ(disable_frame_end_update_cdf);
        }
        if (primary_ref_frame == PRIMARY_REF_NONE) {
            //init_non_coeff_cdfs()
            //setup_past_independence()
        } else {
            ASSERT(0);
        }
        if (use_ref_frame_mvs)
            ASSERT(0);
        if (!parseTileInfo(br, sequence))
            return false;
        if (!m_quant.parse(br, sequence)) {
            return false;
        }
        if (!m_segmentation.parse(br))
            return false;
        if (!m_deltaQ.parse(br, m_quant))
            return false;
        if (!m_deltaLf.parse(br, m_deltaQ))
            return false;
        if (primary_ref_frame == PRIMARY_REF_NONE) {
            //init_coeff_cdfs()
        } else {
            //load_previous_segment_ids()
        }
        {
            CodedLossless = true;
            for (int segmentId = 0; segmentId < MAX_SEGMENTS; segmentId++) {
                int16_t qindex = get_qindex(1, segmentId);
                LosslessArray[segmentId] = qindex == 0 && m_quant.DeltaQYDc == 0 && m_quant.DeltaQUAc == 0 && m_quant.DeltaQUDc == 0 && m_quant.DeltaQVAc == 0 && m_quant.DeltaQVDc == 0;
                if (!LosslessArray[segmentId])
                    CodedLossless = 0;
                if (m_quant.using_qmatrix) {
                    if (LosslessArray[segmentId]) {
                        SegQMLevel[0][segmentId] = 15;
                        SegQMLevel[1][segmentId] = 15;
                        SegQMLevel[2][segmentId] = 15;
                    } else {
                        SegQMLevel[0][segmentId] = m_quant.qm_y;
                        SegQMLevel[1][segmentId] = m_quant.qm_u;
                        SegQMLevel[2][segmentId] = m_quant.qm_v;
                    }
                }
            }
            AllLossless = CodedLossless && (FrameWidth == UpscaledWidth);
        }
        if (!loop_filter_params(br, sequence))
            return false;
        if (!cdef_params(br, sequence))
            return false;
        if (!lr_params(br, sequence))
            return false;
        if (!read_tx_mode(br))
            return false;

        /* frame_reference_mode() */
        /* skip_mode_params() */
        if (FrameIsIntra || error_resilient_mode || !enable_warped_motion)
            allow_warped_motion = false;
        else
            READ(allow_warped_motion);
        READ(reduced_tx_set);
        /* global_motion_params() */
        if (show_frame || showable_frame) {
            /* film_grain_params()*/
        }
        initGeometry();
        return true;
    }
    void FrameHeader::mark_ref_frames(const SequenceHeader& sequence, uint8_t idLen)
    {
        uint8_t diffLen = sequence.delta_frame_id_length_minus2 + 2;
        for (int i = 0; i < NUM_REF_FRAMES; i++) {
            if (current_frame_id > (1 << diffLen)) {
                if (RefFrameId[i] > current_frame_id || RefFrameId[i] < (current_frame_id - (1 << diffLen)))
                    RefValid[i] = false;
            } else {
                if (RefFrameId[i] > current_frame_id || RefFrameId[i] < ((1 << idLen) + current_frame_id - (1 << diffLen)))
                    RefValid[i] = false;
            }
        }
    }

#define ROOF(b, a) ((b + (a -1)) & ~(a-1))
    bool FrameHeader::parseFrameSize(BitReader& br, const SequenceHeader& sequence)
    {
        if (frame_size_override_flag) {
            uint32_t frame_width_minus_1, frame_height_minus_1;
            READ_BITS(frame_width_minus_1, sequence.frame_width_bits_minus_1 + 1);
            READ_BITS(frame_height_minus_1, sequence.frame_height_bits_minus_1 + 1);
            FrameWidth = frame_width_minus_1 + 1;
            FrameHeight = frame_height_minus_1 + 1;
        } else {
            FrameWidth = sequence.max_frame_width_minus_1 + 1;
            FrameHeight = sequence.max_frame_height_minus_1 + 1;
        }
        MiCols = 2 * ((FrameWidth + 7) >> 3);
        MiRows = 2 * ((FrameHeight + 7) >> 3);
        uint32_t align = sequence.use_128x128_superblock ? 128 : 64;
        AlignedMiCols = ROOF(FrameWidth, align) >> 2;
        AlignedMiRows = ROOF(FrameHeight, align) >> 2;
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
        FrameWidth = (UpscaledWidth * SUPERRES_NUM + (SuperresDenom / 2)) / SuperresDenom;
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

    uint32_t tile_log2(uint32_t blkSize, uint32_t target)
    {
        uint32_t k;
        for (k = 0; (blkSize << k) < target; k++) {
        }
        return k;
    }

    bool parseTileLog2(BitReader& br, uint32_t& tileLog2, uint32_t min, uint32_t max)
    {
        tileLog2 = min;
        bool increment;
        while (tileLog2 < max) {
            READ(increment);
            if (increment)
                tileLog2++;
            else
                break;
        }
        return true;
    }

    void getMiStarts(std::vector<uint32_t>& starts, uint32_t sbMax, uint32_t sbShift, uint32_t tileLog2)
    {
        starts.clear();
        uint32_t step = (sbMax + (1 << tileLog2)) >> tileLog2;
        for (uint32_t start = 0; start < sbMax; start += step) {
            starts.push_back(start << sbShift);
        }
        starts.push_back(sbMax<<sbShift);
    }

    bool FrameHeader::parseTileStarts(BitReader& br, std::vector<uint32_t>& starts, uint32_t sbMax, uint32_t sbShift, uint32_t maxTileSb)
    {
        starts.clear();
        uint32_t widestTileSb = 0;
        uint32_t startSb = 0;
        for (uint32_t i = 0; startSb < sbMax && i < MAX_TILE_COLS; i++) {
            starts.push_back(startSb << sbShift);
            uint32_t n = std::min(sbMax - startSb, maxTileSb);
            uint32_t size_minus_1;
            READ_NS(size_minus_1, n);
            uint32_t sizeSb = size_minus_1 + 1;
            widestTileSb = std::max(sizeSb, widestTileSb);
            startSb += sizeSb;
        }
        starts.push_back(sbMax);
        return true;
    }

    bool FrameHeader::read_tx_mode(BitReader& br)
    {
        if (CodedLossless) {
            TxMode = ONLY_4X4;
        } else {
            bool tx_mode_select;
            READ(tx_mode_select);
            if (tx_mode_select) {
                TxMode = TX_MODE_SELECT;
            } else {
                TxMode = TX_MODE_LARGEST;
            }
        }
        return true;
    }

    bool FrameHeader::parseTileInfo(BitReader& br, const SequenceHeader& sequence)
    {
        uint32_t sbCols = sequence.use_128x128_superblock ? ((MiCols + 31) >> 5) : ((MiCols + 15) >> 4);
        uint32_t sbRows = sequence.use_128x128_superblock ? ((MiRows + 31) >> 5) : ((MiRows + 15) >> 4);
        uint8_t sbShift = sequence.use_128x128_superblock ? 5 : 4;
        uint8_t sbSize = sbShift + 2;
        uint32_t maxTileWidthSb = MAX_TILE_WIDTH >> sbSize;
        uint32_t maxTileAreaSb = MAX_TILE_AREA >> (2 * sbSize);
        uint32_t minLog2TileCols = tile_log2(maxTileWidthSb, sbCols);
        uint32_t maxLog2TileCols = tile_log2(1, std::min(sbCols, MAX_TILE_COLS));
        uint32_t maxLog2TileRows = tile_log2(1, std::min(sbRows, MAX_TILE_ROWS));
        uint32_t minLog2Tiles = std::max(minLog2TileCols,
            tile_log2(maxTileAreaSb, sbRows * sbCols));

        bool uniform_tile_spacing_flag;
        READ(uniform_tile_spacing_flag);
        if (uniform_tile_spacing_flag) {
            if (!parseTileLog2(br, TileColsLog2, minLog2TileCols, maxLog2TileCols)) {
                return false;
            }
            getMiStarts(MiColStarts, sbCols, sbShift, TileColsLog2);

            uint32_t minLog2TileRows = std::max(minLog2Tiles - TileColsLog2, (uint32_t)0);
            uint32_t maxTileHeightSb = sbRows >> minLog2TileRows;

            if (!parseTileLog2(br, TileRowsLog2, minLog2TileRows, maxLog2TileRows)) {
                return false;
            }
            getMiStarts(MiRowStarts, sbRows, sbShift, TileRowsLog2);
            TileCols = 1 << TileColsLog2;
            TileRows = 1 << TileRowsLog2;

        } else {
            uint32_t widestTileSb = 0;
            uint32_t startSb = 0;
            uint32_t i;
            for (i = 0; startSb < sbCols && i < MAX_TILE_COLS; i++) {
                MiColStarts.push_back(startSb << sbShift);
                uint32_t maxWidth = std::min(sbCols - startSb, maxTileWidthSb);
                uint32_t width_in_sbs_minus_1;
                READ_NS(width_in_sbs_minus_1, maxWidth);
                uint32_t sizeSb = width_in_sbs_minus_1 + 1;
                widestTileSb = std::max(sizeSb, widestTileSb);
                startSb += sizeSb;
            }
            MiColStarts.push_back(MiCols);
            TileCols = i;
            TileColsLog2 = tile_log2(1, TileCols);
            if (minLog2Tiles > 0)
                maxTileAreaSb = (sbRows * sbCols) >> (minLog2Tiles + 1);
            else
                maxTileAreaSb = sbRows * sbCols;
            uint32_t one = 1;
            uint32_t maxTileHeightSb = std::max(maxTileAreaSb / widestTileSb, one);
            startSb = 0;
            for (i = 0; startSb < sbRows && i < MAX_TILE_ROWS; i++) {
                MiRowStarts.push_back(startSb << sbShift);
                uint32_t maxHeight = std::min(sbRows - startSb, maxTileHeightSb);
                uint32_t height_in_sbs_minus_1;
                READ_NS(height_in_sbs_minus_1, maxHeight);
                uint8_t sizeSb = height_in_sbs_minus_1 + 1;
                startSb += sizeSb;
            }
            MiRowStarts.push_back(MiRows);
            TileRows = i;
            TileRowsLog2 = tile_log2(1, TileRows);
        }
        NumTiles = TileCols * TileRows;
        if (TileColsLog2 > 0 || TileRowsLog2 > 0) {
            uint8_t tile_size_bytes_minus_1;
            READ_BITS(tile_size_bytes_minus_1, 2);
            TileSizeBytes = tile_size_bytes_minus_1 + 1;
        }
        return true;
    }

    int16_t FrameHeader::get_qindex(bool ignoreDeltaQ, int segmentId) const
    {
        int16_t data = m_segmentation.FeatureData[segmentId][SEG_LVL_ALT_Q];
        int16_t qindex = m_quant.base_q_idx + data;
        if (!ignoreDeltaQ && m_deltaQ.delta_q_present) {
            ASSERT(0);
        }
        return CLIP3(0, 255, qindex);
    }

    bool Quantization::parse(BitReader& br, const SequenceHeader& seq)
    {
        READ(base_q_idx);
        READ_DELTA_Q(DeltaQYDc);
        if (seq.NumPlanes > 1) {
            bool diff_uv_delta;
            if (seq.separate_uv_delta_q) {
                READ(diff_uv_delta);
            } else {
                diff_uv_delta = false;
            }
            READ_DELTA_Q(DeltaQUDc);
            READ_DELTA_Q(DeltaQUAc);
            if (diff_uv_delta) {
                READ_DELTA_Q(DeltaQVDc);
                READ_DELTA_Q(DeltaQVAc);
            } else {
                DeltaQVDc = DeltaQUDc;
                DeltaQVAc = DeltaQUAc;
            }
        } else {
            DeltaQUDc = 0;
            DeltaQUDc = 0;
            DeltaQVAc = 0;
            DeltaQVAc = 0;
        }
        READ(using_qmatrix);
        if (using_qmatrix) {
            READ_BITS(qm_y, 4);
            READ_BITS(qm_u, 4);
            if (!seq.separate_uv_delta_q)
                qm_v = qm_u;
            else
                READ_BITS(qm_v, 4);
        }
        return true;
    }

    bool Parser::parseFrame(BitReader& br, TileGroup& group)
    {
        if (!parseFrameHeader(br))
            return false;
        skipTrailingBits(br);
        return parseTileGroup(br, group);
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

    bool Segmentation::parse(BitReader& br)
    {
        READ(segmentation_enabled);
        if (segmentation_enabled)
            ASSERT(0);
        else {
            for (int i = 0; i < MAX_SEGMENTS; i++) {
                for (int j = 0; j < SEG_LVL_MAX; j++) {
                    FeatureEnabled[i][j] = 0;
                    FeatureData[i][j] = 0;
                }
            }
        }
        SegIdPreSkip = false;
        LastActiveSegId = 0;
        for (int i = 0; i < MAX_SEGMENTS; i++) {
            for (int j = 0; j < SEG_LVL_MAX; j++) {
                if (FeatureEnabled[i][j]) {
                    LastActiveSegId = i;
                    if (j >= SEG_LVL_REF_FRAME) {
                        SegIdPreSkip = 1;
                    }
                }
            }
        }
        return true;
    }
    bool Segmentation::seg_feature_active_idx(int segmentId, SEG_LVL_FEATURE feature)
    {
        return segmentation_enabled && FeatureEnabled[segmentId][feature];
    }

    bool DeltaQ::parse(BitReader& br, const Quantization& quant)
    {
        delta_q_res = 0;
        delta_q_present = 0;
        if (quant.base_q_idx > 0) {
            READ(delta_q_present);
        }
        if (delta_q_present) {
            READ_BITS(delta_q_res, 2);
        }
        return true;
    }

    bool DeltaLf::parse(BitReader& br, const DeltaQ& deltaQ)
    {
        delta_lf_present = 0;
        delta_lf_res = 0;
        delta_lf_multi = 0;
        if (deltaQ.delta_q_present) {
            READ(delta_lf_present);
            if (delta_lf_present) {
                READ_BITS(delta_lf_res, 2);
                READ(delta_lf_multi);
            }
        }
        return true;
    }

    bool LoopFilter::parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame)
    {
        if (frame.CodedLossless || frame.allow_intrabc) {
            loop_filter_level[0] = 0;
            loop_filter_level[1] = 0;
            loop_filter_ref_deltas[INTRA_FRAME] = 1;
            loop_filter_ref_deltas[LAST_FRAME] = 0;
            loop_filter_ref_deltas[LAST2_FRAME] = 0;
            loop_filter_ref_deltas[LAST3_FRAME] = 0;
            loop_filter_ref_deltas[BWDREF_FRAME] = 0;
            loop_filter_ref_deltas[GOLDEN_FRAME] = -1;
            loop_filter_ref_deltas[ALTREF_FRAME] = -1;
            loop_filter_ref_deltas[ALTREF2_FRAME] = -1;

            for (uint8_t i = 0; i < 2; i++) {
                loop_filter_mode_deltas[i] = 0;
            }
            return true;
        }
        READ_BITS(loop_filter_level[0], 6);
        READ_BITS(loop_filter_level[1], 6);
        if (seq.NumPlanes > 1) {
            if (loop_filter_level[0] || loop_filter_level[1]) {
                READ_BITS(loop_filter_level[2], 6);
                READ_BITS(loop_filter_level[3], 6);
            }
        }
        READ_BITS(loop_filter_sharpness, 3);
        bool loop_filter_delta_enabled;
        READ(loop_filter_delta_enabled);
        if (loop_filter_delta_enabled) {
            bool loop_filter_delta_update;
            READ(loop_filter_delta_update);
            if (loop_filter_delta_update) {
                for (uint8_t i = 0; i < TOTAL_REFS_PER_FRAME; i++) {
                    bool update_ref_delta;
                    READ(update_ref_delta);
                    if (update_ref_delta)
                        READ_SU(loop_filter_ref_deltas[i], 7);
                }
                for (uint8_t i = 0; i < 2; i++) {
                    bool update_mode_delta;
                    READ(update_mode_delta);
                    if (update_mode_delta)
                        READ_SU(loop_filter_mode_deltas[i], 7);
                }
            }
        }

        return true;
    }

    bool Cdef::parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame)
    {
        cdef_idx.assign(frame.MiRows, std::vector<int>(frame.MiCols, -1));
        if (frame.CodedLossless || frame.allow_intrabc || !seq.enable_cdef) {
            cdef_bits = 0;
            cdef_y_pri_strength[0] = 0;
            cdef_y_sec_strength[0] = 0;
            cdef_uv_pri_strength[0] = 0;
            cdef_uv_sec_strength[0] = 0;
            CdefDamping = 3;
            return true;
        }
        uint8_t cdef_damping_minus_3;
        READ_BITS(cdef_damping_minus_3, 2);
        CdefDamping = cdef_damping_minus_3 + 3;
        READ_BITS(cdef_bits, 2);
        for (int i = 0; i < (1 << cdef_bits); i++) {
            READ_BITS(cdef_y_pri_strength[i], 4);
            READ_BITS(cdef_y_sec_strength[i], 2);
            if (cdef_y_sec_strength[i] == 3)
                cdef_y_sec_strength[i] += 1;
            if (seq.NumPlanes > 1) {
                READ_BITS(cdef_uv_pri_strength[i], 4);
                READ_BITS(cdef_uv_sec_strength[i], 2);
                if (cdef_uv_sec_strength[i] == 3)
                    cdef_uv_sec_strength[i] += 1;
            }
        }
        return true;
    }
    void Cdef::read_cdef(EntropyDecoder& entropy, uint32_t MiRow, uint32_t MiCol, uint32_t MiSize)
    {
        uint32_t cdefSize4 = Num_4x4_Blocks_Wide[ BLOCK_64X64 ];
        uint32_t cdefMask4 = ~(cdefSize4 - 1);
        int r = MiRow & cdefMask4;
        int c = MiCol & cdefMask4;
        if ( cdef_idx[ r ][ c ] == -1 ) {
            uint32_t idx = entropy.readLiteral(cdef_bits);
            int w4 = Num_4x4_Blocks_Wide[ MiSize ];
            int h4 = Num_4x4_Blocks_High[ MiSize ];
            for (int i = r; i < r + h4 ; i += cdefSize4 ) {
                for (int j = c; j < c + w4 ; j += cdefSize4 ) {
                    cdef_idx[ i ][ j ] = idx;
                }
            }
        }
    }
    const static RestorationType Remap_Lr_Type[4] = {
        RESTORE_NONE, RESTORE_SWITCHABLE, RESTORE_WIENER, RESTORE_SGRPROJ
    };
    const static int RESTORATION_TILESIZE_MAX = 256;

    static int count_units_in_frame(int unitSize, int frameSize) {
        return std::max((frameSize + (unitSize >> 1)) / unitSize, 1);
    }

    bool LoopRestoration::parse(BitReader& br, const SequenceHeader& seq, const FrameHeader& frame)
    {
        if (frame.AllLossless || frame.allow_intrabc ||
            !seq.enable_restoration ) {
            FrameRestorationType[0] = RESTORE_NONE;
            FrameRestorationType[1] = RESTORE_NONE;
            FrameRestorationType[2] = RESTORE_NONE;
            UsesLr = false;
            return true;
        }
        UsesLr = false;
        bool usesChromaLr = false;
        for (int i = 0; i < seq.NumPlanes; i++) {
            uint8_t lr_type;
            READ_BITS(lr_type, 2);
            FrameRestorationType[i] = Remap_Lr_Type[lr_type];
            if (FrameRestorationType[i] != RESTORE_NONE ) {
                UsesLr = true;
                if ( i > 0 ) {
                    usesChromaLr = true;
                }
            }
        }
        if (UsesLr) {
            uint8_t lr_unit_shift;
            if (seq.use_128x128_superblock ) {
                READ_BITS(lr_unit_shift, 1);
                lr_unit_shift++;
            } else {
                READ_BITS(lr_unit_shift, 1);
                if ( lr_unit_shift ) {
                    uint8_t lr_unit_extra_shift;
                    READ_BITS(lr_unit_extra_shift, 1);
                    lr_unit_shift += lr_unit_extra_shift;
                }
            }
            LoopRestorationSize[ 0 ] = RESTORATION_TILESIZE_MAX >> (2 - lr_unit_shift);
            uint8_t lr_uv_shift;
            if (seq.subsampling_x && seq.subsampling_y && usesChromaLr ) {
                READ_BITS(lr_uv_shift, 1);
            } else {
                lr_uv_shift = 0;
            }
            LoopRestorationSize[ 1 ] = LoopRestorationSize[ 0 ] >> lr_uv_shift;
            LoopRestorationSize[ 2 ] = LoopRestorationSize[ 0 ] >> lr_uv_shift;

            LrType.resize(seq.NumPlanes);
            LrWiener.resize(seq.NumPlanes);
            RefLrWiener.resize(seq.NumPlanes);
            for (int plane = 0; plane < seq.NumPlanes; plane++ ) {
                if ( FrameRestorationType[ plane ] != RESTORE_NONE ) {
                    int subX = (plane == 0) ? 0 : seq.subsampling_x;
                    int subY = (plane == 0) ? 0 : seq.subsampling_y;
                    int unitSize = LoopRestorationSize[ plane ];
                    unitRows = count_units_in_frame( unitSize, ROUND2(frame.FrameHeight, subY) );
                    unitCols = count_units_in_frame( unitSize, ROUND2(frame.UpscaledWidth, subX) );
                    LrType[plane].assign(unitRows, std::vector<RestorationType>(unitCols));
                    std::vector<std::vector<uint8_t>> v1(MAX_PASSES, std::vector<uint8_t>(MAX_WIENER_COEFFS));
                    std::vector<std::vector<std::vector<uint8_t>>> v2(unitCols, v1);
                    LrWiener[plane].assign(unitRows, v2);
                    RefLrWiener[plane].assign(MAX_PASSES, std::vector<uint8_t>(MAX_WIENER_COEFFS));
                }
            }
        }
        return true;
    }
    void LoopRestoration::read_lr(Tile& tile,
        int r, int c, BLOCK_SIZE bSize)
    {
        FrameHeader& frame = *tile.m_frame;
        SequenceHeader seq = *tile.m_sequence;
        if (frame.allow_intrabc) {
            return;
        }
        int w = Num_4x4_Blocks_Wide[bSize];
        int h = Num_4x4_Blocks_High[bSize];
        for (int plane = 0; plane < seq.NumPlanes; plane++ ) {
            if ( FrameRestorationType[ plane ] != RESTORE_NONE ) {
                int subX = (plane == 0) ? 0 : seq.subsampling_x;
                int subY = (plane == 0) ? 0 : seq.subsampling_y;
                int unitSize = LoopRestorationSize[ plane ];
                int unitRowStart = ( r * ( MI_SIZE >> subY) + unitSize - 1 ) / unitSize;
                int unitRowEnd = std::min( unitRows, ( (r + h) * ( MI_SIZE >> subY) + unitSize - 1 ) / unitSize);
                int numerator;
                int denominator;
                if (frame.use_superres ) {
                    numerator = (MI_SIZE >> subX) * frame.SuperresDenom;
                    denominator = unitSize * FrameHeader::SUPERRES_NUM;
                } else {
                    numerator = MI_SIZE >> subX;
                    denominator = unitSize;
                }
                int unitColStart = (c * numerator + denominator - 1 ) / denominator;
                int unitColEnd = std::min( unitCols, ( (c + w) * numerator + denominator - 1 ) / denominator);
                for (int unitRow = unitRowStart; unitRow < unitRowEnd; unitRow++ ) {
                    for (int unitCol = unitColStart; unitCol < unitColEnd; unitCol++ ) {
                        read_lr_unit(*tile.m_entropy, plane, unitRow, unitCol);
                    }
                }
            }
        }
    }

    static const int Wiener_Taps_Min[3] = { -5, -23, -17 };
    static const int Wiener_Taps_Max[3] = { 10, 8, 46 };
    static const int Wiener_Taps_K[3] = { 1, 2, 3 };
    static const int Wiener_Taps_Mid[3] = { 3, -7, 15 };
    void LoopRestoration::read_lr_unit(EntropyDecoder& entropy,
        int plane, int unitRow, int unitCol)
    {
        RestorationType restoration_type;
        if ( FrameRestorationType[ plane ] == RESTORE_WIENER ) {
            bool use_wiener = entropy.readUseWiener();
            restoration_type = use_wiener ? RESTORE_WIENER : RESTORE_NONE;
        } else if ( FrameRestorationType[ plane ] == RESTORE_SGRPROJ ) {
            ASSERT(0);
        } else {
            ASSERT(0);
        }
        LrType[plane][unitRow][unitCol] = restoration_type;
        int firstCoeff;
        if (restoration_type == RESTORE_WIENER ) {
            for (int pass = 0; pass < MAX_PASSES; pass++ ) {
                if ( plane ) {
                    firstCoeff = 1;
                    LrWiener[plane][unitRow][unitCol][pass][0] = 0;
                } else {
                    firstCoeff = 0;
                }
                for (int j = firstCoeff; j < MAX_WIENER_COEFFS; j++ ) {
                    int min = Wiener_Taps_Min[ j ];
                    int max = Wiener_Taps_Max[ j ];
                    int k = Wiener_Taps_K[ j ];
                    int v = entropy.decode_signed_subexp_with_ref_bool(min, max + 1, k, RefLrWiener[plane][pass][j]);
                    LrWiener[plane][unitRow][unitCol ][ pass ][ j ] = v;
                    RefLrWiener[ plane ][ pass ][ j ] = v;
                }
            }
        } else if ( restoration_type == RESTORE_SGRPROJ ) {
            ASSERT(0);
        } else {
            ASSERT(0);
        }

    }
    void LoopRestoration::resetRefs(int NumPlanes)
    {
        for (int plane = 0; plane < NumPlanes; plane++ ) {
            if (FrameRestorationType[plane] != RESTORE_NONE) {
                for (int pass = 0; pass < MAX_PASSES; pass++ ) {
                    //RefSgrXqd[ plane ][ pass ] = Sgrproj_Xqd_Mid[ pass ]
                    for (int i = 0; i < MAX_WIENER_COEFFS; i++ ) {
                        RefLrWiener[plane][pass][i] = Wiener_Taps_Mid[i];
                    }
                }
            }
        }
    }
}
}
