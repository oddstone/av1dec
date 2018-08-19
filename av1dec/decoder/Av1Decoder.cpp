#include "Av1Decoder.h"
#include "codecparsers/bitReader.h"
#include "codecparsers/Av1Parser.h"


namespace YamiParser {
	namespace Av1 {
		Decoder::Decoder()
		{
			m_parser.reset(new Parser);
		}


		Decoder::~Decoder()
		{
		}


		bool Decoder::decode(uint8_t* data, size_t size)
		{
			BitReader reader(data, size);
			while (reader.getRemainingBitsCount() > 0) {
				obu_header header;
				if (!header.parse(reader))
					return false;
				bool ret;
				ObuType type = header.obu_type;
				uint64_t sz = header.obu_size;
				printf("type = %s, extension = %d, obu_size = %d\r\n", obuType2String(type), header.obu_extension_flag, (int)sz);

				BitReader br(data + (reader.getPos() >> 3), sz);
				if (type == OBU_SEQUENCE_HEADER) {
					ret = m_parser->parseSequenceHeader(br);
				} else if (type == OBU_TD) {
					ret = m_parser->parseTemporalDelimiter(br);
				} else if (type == OBU_FRAME_HEADER) {
					ret = m_parser->parseFrameHeader(br);
				} else if (type == OBU_TILE_GROUP) {
					TileGroupPtr group;
					ret = m_parser->parseTileGroup(br, group);
				} else if (type == OBU_METADATA) {
					ret = m_parser->parseMetadata(br);
				} else if (type == OBU_PADDING) {
					ret = m_parser->parsePadding(br);
				} else if (type == OBU_FRAME) {
					TileGroupPtr group;
					ret = m_parser->parseFrame(br, group);
				} else {
					ret = m_parser->praseReserved(br);
				}
				if (!ret) {
					return false;
				}
				reader.skip(sz << 3);
			}
			return true;

			

		}
	};
};
