#include "pch.h"
#include <memory>
#include "../av1dec/codecparsers/SymbolDecoder.h"
#include "../av1dec/codecparsers/EntropyDecoder.h"

using namespace YamiParser::Av1;
#define DIM(a) (sizeof(a)/sizeof(a[0]))

TEST(EntropyDecoder, read) {
	const uint8_t data[] = { 0x16,  0x7f,  0xd4, 0xbf, 0xf1, 0x3f,  0x3d, 0xda,  0x0e,  0x20 };
	std::unique_ptr<EntropyDecoder> m_decoder(new EntropyDecoder(data, sizeof(data), false, 0));
	
	PARTITION_TYPE partition = m_decoder->readPartition(0, 1);
	EXPECT_EQ(PARTITION_NONE, partition);

	bool skip = m_decoder->readSkip(0);
	EXPECT_FALSE(skip);

	PREDICTION_MODE yMode = m_decoder->readIntraFrameYMode(0, 0);
	EXPECT_EQ(DC_PRED, yMode);

	UV_PREDICTION_MODE uvMode = m_decoder->readUvMode(CFL_ALLOWED, yMode);
	EXPECT_EQ(UV_V_PRED, uvMode);

	uint8_t angleDeltaUv = m_decoder->readAngleDeltaUV(uvMode);
	EXPECT_EQ(0, angleDeltaUv);

	bool useFilterIntra = m_decoder->readUseFilterIntra(BLOCK_8X8);
	EXPECT_FALSE(useFilterIntra);

	bool allZero = m_decoder->readAllZero(0, 1);
	EXPECT_FALSE(allZero);

	

}
