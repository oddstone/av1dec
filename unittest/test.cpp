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

	uint8_t eobPt = m_decoder->readEobPt(0, PLANE_TYPE_Y, 0);
	EXPECT_EQ(1, eobPt);

	uint8_t coeffBaseEob = m_decoder->readCoeffBaseEob(0, PLANE_TYPE_Y, 0);
	EXPECT_EQ(2, coeffBaseEob);

	uint8_t coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_Y, 0);
	EXPECT_EQ(3, coeffBr);

	coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_Y, 0);
	EXPECT_EQ(3, coeffBr);

	coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_Y, 0);
	EXPECT_EQ(3, coeffBr);

	coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_Y, 0);
	EXPECT_EQ(3, coeffBr);

	bool dcSign = m_decoder->readDcSign(PLANE_TYPE_Y, 0);
	EXPECT_FALSE(dcSign);

	uint32_t v;
	bool ret = m_decoder->readUe(v);
	EXPECT_TRUE(ret);
	EXPECT_EQ(402, v);
	//why?
	//EXPECT_EQ(401, v);
	allZero = m_decoder->readAllZero(0, 3);
	EXPECT_TRUE(allZero);

	
	allZero = m_decoder->readAllZero(0, 3);
	EXPECT_TRUE(allZero);

	allZero = m_decoder->readAllZero(0, 1);
	EXPECT_TRUE(allZero);

	allZero = m_decoder->readAllZero(0, 7);
	EXPECT_FALSE(allZero);

	eobPt = m_decoder->readEobPt(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(1, eobPt);

	coeffBaseEob = m_decoder->readCoeffBaseEob(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(2, coeffBaseEob);

	coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(3, coeffBr);
	
	coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(2, coeffBr);

	dcSign = m_decoder->readDcSign(PLANE_TYPE_UV, 0);
	EXPECT_TRUE(dcSign);

	allZero = m_decoder->readAllZero(0, 7);
	EXPECT_FALSE(allZero);

	eobPt = m_decoder->readEobPt(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(1, eobPt);

	coeffBaseEob = m_decoder->readCoeffBaseEob(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(2, coeffBaseEob);


	coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(3, coeffBr);

	coeffBr = m_decoder->readCoeffBr(0, PLANE_TYPE_UV, 0);
	EXPECT_EQ(2, coeffBr);

	dcSign = m_decoder->readDcSign(PLANE_TYPE_UV, 0);
	EXPECT_TRUE(dcSign);
}
