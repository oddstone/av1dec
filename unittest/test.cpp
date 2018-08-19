#include "pch.h"
#include <memory>
#include "../av1dec/SymbolDecoder.h"

using namespace YamiParser::Av1;
#define DIM(a) (sizeof(a)/sizeof(a[0]))
TEST(SymoblDecoder, read) {
	const uint8_t data[] = { 0x16,  0x7f,  0xd4, 0xbf, 0xf1, 0x3f,  0x3d, 0xda,  0x0e,  0x20 };
	std::unique_ptr<SymbolDecoder> m_decoder(new SymbolDecoder(data, sizeof(data), false));

	uint16_t partitionCdf[] = { 13636 , 7258, 2376, 0, 0 };
	uint8_t partition = m_decoder->read(partitionCdf, DIM(partitionCdf) - 1 );
	EXPECT_EQ(0, partition);
	uint16_t updatedPartitionCdf[] = { 0x339a, 0x1b78, 0x08fe, 0x0000 , 0x1};
	EXPECT_TRUE(std::equal(updatedPartitionCdf, updatedPartitionCdf + DIM(updatedPartitionCdf), partitionCdf));

	uint16_t skipCdf[] = { 1097, 0, 0};
	uint8_t skip = m_decoder->read(skipCdf, DIM(skipCdf) - 1);
	EXPECT_EQ(0, skip);

	uint16_t ymodeCdf[] = { 0x431c, 0x3d7d, 0x3476, 0x3106, 0x2f36, 0x2d8a, 0x2abf, 0x2534, 0x2183, 0x11fb, 0x0e5b, 0x08fe, 0x0000 };
	uint8_t ymode = m_decoder->read(ymodeCdf, DIM(ymodeCdf));
	EXPECT_EQ(0, ymode);


	uint16_t updatedSkipCdf[] = { 0x0405, 0x0000, 0x0001 };
	EXPECT_TRUE(std::equal(updatedSkipCdf, updatedSkipCdf + DIM(updatedSkipCdf), skipCdf));

	uint16_t intraCdf[] = { 0x431c, 0x3d7d, 0x3476, 0x3106, 0x2f36, 0x2d8a, 0x2abf, 0x2534, 0x2183, 0x11fb, 0x0e5b, 0x08fe, 0x0000 , 0x0};
	uint8_t intra = m_decoder->read(intraCdf, DIM(intraCdf) - 1);
	EXPECT_EQ(0, intra);
	uint16_t updatedIntraCdf[] = { 0x4104, 0x3b92, 0x32d3, 0x2f7e, 0x2dbd, 0x2c1e, 0x296a, 0x240b, 0x2077, 0x116c, 0x0de9, 0x08b7, 0x0000, 0x1 };
	EXPECT_TRUE(std::equal(updatedIntraCdf, updatedIntraCdf + DIM(updatedIntraCdf), intraCdf));


	uint16_t intraUvCdf[] = { 0x5759, 0x5438, 0x4d9c, 0x4c83, 0x4a01, 0x48a1, 0x45cd, 0x42d8, 0x418e, 0x318a, 0x2dfd, 0x2849, 0x216c, 0x0, 0x0 };

	uint8_t intraUv = m_decoder->read(intraUvCdf, DIM(intraUvCdf) - 1);
	EXPECT_EQ(1, intraUv);

	uint16_t updateIntraUvCdf[] = { 0x589e, 0x5197, 0x4b30, 0x4a1f, 0x47b1, 0x465c, 0x439f, 0x40c2, 0x3f82, 0x2ffe, 0x2c8e, 0x2707, 0x2061, 0x0, 0x1 };
	EXPECT_TRUE(std::equal(updateIntraUvCdf, updateIntraUvCdf + DIM(updateIntraUvCdf), intraUvCdf));

	uint16_t angleDeltaCdf[] = { 0x777c, 0x6c58, 0x6271, 0x2708, 0x1693, 0x09f7, 0x0000, 0x0 };
	uint8_t angleDelta = m_decoder->read(angleDeltaCdf, DIM(angleDeltaCdf) - 1);
	EXPECT_EQ(3, angleDelta);

	uint16_t filterIntraModeCdf[] = { 0x6146, 0x0000, 0x0};
	uint8_t filterIntraMode = m_decoder->read(filterIntraModeCdf, DIM(filterIntraModeCdf) - 1);;
	EXPECT_EQ(0, filterIntraMode);

}