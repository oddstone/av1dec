#include "decodeinput.h"
#include "h265Parser.h"
#include "common/nalreader.h"
#include "decoder/ArithmeticDecoder.h"
#include <functional>
#include <set>

using namespace YamiParser::H265;
#define N_ELEMENTS(a) (sizeof(a)/sizeof(a[0]))

bool isIdr(const NalUnit* const nalu)
{
	return nalu->nal_unit_type == NalUnit::IDR_W_RADL
		|| nalu->nal_unit_type == NalUnit::IDR_N_LP;
}

bool isBla(const NalUnit* const nalu)
{
	return nalu->nal_unit_type == NalUnit::BLA_W_LP
		|| nalu->nal_unit_type == NalUnit::BLA_W_RADL
		|| nalu->nal_unit_type == NalUnit::BLA_N_LP;
}

bool isIrap(const NalUnit* const nalu)
{
	return nalu->nal_unit_type >= NalUnit::BLA_W_LP
		&& nalu->nal_unit_type <= NalUnit::RSV_IRAP_VCL23;
}

bool isRasl(const NalUnit* const nalu)
{
	return nalu->nal_unit_type == NalUnit::RASL_R
		|| nalu->nal_unit_type == NalUnit::RASL_N;
}

bool isRadl(const NalUnit* const nalu)
{
	return nalu->nal_unit_type == NalUnit::RADL_R
		|| nalu->nal_unit_type == NalUnit::RADL_N;
}

bool isCra(const NalUnit* const nalu)
{
	return nalu->nal_unit_type == NalUnit::CRA_NUT;
}

bool isSublayerNoRef(const NalUnit* const nalu)
{
	static const uint8_t noRef[] = {
		NalUnit::TRAIL_N,
		NalUnit::TSA_N,
		NalUnit::STSA_N,
		NalUnit::RADL_N,
		NalUnit::RASL_N,
		NalUnit::RSV_VCL_N10,
		NalUnit::RSV_VCL_N12,
		NalUnit::RSV_VCL_N14
	};
	static const uint8_t* end = noRef + N_ELEMENTS(noRef);
	return std::binary_search(noRef, end, nalu->nal_unit_type);
}

class DecPictureH265;
class HevcDecoder
{
	typedef YamiParser::H265::SPS SPS;
	typedef YamiParser::H265::SliceHeader SliceHeader;
	typedef YamiParser::H265::NalUnit NalUnit;
	typedef YamiParser::H265::Parser Parser;

public:
	typedef SharedPtr<DecPictureH265> PicturePtr;
	typedef std::vector<DecPictureH265*> RefSet;
	HevcDecoder()
	{
		m_parser.reset(new Parser());
		m_prevSlice.reset(new SliceHeader());

	}
	YamiStatus decode(const uint8_t* data, uint32_t size)
	{
		YamiMediaCodec::NalReader nr(data, size);
		const uint8_t* nalData;
		int32_t nalSize;
		YamiParser::H265::NalUnit  nal;
		while (nr.read(nalData, nalSize)) {
			if (nal.parseNaluHeader(nalData, nalSize)) {
				printf("%d\r\n", nal.nal_unit_type);
			}
		}
		return YAMI_SUCCESS;
	}
private:
	class DPB {
		typedef HevcDecoder::RefSet     RefSet;
		typedef std::function<YamiStatus(const PicturePtr&)> OutputCallback;
		typedef std::function<void(const PicturePtr&)> ForEachFunction;
	public:
		typedef HevcDecoder::PicturePtr PicturePtr;
		DPB(OutputCallback output);
		bool init(const PicturePtr&,
			const SliceHeader *const,
			const NalUnit *const,
			bool newStream);
		bool add(const PicturePtr&, const SliceHeader* const lastSlice);
		void flush();

		RefSet m_stCurrBefore;
		RefSet m_stCurrAfter;
		RefSet m_stFoll;
		RefSet m_ltCurr;
		RefSet m_ltFoll;
	private:
		void forEach(ForEachFunction);
		bool initReference(const PicturePtr&,
			const SliceHeader *const,
			const NalUnit *const,
			bool newStream);
		bool initShortTermRef(const PicturePtr& picture,
			const SliceHeader* const);
		bool initShortTermRef(RefSet& ref, int32_t currPoc,
			const int32_t* const delta,
			const uint8_t* const used,
			uint8_t num);
		bool initLongTermRef(const PicturePtr&,
			const SliceHeader *const);
		DecPictureH265* getPic(int32_t poc, bool hasMsb = true);

		//for C.5.2.2
		bool checkReorderPics(const SPS* const sps);
		bool checkDpbSize(const SPS* const);
		bool checkLatency(const SPS* const);
		void removeUnused();
		void clearRefSet();

		void bumpAll();
		bool bump();
		bool output(const PicturePtr& picture);


		struct PocLess
		{
			inline bool operator()(const PicturePtr& left, const PicturePtr& right) const;
		};
		typedef std::set<PicturePtr, PocLess> PictureList;
		PictureList     m_pictures;
		OutputCallback  m_output;
		PicturePtr      m_dummy;
	};

	YamiStatus decodeNalu(NalUnit* nalu)
	{
		uint8_t type = nalu->nal_unit_type;
		YamiStatus status = YAMI_SUCCESS;

		if (NalUnit::TRAIL_N <= type && type <= NalUnit::CRA_NUT) {
			status = decodeSlice(nalu);
			if (status == YAMI_DECODE_INVALID_DATA) {
				//ignore invalid data while decoding slice
				m_current.reset();
				status = YAMI_SUCCESS;
			}
		}
		else {
			status = decodeCurrent();
			if (status != YAMI_SUCCESS)
				return status;
			switch (type) {
			case NalUnit::VPS_NUT:
			case NalUnit::SPS_NUT:
			case NalUnit::PPS_NUT:
				status = decodeParamSet(nalu);
				break;
			case NalUnit::EOB_NUT:
				m_newStream = true;
				break;
			case NalUnit::EOS_NUT:
				m_endOfSequence = true;
				break;
			case NalUnit::AUD_NUT:
			case NalUnit::FD_NUT:
			case NalUnit::PREFIX_SEI_NUT:
			case NalUnit::SUFFIX_SEI_NUT:
			default:
				break;
			}
		}

		return status;
	}

	YamiStatus decodeParamSet(NalUnit* nalu)
	{
		bool res = true;

		switch (nalu->nal_unit_type) {
		case NalUnit::VPS_NUT:
			res = m_parser->parseVps(nalu);
			break;
		case NalUnit::SPS_NUT:
			res = m_parser->parseSps(nalu);
			break;
		case NalUnit::PPS_NUT:
			res = m_parser->parsePps(nalu);
		}

		return res ? YAMI_SUCCESS : YAMI_DECODE_INVALID_DATA;
	}

	YamiStatus decodeSlice(NalUnit* nalu)
	{
		SharedPtr<SliceHeader> currSlice(new SliceHeader());
		SliceHeader* slice = currSlice.get();
		YamiStatus status;

		if (!m_parser->parseSlice(nalu, slice))
			return YAMI_DECODE_INVALID_DATA;

		if (slice->first_slice_segment_in_pic_flag) {
			status = decodeCurrent();
			if (status != YAMI_SUCCESS)
				return status;
			
		}

		if (!slice->dependent_slice_segment_flag)
			std::swap(currSlice, m_prevSlice);
		return status;
	}
	YamiStatus decodeSliceData(NalUnit* nalu, SliceHeader* slice)
	{
		uint32_t CtbAddrInTs = 0;
		do {
			YamiParser::NalReader nal(nalu->m_data + slice->getSliceDataByteOffset(), nalu->m_size - slice->getSliceDataByteOffset());
			//bool end_of_slice_segment_flag = 
		} while (false);
	}

	YamiStatus decodeCurrent()
	{
		return YAMI_SUCCESS;
	}
private:

	SharedPtr<Parser> m_parser;
	PicturePtr  m_current;
	uint16_t    m_prevPicOrderCntMsb;
	int32_t     m_prevPicOrderCntLsb;
	int32_t     m_nalLengthSize;
	bool        m_associatedIrapNoRaslOutputFlag;
	bool        m_noRaslOutputFlag;
	bool        m_newStream;
	bool        m_endOfSequence;
	SharedPtr<SliceHeader> m_prevSlice;

	//DPB         m_dpb;
};

int main(int argc, char** argv)
{
	if (argc != 2) {
		printf("decode a.265");
		return -1;
	}
	SharedPtr<DecodeInput> input(DecodeInput::create(argv[1]));
	if (!input) {
		printf("can't open input %s", argv[1]);
		return -1;
	}
	HevcDecoder decoder;
	VideoDecodeBuffer buf;
	while (input->getNextDecodeUnit(buf)) {
		decoder.decode(buf.data, buf.size);
		
	}
	//getchar();
	//getchar();



}