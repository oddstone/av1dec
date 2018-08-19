#include "tests/decodeinput.h"
#include "decoder/Av1Decoder.h"
int main(int argc, char** argv)
{
	if (argc != 2) {
		printf("decode a.ivf");
		return -1;
	}
	SharedPtr<DecodeInput> input(DecodeInput::create(argv[1]));
	if (!input) {
		printf("can't open input %s", argv[1]);
		return -1;
	}

	VideoDecodeBuffer buf;
	YamiParser::Av1::Decoder decoder;
	while (input->getNextDecodeUnit(buf)) {
		printf("%d\r\n", buf.size);
		decoder.decode(buf.data, buf.size);

	}
	getchar();
    return 0;
}

