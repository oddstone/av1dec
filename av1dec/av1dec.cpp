#include "tests/decodeinput.h"
#include "codecparsers/Av1Parser.h"
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
	YamiParser::Av1::Parser parser;
	while (input->getNextDecodeUnit(buf)) {
		printf("%d\r\n", buf.size);
		parser.parse(buf.data, buf.size);

	}
	getchar();
    return 0;
}

