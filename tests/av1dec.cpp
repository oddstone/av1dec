#include "Av1Decoder.h"
#include "VideoFrame.h"
#include "decodeinput.h"
#include <string.h>

void usage(const char* app)
{
    printf("%s -i input output", app);
}

void writeFrame(FILE* fp, std::shared_ptr<Yami::YuvFrame>& frame)
{
    for (int p = 0; p < Yami::YuvFrame::MAX_PLANES; p++) {
        uint8_t* start = frame->data[p];
        int stride = frame->strides[p];
        int width = frame->width;
        int height = frame->height;
        if (p) {
            width >>= 1;
            height >>= 1;
        }
        for (int h = 0; h < height; h++) {
            uint8_t* line = start + h * stride;
            fwrite(line, 1, width, fp);
        }
    }
    fflush(fp);
}

int main(int argc, char** argv)
{
    if (argc != 4 || strcmp(argv[1], "-i")) {
        usage(argv[0]);
        return -1;
    }
    SharedPtr<DecodeInput> input(DecodeInput::create(argv[2]));
    if (!input) {
        printf("can't open input %s", argv[1]);
        return -1;
    }
    FILE* out = fopen(argv[3], "wb");
    if (!out) {
        printf("can't open %s for write");
        return -1;
    }

    VideoDecodeBuffer buf;
    YamiAv1::Decoder decoder;
    while (input->getNextDecodeUnit(buf)) {
        printf("%d\r\n", buf.size);
        decoder.decode(buf.data, buf.size);
        std::shared_ptr<Yami::YuvFrame> frame;
        while ((frame = decoder.getOutput())) {
            writeFrame(out, frame);
        }
    }
    fclose(out);
    //getchar();
    return 0;
}
