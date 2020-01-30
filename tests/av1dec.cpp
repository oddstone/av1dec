#include "Av1Decoder.h"
#include "VideoFrame.h"
#include "decodeinput.h"
#include <chrono>
#include <iostream>
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

using std::chrono::duration;
using std::chrono::system_clock;
using std::chrono::time_point;
class Fps {
    enum Tag {
        READ,
        WRITE,
        DECODE,
        COUNT_OF_TAG,
    };

public:
    void startRead()
    {
        start(READ);
    }
    void endRead()
    {
        end(READ);
    }
    void startDecode()
    {
        start(DECODE);
    }
    void endDecode()
    {
        end(DECODE);
    }
    void startWrite()
    {
        start(WRITE);
    }
    void endWrite()
    {
        end(WRITE);
    }
    void fps()
    {
        std::cout << "decode fps: " << getFps(DECODE) << "\r";
    }
    void summary()
    {
        std::cout << "decode fps = " << getFps(DECODE)
                  << ", read fps = " << getFps(READ)
                  << ", write fps = " << getFps(WRITE) << std::endl;
    }

private:
    float getFps(Tag tag)
    {
        float d = m_duration[tag].count();
        if (d > 0) {
            return count[tag] / d;
        }
        return 0;
    }
    void start(Tag tag)
    {
        m_start[tag] = system_clock::now();
    }
    void end(Tag tag)
    {
        duration<float> d = system_clock::now() - m_start[tag];
        m_duration[tag] += d;
        count[tag]++;
    }

    time_point<system_clock> m_start[COUNT_OF_TAG];
    duration<float> m_duration[COUNT_OF_TAG];
    int count[COUNT_OF_TAG];
};

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
    Fps fps;
    while (1) {
        //printf("%d\r\n", buf.size);
        fps.startRead();
        if (!input->getNextDecodeUnit(buf))
            break;
        fps.endRead();

        fps.startDecode();
        decoder.decode(buf.data, buf.size);
        fps.endDecode();

        fps.fps();

        std::shared_ptr<Yami::YuvFrame> frame;
        while ((frame = decoder.getOutput())) {
            fps.startWrite();
            writeFrame(out, frame);
            fps.endWrite();
        }
    }
    fps.summary();
    fclose(out);
    //getchar();
    return 0;
}
