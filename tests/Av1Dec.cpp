/*
 * Copyright 2020, av1dec authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Av1Decoder.h"
#include "DecodeInput.h"
#include "DecodeOutput.h"
#include "VideoFrame.h"
#include <chrono>
#include <iostream>
#include <string.h>

void usage(const char* app)
{
    printf("%s -i input [output]", app);
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
    Fps()
    {
        for (int i = 0; i < COUNT_OF_TAG; i++) {
            m_duration[i] = duration<float>::zero();
            count[i] = 0;
        }
    }
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
        std::cout << "decode fps: " << getFps(DECODE) << ", frame = " << count[DECODE] << "        \r";
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

class Decode {
public:
    bool parse(int argc, char** argv);
    int run();
    bool output(const std::shared_ptr<Yami::YuvFrame>& frame);

private:
    bool parseOption(int argc, char** argv, int& i);
    SharedPtr<DecodeInput> m_input;
    std::shared_ptr<DecodeOutput> m_output;
    std::shared_ptr<DecodeOutputMd5> m_md5;
    Fps m_fps;
};

bool Decode::parseOption(int argc, char** argv, int& i)
{
    if (strcmp(argv[i], "-i") == 0) {
        i++;
        if (i >= argc) {
            printf("no input file followed -i option\n");
            return false;
        }
        if (m_input) {
            printf("only support one input\n");
            return false;
        }
        m_input.reset(DecodeInput::create(argv[i]));
        if (!m_input) {
            printf("can't open input %s\n", argv[i]);
            return false;
        }
    } else if (strcmp(argv[i], "-md5") == 0) {
        m_md5 = DecodeOutputMd5::create();
    } else {
        printf("invalid command line param %s\n", argv[i]);
        return false;
    }
    return true;
}

bool Decode::parse(int argc, char** argv)
{
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if (!parseOption(argc, argv, i))
                return false;
        } else {
            if (m_output) {
                printf("do not support multi output\n");
                return false;
            }
            m_output = DecodeOutput::create(argv[i]);
            if (!m_output) {
                printf("can't open %s for write\n", argv[3]);
                return false;
            }
        }
    }
    return true;
}

bool Decode::output(const std::shared_ptr<Yami::YuvFrame>& frame)
{
    if (m_output) {
        m_fps.startWrite();
        if (!m_output->output(frame))
            return false;
        m_fps.endWrite();
    }
    if (m_md5) {
        m_md5->output(frame);
    }
    return true;
}

int Decode::run()
{
    VideoDecodeBuffer buf;
    YamiAv1::Decoder decoder;
    while (1) {
        //printf("%d\r\n", buf.size);
        m_fps.startRead();
        if (!m_input->getNextDecodeUnit(buf))
            break;
        m_fps.endRead();

        m_fps.startDecode();
        decoder.decode(buf.data, buf.size);
        m_fps.endDecode();

        m_fps.fps();

        std::shared_ptr<Yami::YuvFrame> frame;
        while ((frame = decoder.getOutput())) {
            if (!output(frame))
                break;
        }
    }
    m_fps.summary();
    return 0;
}

int main(int argc, char** argv)
{
    Decode decode;
    if (!decode.parse(argc, argv))
        return -1;
    return decode.run();
}
