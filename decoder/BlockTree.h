#ifndef BlockTree_h
#define BlockTree_h


namespace Yami {
    namespace Av1 {
        struct BlockTree {
        public:
            virtual void parse() = 0;
            virtual bool decode(std::shared_ptr<YuvFrame>&, const FrameStore&) = 0;
        };
    }
}

#endif //BlockTree_h
