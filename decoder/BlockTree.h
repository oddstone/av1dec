#ifndef BlockTree_h
#define BlockTree_h

struct BlockTree {
public:
    virtual void parse() = 0;
    virtual bool decode(std::shared_ptr<YuvFrame>&) = 0;
};

#endif //BlockTree_h
