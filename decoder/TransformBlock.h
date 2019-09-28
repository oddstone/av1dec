#ifndef TransformBlock_h
#define TransformBlock_h

#include "Block.h"

struct YuvFrame;

namespace Yami {
    namespace Av1 {

        enum TxClass {
            TX_CLASS_2D,
            TX_CLASS_HORIZ,
            TX_CLASS_VERT
        };

        class TransformBlock {
        public:
            TransformBlock(Block& block, int plane, int startX, int startY, TX_SIZE txSz);
            void parse();
            bool decode(std::shared_ptr<YuvFrame>& frame);

        private:
            TX_TYPE compute_tx_type() const;
            bool is_tx_type_in_set(TxSet txSet, TX_TYPE txType) const;
            TxSet get_tx_set() const;
            uint8_t getAllZeroCtx();
            int16_t get_q_idx();
            void transform_type(TX_TYPE type);
            void transform_type();

            const int16_t* get_mrow_scan() const;
            const int16_t* get_mcol_scan() const;
            const int16_t* get_default_scan() const;
            const int16_t* get_scan() const;

            TxClass get_tx_class() const;

            int getEob(int eobMultisize);
            uint8_t get_coeff_base_ctx(int pos, int c, bool isEob) const;
            uint8_t getCoeffBrCtx(int pos);
            uint8_t getDcSignCtx() const;
            bool getSign(int c, int pos);
            int16_t getLevel();
            int16_t getLevel(int c, int eob, int pos);

            int coeffs();
            int dc_q(int16_t b) const;
            int ac_q(int16_t b) const;
            int get_dc_quant() const;
            int get_ac_quant() const;
            int getQ2(int i, int j) const;
            void inverseTransform();
            void reconstruct();
            void predict_intra(int availL, int availU, bool decodedUpRight, bool decodedBottomLeft,
                int mode, const std::shared_ptr<YuvFrame>& frame);
            void recursiveIntraPrediction(const uint8_t* AboveRow, const uint8_t* LeftCol,
                const std::shared_ptr<YuvFrame>& frame);
            void paethPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);
            void dcPredict(bool haveAbove, bool haveLeft,
                const uint8_t* AboveRow, const uint8_t* LeftCol);
            bool getLeftSmooth() const;
            bool getAboveSmooth() const;
            bool getSmooth(int r, int c) const;
            bool get_filter_type(bool haveLeft, bool haveAbove) const;
            uint8_t* intraEdgeUpsample(const uint8_t* edge, int numPx, std::vector<uint8_t>& upsampled) const;
            void directionalIntraPredict(bool haveAbove, bool haveLeft, uint8_t* AboveRow, uint8_t* LeftCol,
                int mode);
            PREDICTION_MODE getIntraDir();
            void predict_chroma_from_luma(std::shared_ptr<YuvFrame>& frame);
            void smoothPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);
            void smoothVPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);
            void smoothHPredict(const uint8_t* AboveRow, const uint8_t* LeftCol);

            EntropyDecoder& m_entropy;
            Block& m_block;
            Tile& m_tile;
            FrameHeader& m_frame;
            const SequenceHeader& m_sequence;

            int plane;
            PLANE_TYPE ptype;
            int x;
            int y;
            int x4;
            int y4;
            TX_SIZE txSz;
            TX_SIZE txSzSqr;
            TX_SIZE txSzSqrUp;
            TX_SIZE txSzCtx;
            TxClass txClass;
            int w;
            int h;
            int w4;
            int h4;
            int log2W;
            int log2H;

            std::vector<std::vector<uint8_t>> pred;
            TX_TYPE PlaneTxType;
            int Quant[1024];
            int Dequant[64][64];
            int Residual[64][64];

            //for reconstruction
            int dqDenom;
            int tw;
            int th;
            bool flipUD;
            bool flipLR;
            int m_eob;
        };
    }
}
#endif