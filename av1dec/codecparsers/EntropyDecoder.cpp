#include "EntropyDecoder.h"
#include "../aom/entropymode.c"
#include <string.h>


EntropyDecoder::EntropyDecoder()
{
	aom_cdf_prob prob = default_kf_y_mode_cdf[0][0][0];
	//memset(default_kf_y_mode_cdf, 0, sizeof(default_kf_y_mode_cdf));
}


EntropyDecoder::~EntropyDecoder()
{
}
