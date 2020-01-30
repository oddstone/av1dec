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

#pragma once

#include "prob.h"
#include <vector>

namespace YamiAv1 {

class Cdfs {
    friend class EntropyDecoder;

public:
    void init_non_coeff_cdfs();
    void init_coeff_cdfs(uint32_t baseQ);
    void load_cdfs(const Cdfs&);

private:
    friend class EntropyDecoder;
    typedef std::vector<aom_cdf_prob> Vec1;
    typedef std::vector<Vec1> Vec2;
    typedef std::vector<Vec2> Vec3;
    typedef std::vector<Vec3> Vec4;
    typedef std::vector<Vec4> Vec5;
    Vec3 txb_skip_cdf;
    Vec4 eob_extra_cdf;
    Vec3 dc_sign_cdf;
    Vec3 eob_flag_cdf16;
    Vec3 eob_flag_cdf32;
    Vec3 eob_flag_cdf64;
    Vec3 eob_flag_cdf128;
    Vec3 eob_flag_cdf256;
    Vec3 eob_flag_cdf512;
    Vec3 eob_flag_cdf1024;
    Vec4 coeff_base_eob_cdf;
    Vec4 coeff_base_cdf;
    Vec4 coeff_br_cdf;

    Vec2 newmv_cdf;
    Vec2 zeromv_cdf;
    Vec2 refmv_cdf;
    Vec2 drl_cdf;

    Vec2 inter_compound_mode_cdf;
    Vec2 compound_type_cdf;
    Vec2 wedge_idx_cdf;
    Vec2 interintra_cdf;
    Vec2 wedge_interintra_cdf;
    Vec2 interintra_mode_cdf;
    Vec2 motion_mode_cdf;
    Vec2 obmc_cdf;
    Vec2 palette_y_size_cdf;
    Vec2 palette_uv_size_cdf;
    Vec3 palette_y_color_index_cdf;
    Vec3 palette_uv_color_index_cdf;
    Vec3 palette_y_mode_cdf;
    Vec2 palette_uv_mode_cdf;
    Vec2 comp_inter_cdf;
    Vec3 single_ref_cdf;
    Vec2 comp_ref_type_cdf;
    Vec3 uni_comp_ref_cdf;
    Vec3 comp_ref_cdf;
    Vec3 comp_bwdref_cdf;
    Vec2 txfm_partition_cdf;
    Vec2 compound_index_cdf;
    Vec2 comp_group_idx_cdf;
    Vec2 skip_mode_cdfs;
    Vec2 skip_cdfs;
    Vec2 intra_inter_cdf;
    Vec1 intrabc_cdf;
    //struct segmentation_probs seg;
    Vec2 filter_intra_cdfs;
    Vec1 filter_intra_mode_cdf;
    Vec1 switchable_restore_cdf;
    Vec1 wiener_restore_cdf;
    Vec1 sgrproj_restore_cdf;
    Vec2 y_mode_cdf;
    Vec3 uv_mode_cdf;
    Vec2 partition_cdf;
    Vec2 switchable_interp_cdf;
    /* kf_y_cdf is discarded after use, so does not require persistent storage.
    However, we keep it with the other CDFs in this struct since it needs to
    be copied to each tile to support parallelism just like the others.
    */
    Vec3 kf_y_cdf;

    Vec2 angle_delta_cdf;

    Vec3 tx_size_cdf;
    Vec1 delta_q_cdf;
    Vec2 delta_lf_multi_cdf;
    Vec1 delta_lf_cdf;
    Vec4 intra_ext_tx_cdf;
    Vec3 inter_ext_tx_cdf;
    Vec1 cfl_sign_cdf;
    Vec2 cfl_alpha_cdf;

    struct nmv_component {
        Vec1 classes_cdf;
        Vec2 class0_fp_cdf;
        Vec1 fp_cdf;
        Vec1 sign_cdf;
        Vec1 class0_hp_cdf;
        Vec1 hp_cdf;
        Vec1 class0_cdf;
        Vec2 bits_cdf;
        void load_cdfs(const nmv_component&);
    };

    struct NmvContext {
        Vec1 joints_cdf;
        nmv_component comps[2];
        void load_cdfs(const NmvContext&);
    };

    NmvContext nmv_context[2];
};

}