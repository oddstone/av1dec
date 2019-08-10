/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

/*!\defgroup codec Common Algorithm Interface
 * This abstraction allows applications to easily support multiple video
 * formats with minimal code duplication. This section describes the interface
 * common to all codecs (both encoders and decoders).
 * @{
 */

/*!\file
 * \brief Describes the codec algorithm interface to applications.
 *
 * This file describes the interface between an application and a
 * video codec algorithm.
 *
 * An application instantiates a specific codec instance by using
 * aom_codec_init() and a pointer to the algorithm's interface structure:
 *     <pre>
 *     my_app.c:
 *       extern aom_codec_iface_t my_codec;
 *       {
 *           aom_codec_ctx_t algo;
 *           res = aom_codec_init(&algo, &my_codec);
 *       }
 *     </pre>
 *
 * Once initialized, the instance is manged using other functions from
 * the aom_codec_* family.
 */
#ifndef AOM_AOM_CODEC_H_
#define AOM_AOM_CODEC_H_

#ifdef __cplusplus
extern "C" {
#endif


/*!\brief Decorator indicating a function is deprecated */
#ifndef AOM_DEPRECATED
#if defined(__GNUC__) && __GNUC__
#define AOM_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define AOM_DEPRECATED
#else
#define AOM_DEPRECATED
#endif
#endif /* AOM_DEPRECATED */

#ifndef AOM_DECLSPEC_DEPRECATED
#if defined(__GNUC__) && __GNUC__
#define AOM_DECLSPEC_DEPRECATED /**< \copydoc #AOM_DEPRECATED */
#elif defined(_MSC_VER)
/*!\brief \copydoc #AOM_DEPRECATED */
#define AOM_DECLSPEC_DEPRECATED __declspec(deprecated)
#else
#define AOM_DECLSPEC_DEPRECATED /**< \copydoc #AOM_DEPRECATED */
#endif
#endif /* AOM_DECLSPEC_DEPRECATED */

/*!\brief Decorator indicating a function is potentially unused */
#ifdef AOM_UNUSED
#elif defined(__GNUC__) || defined(__clang__)
#define AOM_UNUSED __attribute__((unused))
#else
#define AOM_UNUSED
#endif

/*!\brief Decorator indicating that given struct/union/enum is packed */
#ifndef ATTRIBUTE_PACKED
#if defined(__GNUC__) && __GNUC__
#define ATTRIBUTE_PACKED __attribute__((packed))
#elif defined(_MSC_VER)
#define ATTRIBUTE_PACKED
#else
#define ATTRIBUTE_PACKED
#endif
#endif /* ATTRIBUTE_PACKED */

/*!@} - end defgroup codec*/
#ifdef __cplusplus
}
#endif
#endif  // AOM_AOM_CODEC_H_
