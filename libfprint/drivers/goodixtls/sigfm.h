/*
 * SIGFM algorithm for libfprint
 *
 * Copyright (C) 2022 Matthieu CHARETTE <matthieu.charette@gmail.com>
 * Copyright (c) 2022 Natasha England-Elbro <ashenglandelbro@protonmail.com>
 * Copyright (c) 2022 Timur Mangliev <tigrmango@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
  typedef unsigned char SigfmPix;

  /* Opaque handle for extracted fingerprint features.
   * Obtain via sigfm_extract(), free with sigfm_free_info(). */
  typedef struct SigfmImgInfo SigfmImgInfo;

  /* Extract FAST-9/BRIEF-256 features from a greyscale image.
   * Returns a newly-allocated SigfmImgInfo (free with sigfm_free_info()).
   * Uses the historical default cap of 128 keypoints per frame.  For sensors
   * whose natural keypoint density justifies a different cap, call
   * sigfm_extract_ex directly. */
  SigfmImgInfo *sigfm_extract(const SigfmPix *pix, int width, int height);

  /* Same as sigfm_extract but with a caller-supplied per-frame keypoint cap.
   * The cap is per-scale (full-res and half-res are detected separately and
   * merged with near-duplicate suppression, so the returned count can briefly
   * exceed the cap during merge but is then bounded by it).
   * max_kp must be > 0; returns NULL on invalid input. */
  SigfmImgInfo *sigfm_extract_ex(const SigfmPix *pix, int width, int height,
                                 int max_kp);

  /* Free an SigfmImgInfo.  Do not use plain free(). */
  void sigfm_free_info(SigfmImgInfo *info);

  /* Score how closely frame matches enrolled.
   * Returns >=0 on success (higher = better match), <0 on error. */
  int sigfm_match_score(SigfmImgInfo *frame, SigfmImgInfo *enrolled);

  /* Serialize info into a byte array.  Caller frees the return value.
   * Sets *outlen to the length of the returned buffer. */
  unsigned char *sigfm_serialize_binary(SigfmImgInfo *info, int *outlen);

  /* Deserialize an SigfmImgInfo from bytes/len.
   * Returns NULL on failure. */
  SigfmImgInfo *sigfm_deserialize_binary(const unsigned char *bytes, int len);

  /* Number of keypoints in info.  Low counts indicate poor image quality. */
  int sigfm_keypoints_count(SigfmImgInfo *info);

  /* Return a deep copy of info. */
  SigfmImgInfo *sigfm_copy_info(SigfmImgInfo *info);

#ifdef __cplusplus
}
#endif
