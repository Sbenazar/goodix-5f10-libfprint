/*
 * SIGFM algorithm for libfprint -- pure-C ORB replacement
 *
 * Copyright (C) 2022 Matthieu CHARETTE <matthieu.charette@gmail.com>
 * Copyright (c) 2022 Natasha England-Elbro <ashenglandelbro@protonmail.com>
 * Copyright (c) 2022 Timur Mangliev <tigrmango@gmail.com>
 * Copyright (c) 2026 libfprint contributors
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * Algorithm: FAST-9 corner detection + unsteered BRIEF-256 descriptors.
 * Geometric verification uses RANSAC rigid-transform estimation with
 * cross-check filtered descriptor matching and inlier counting.
 */

#include "sigfm.h"

#include <glib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Data structures
 * ---------------------------------------------------------------------- */

typedef struct
{
  float x, y;
  float response;
} OrbKeypoint;

struct SigfmImgInfo
{
  int n_kp;
  OrbKeypoint *kp;
  uint8_t *desc; /* n_kp x 32 bytes, 256-bit BRIEF descriptor per kp */
};

/* -------------------------------------------------------------------------
 * BRIEF-256 bit-test pattern
 * Pairs (x1,y1,x2,y2) in range +/-PATCH_HALF, lazily initialised once.
 * -------------------------------------------------------------------- */

#define PATCH_HALF 7
#define PATCH_SIZE (PATCH_HALF * 2 + 1) /* 15 */
#define N_PAIRS 256
#define DESC_BYTES 32 /* 256 / 8 */

static int8_t brief_pattern[N_PAIRS][4];
static gsize brief_init_done = 0;

static void
init_brief_pattern (void)
{
  if (g_once_init_enter (&brief_init_done))
    {
      /* Deterministic hash-based pairs; covers all four quadrants of the patch. */
      for (int i = 0; i < N_PAIRS; i++)
        {
          unsigned int s1 = (unsigned int) (i * 7741u + 1234567u);
          unsigned int s2 = (unsigned int) (i * 3571u + 9876543u);
          int r1 = (int) (s1 % (unsigned) (PATCH_SIZE * PATCH_SIZE));
          int r2 = (int) (s2 % (unsigned) (PATCH_SIZE * PATCH_SIZE));
          brief_pattern[i][0] = (int8_t) (r1 % PATCH_SIZE - PATCH_HALF); /* x1 */
          brief_pattern[i][1] = (int8_t) (r1 / PATCH_SIZE - PATCH_HALF); /* y1 */
          brief_pattern[i][2] = (int8_t) (r2 % PATCH_SIZE - PATCH_HALF); /* x2 */
          brief_pattern[i][3] = (int8_t) (r2 / PATCH_SIZE - PATCH_HALF); /* y2 */
        }
      g_once_init_leave (&brief_init_done, 1);
    }
}

/* -------------------------------------------------------------------------
 * 3x3 box blur (scratch buffer passed in to avoid allocation per-keypoint)
 * ---------------------------------------------------------------------- */

static void
box3_blur(const uint8_t *src, uint8_t *dst, int w, int h)
{
  for (int y = 0; y < h; y++)
    {
      for (int x = 0; x < w; x++)
        {
          int sum = 0, cnt = 0;
          for (int dy = -1; dy <= 1; dy++)
            {
              int ny = y + dy;
              if (ny < 0 || ny >= h)
                continue;
              for (int dx = -1; dx <= 1; dx++)
                {
                  int nx = x + dx;
                  if (nx < 0 || nx >= w)
                    continue;
                  sum += src[ny * w + nx];
                  cnt++;
                }
            }
          dst[y * w + x] = (uint8_t)(sum / cnt);
        }
    }
}

/* -------------------------------------------------------------------------
 * FAST-9 corner detector
 * ---------------------------------------------------------------------- */

/* Bresenham circle radius-3 offsets (16 points) */
static const int fast_dx[16] = { 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1 };
static const int fast_dy[16] = { 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1, 0, 1, 2, 3 };

/* Returns a score > 0 if (x,y) is a FAST-9 corner, 0 otherwise. */
static int
fast9_score(const uint8_t *img, int w, int x, int y, int threshold)
{
  int cx = img[y * w + x];
  int v[16];

  for (int i = 0; i < 16; i++)
    v[i] = img[(y + fast_dy[i]) * w + (x + fast_dx[i])] - cx;

  /* Compass quick-reject: need at least 2 of the 4 cardinal points
   * to exceed the threshold in the same direction. */
  int nb = 0, nd = 0;
  for (int i = 0; i < 4; i++)
    {
      if (v[i * 4] > threshold)
        nb++;
      if (v[i * 4] < -threshold)
        nd++;
    }
  if (nb < 2 && nd < 2)
    return 0;

  /* Full test: look for 9 consecutive pixels all brighter or darker. */
  for (int start = 0; start < 16; start++)
    {
      int b = 1, d = 1;
      for (int k = 0; k < 9; k++)
        {
          int val = v[(start + k) % 16];
          if (val <= threshold)
            b = 0;
          if (val >= -threshold)
            d = 0;
        }
      if (!b && !d)
        continue;

      /* Score = sum of absolute deviations (higher -> stronger corner). */
      int score = 0;
      for (int i = 0; i < 16; i++)
        score += abs(v[i]);
      return score;
    }
  return 0;
}

/* -------------------------------------------------------------------------
 * Detect keypoints: FAST-9 + 3x3 non-maximum suppression
 * ---------------------------------------------------------------------- */

#define FAST_THRESHOLD 10
#define FAST_BORDER (3 + PATCH_HALF) /* avoid circle + patch OOB */
#define FAST_BORDER_DETECT 4         /* FAST radius 3 + 1 for NMS only */
/* Hard sanity ceiling for deserialized templates: large enough to accept any
 * driver's chosen extraction max_kp, small enough to reject malformed input. */
#define SIGFM_DESERIALIZE_KP_LIMIT 1024
#define MULTISCALE_DUP_DIST_SQ 16.0f /* 4px near-duplicate suppression */

static int
detect_keypoints_ex(const uint8_t *img, int w, int h, OrbKeypoint *kp_out, int max_kp,
                    int border)
{
  /* Score map: 0 = not a corner */
  int *scores = g_malloc0((gsize)(w * h) * sizeof(int));

  /* Pass 1: compute FAST scores */
  for (int y = border; y < h - border; y++)
    {
      for (int x = border; x < w - border; x++)
        scores[y * w + x] = fast9_score(img, w, x, y, FAST_THRESHOLD);
    }

  /* Pass 2: 3x3 NMS -- keep only local maxima */
  int n = 0;
  for (int y = border; y < h - border && n < max_kp; y++)
    {
      for (int x = border; x < w - border && n < max_kp; x++)
        {
          int s = scores[y * w + x];
          if (s == 0)
            continue;
          int is_max = 1;
          for (int dy = -1; dy <= 1 && is_max; dy++)
            for (int dx = -1; dx <= 1 && is_max; dx++)
              if (!(dx == 0 && dy == 0) && scores[(y + dy) * w + (x + dx)] > s)
                is_max = 0;
          if (is_max)
            {
              kp_out[n].x = (float)x;
              kp_out[n].y = (float)y;
              kp_out[n].response = (float)s;
              n++;
            }
        }
    }

  g_free(scores);
  return n;
}

/* Full-resolution detection with BRIEF clearance */
static int
detect_keypoints(const uint8_t *img, int w, int h, OrbKeypoint *kp_out, int max_kp)
{
  return detect_keypoints_ex(img, w, h, kp_out, max_kp, FAST_BORDER);
}

/* -------------------------------------------------------------------------
 * 2x downsample via 2x2 average pooling (for multi-scale pyramid)
 * ---------------------------------------------------------------------- */

static void
downsample_2x(const uint8_t *src, int w, int h, uint8_t *dst)
{
  int hw = w / 2, hh = h / 2;
  for (int y = 0; y < hh; y++)
    for (int x = 0; x < hw; x++)
      {
        int sx = x * 2, sy = y * 2;
        dst[y * hw + x]
            = (uint8_t)((src[sy * w + sx] + src[sy * w + sx + 1] + src[(sy + 1) * w + sx]
                         + src[(sy + 1) * w + sx + 1] + 2)
                        / 4);
      }
}

/* -------------------------------------------------------------------------
 * BRIEF-256 descriptor computation (no orientation steering)
 * ---------------------------------------------------------------------- */

static void
compute_descriptor(const uint8_t *img, int w, const OrbKeypoint *kp,
                   uint8_t *desc /* 32 bytes out */)
{
  int cx = (int)kp->x;
  int cy = (int)kp->y;

  memset(desc, 0, DESC_BYTES);
  for (int i = 0; i < N_PAIRS; i++)
    {
      int x1 = cx + brief_pattern[i][0];
      int y1 = cy + brief_pattern[i][1];
      int x2 = cx + brief_pattern[i][2];
      int y2 = cy + brief_pattern[i][3];
      if (img[y1 * w + x1] < img[y2 * w + x2])
        desc[i / 8] |= (uint8_t)(1u << (i % 8));
    }
}

/* -------------------------------------------------------------------------
 * Hamming distance between two 32-byte descriptors
 * ---------------------------------------------------------------------- */

static int
hamming_dist(const uint8_t *a, const uint8_t *b)
{
  int dist = 0;
  for (int i = 0; i < DESC_BYTES; i++)
    {
      uint8_t x = a[i] ^ b[i];
      /* Brian Kernighan bit-count */
      while (x)
        {
          dist++;
          x &= (uint8_t)(x - 1u);
        }
    }
  return dist;
}

/* -------------------------------------------------------------------------
 * KNN matching (k=2) with Lowe ratio test
 * ---------------------------------------------------------------------- */

#define RATIO_TEST 0.80f
#define MIN_MATCH 5

typedef struct
{
  int qi, ti;
  int dist;
} Match;

static int
knn_match(const SigfmImgInfo *query, const SigfmImgInfo *train, Match *matches_out,
          int max_matches)
{
  int n = 0;
  for (int q = 0; q < query->n_kp && n < max_matches; q++)
    {
      const uint8_t *qd = query->desc + q * DESC_BYTES;
      int best1 = 256, best2 = 256, best1_idx = -1;

      for (int t = 0; t < train->n_kp; t++)
        {
          int d = hamming_dist(qd, train->desc + t * DESC_BYTES);
          if (d < best1)
            {
              best2 = best1;
              best1 = d;
              best1_idx = t;
            }
          else if (d < best2)
            {
              best2 = d;
            }
        }

      if (best1_idx >= 0 && best2 > 0 && (float)best1 < RATIO_TEST * (float)best2)
        {
          matches_out[n].qi = q;
          matches_out[n].ti = best1_idx;
          matches_out[n].dist = best1;
          n++;
        }
    }
  return n;
}

/* Cross-check filter: keep only matches (q,t) where t->q is also the
 * best forward match in the reverse direction.  This eliminates many
 * false descriptor matches and dramatically reduces impostor match
 * counts, improving RANSAC discriminability.  */
static int
cross_check_filter(const SigfmImgInfo *query, const SigfmImgInfo *train, Match *matches,
                   int n_matches)
{
  int out = 0;
  for (int i = 0; i < n_matches; i++)
    {
      /* For train descriptor matches[i].ti, find best match in query */
      const uint8_t *td = train->desc + matches[i].ti * DESC_BYTES;
      int best_dist = 256, best_q = -1;
      for (int q = 0; q < query->n_kp; q++)
        {
          int d = hamming_dist(td, query->desc + q * DESC_BYTES);
          if (d < best_dist)
            {
              best_dist = d;
              best_q = q;
            }
        }
      /* Keep only if reverse best-match agrees */
      if (best_q == matches[i].qi)
        matches[out++] = matches[i];
    }
  return out;
}

/* -------------------------------------------------------------------------
 * Geometric verification -- RANSAC rigid-transform estimation
 *
 * Random 2-point sampling estimates rotation + translation; inliers
 * are counted under a pixel-distance threshold.  Least-squares
 * refinement from all inliers of the best model further improves the
 * transform before final scoring.
 * ---------------------------------------------------------------------- */

#define RANSAC_ITERATIONS 200
#define RANSAC_INLIER_THRESH 2.0f /* pixels */
#define RANSAC_MIN_DIST_SQ 9.0f   /* reject sample pairs < 3 px apart */

/* Simple xorshift32 PRNG -- deterministic, no seed dependency on libc */
static uint32_t
xorshift32(uint32_t *state)
{
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

/* Count inliers for a given rigid transform (cos,-sin,sin,cos,tx,ty). */
static int
count_inliers(const SigfmImgInfo *frame, const SigfmImgInfo *enrolled,
              const Match *matches, int n_matches, float a, float b, float c, float d,
              float tx, float ty, float eps_sq)
{
  int inliers = 0;
  for (int m = 0; m < n_matches; m++)
    {
      float px = frame->kp[matches[m].qi].x;
      float py = frame->kp[matches[m].qi].y;
      float qx = enrolled->kp[matches[m].ti].x;
      float qy = enrolled->kp[matches[m].ti].y;

      float pred_x = a * px + b * py + tx;
      float pred_y = c * px + d * py + ty;

      float ex = pred_x - qx;
      float ey = pred_y - qy;

      if (ex * ex + ey * ey < eps_sq)
        inliers++;
    }
  return inliers;
}

/* Least-squares refinement: re-estimate transform from inliers, re-count.
 * Uses rigid model (rotation + translation) for the refinement step to
 * prevent overfitting the affine model to noise. */
static int
ls_refine(const SigfmImgInfo *frame, const SigfmImgInfo *enrolled, const Match *matches,
          int n_matches, float a, float b, float c, float d, float tx, float ty,
          float eps_sq, float *out_a, float *out_b, float *out_c, float *out_d,
          float *out_tx, float *out_ty)
{
  /* 1. Compute centroids of inlier correspondences */
  float cx = 0, cy = 0, dx = 0, dy = 0;
  int ni = 0;
  for (int m = 0; m < n_matches; m++)
    {
      float px = frame->kp[matches[m].qi].x;
      float py = frame->kp[matches[m].qi].y;
      float qx = enrolled->kp[matches[m].ti].x;
      float qy = enrolled->kp[matches[m].ti].y;

      float pred_x = a * px + b * py + tx;
      float pred_y = c * px + d * py + ty;
      float ex = pred_x - qx;
      float ey = pred_y - qy;

      if (ex * ex + ey * ey < eps_sq)
        {
          cx += px;
          cy += py;
          dx += qx;
          dy += qy;
          ni++;
        }
    }
  if (ni < 3)
    return -1;

  cx /= ni;
  cy /= ni;
  dx /= ni;
  dy /= ni;

  /* 2. Compute optimal rotation via cross-covariance */
  float sum_cos = 0, sum_sin = 0;
  for (int m = 0; m < n_matches; m++)
    {
      float px = frame->kp[matches[m].qi].x;
      float py = frame->kp[matches[m].qi].y;
      float qx = enrolled->kp[matches[m].ti].x;
      float qy = enrolled->kp[matches[m].ti].y;

      float pred_x = a * px + b * py + tx;
      float pred_y = c * px + d * py + ty;
      float ex = pred_x - qx;
      float ey = pred_y - qy;

      if (ex * ex + ey * ey < eps_sq)
        {
          float pcx = px - cx;
          float pcy = py - cy;
          float qcx = qx - dx;
          float qcy = qy - dy;
          sum_cos += pcx * qcx + pcy * qcy;
          sum_sin += pcx * qcy - pcy * qcx;
        }
    }

  float norm = sqrtf(sum_cos * sum_cos + sum_sin * sum_sin);
  if (norm < 1e-6f)
    return -1;

  float ref_cos = sum_cos / norm;
  float ref_sin = sum_sin / norm;
  *out_a = ref_cos;
  *out_b = -ref_sin;
  *out_c = ref_sin;
  *out_d = ref_cos;
  *out_tx = dx - (ref_cos * cx - ref_sin * cy);
  *out_ty = dy - (ref_sin * cx + ref_cos * cy);

  /* 3. Re-count inliers with refined transform */
  return count_inliers(frame, enrolled, matches, n_matches, *out_a, *out_b, *out_c,
                       *out_d, *out_tx, *out_ty, eps_sq);
}

static int
ransac_score(const SigfmImgInfo *frame, const SigfmImgInfo *enrolled,
             const Match *matches, int n_matches)
{
  if (n_matches < 2)
    return 0;

  const float eps_sq = RANSAC_INLIER_THRESH * RANSAC_INLIER_THRESH;
  int best_inliers = 0;
  float best_a = 1, best_b = 0, best_c = 0, best_d = 1;
  float best_tx = 0, best_ty = 0;

  /* RANSAC: random 2-point sampling for rigid transform estimation */
  uint32_t rng = 2654435761u;
  for (int i = 0; i < n_matches && i < 8; i++)
    rng ^= (uint32_t)(matches[i].dist * 31 + matches[i].qi * 97 + matches[i].ti * 53);
  if (rng == 0)
    rng = 1;

  for (int iter = 0; iter < RANSAC_ITERATIONS; iter++)
    {
      int i1 = (int)(xorshift32(&rng) % (uint32_t)n_matches);
      int i2 = (int)(xorshift32(&rng) % (uint32_t)(n_matches - 1));
      if (i2 >= i1)
        i2++;

      float p1x = frame->kp[matches[i1].qi].x;
      float p1y = frame->kp[matches[i1].qi].y;
      float p2x = frame->kp[matches[i2].qi].x;
      float p2y = frame->kp[matches[i2].qi].y;

      float q1x = enrolled->kp[matches[i1].ti].x;
      float q1y = enrolled->kp[matches[i1].ti].y;
      float q2x = enrolled->kp[matches[i2].ti].x;
      float q2y = enrolled->kp[matches[i2].ti].y;

      float ddx = p2x - p1x, ddy = p2y - p1y;
      float len1_sq = ddx * ddx + ddy * ddy;
      if (len1_sq < RANSAC_MIN_DIST_SQ)
        continue;

      float edx = q2x - q1x, edy = q2y - q1y;
      float cos_t = (ddx * edx + ddy * edy) / len1_sq;
      float sin_t = (ddx * edy - ddy * edx) / len1_sq;
      float scale_sq = cos_t * cos_t + sin_t * sin_t;
      if (scale_sq < 0.64f || scale_sq > 1.44f)
        continue;

      float rtx = q1x - (cos_t * p1x - sin_t * p1y);
      float rty = q1y - (sin_t * p1x + cos_t * p1y);

      int inliers = count_inliers(frame, enrolled, matches, n_matches, cos_t, -sin_t,
                                  sin_t, cos_t, rtx, rty, eps_sq);

      if (inliers > best_inliers)
        {
          best_inliers = inliers;
          best_a = cos_t;
          best_b = -sin_t;
          best_c = sin_t;
          best_d = cos_t;
          best_tx = rtx;
          best_ty = rty;
          if (best_inliers >= n_matches - 1)
            break;
        }
    }

  /* Least-squares refinement from all inliers of best model */
  if (best_inliers >= 3)
    {
      float ra, rb, rc, rd, rtx, rty;
      int refined
          = ls_refine(frame, enrolled, matches, n_matches, best_a, best_b, best_c, best_d,
                      best_tx, best_ty, eps_sq, &ra, &rb, &rc, &rd, &rtx, &rty);
      if (refined > best_inliers)
        best_inliers = refined;
    }

  return best_inliers;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

SigfmImgInfo *
sigfm_extract(const SigfmPix *pix, int width, int height)
{
  /* Backward-compatible default. Per-driver tuning should call sigfm_extract_ex
   * directly with the value calibrated for that sensor. */
  return sigfm_extract_ex(pix, width, height, 128);
}

SigfmImgInfo *
sigfm_extract_ex(const SigfmPix *pix, int width, int height, int max_kp)
{
  if (max_kp <= 0)
    return NULL;

  init_brief_pattern();

  /* Blur for stable keypoint detection, but compute descriptors on the
   * original (unblurred) image to preserve discriminative pixel detail. */
  uint8_t *blurred = g_malloc((gsize)(width * height));
  box3_blur(pix, blurred, width, height);

  /* --- Scale 0 (full resolution) --- */
  OrbKeypoint *raw_kp = g_malloc((gsize)max_kp * sizeof(OrbKeypoint));
  int n = detect_keypoints(blurred, width, height, raw_kp, max_kp);

  /* --- Scale 1 (half resolution, 2-level pyramid) --- */
  int hw = width / 2, hh = height / 2;
  if (hw >= 16 && hh >= 16 && n < max_kp)
    {
      uint8_t *half = g_malloc((gsize)(hw * hh));
      downsample_2x(blurred, width, height, half);

      OrbKeypoint *half_kp = g_malloc((gsize)max_kp * sizeof(OrbKeypoint));
      int nh = detect_keypoints_ex(half, hw, hh, half_kp, max_kp, FAST_BORDER_DETECT);

      /* Map half-res keypoints to full-res coords and merge */
      for (int i = 0; i < nh && n < max_kp; i++)
        {
          float fx = half_kp[i].x * 2.0f + 0.5f;
          float fy = half_kp[i].y * 2.0f + 0.5f;

          /* BRIEF clearance check at full resolution */
          if ((int)fx < PATCH_HALF || (int)fx >= width - PATCH_HALF
              || (int)fy < PATCH_HALF || (int)fy >= height - PATCH_HALF)
            continue;

          /* Near-duplicate suppression: skip if within 4px of
           * an existing full-res keypoint */
          int dup = 0;
          for (int j = 0; j < n; j++)
            {
              float ddx = raw_kp[j].x - fx;
              float ddy = raw_kp[j].y - fy;
              if (ddx * ddx + ddy * ddy < MULTISCALE_DUP_DIST_SQ)
                {
                  dup = 1;
                  break;
                }
            }
          if (!dup)
            {
              raw_kp[n].x = fx;
              raw_kp[n].y = fy;
              raw_kp[n].response = half_kp[i].response;
              n++;
            }
        }
      g_free(half_kp);
      g_free(half);
    }

  g_free(blurred);

  if (n == 0)
    {
      g_free(raw_kp);
      return NULL;
    }

  SigfmImgInfo *info = g_new (SigfmImgInfo, 1);

  info->n_kp = n;
  info->kp = g_malloc((gsize)n * sizeof(OrbKeypoint));
  info->desc = g_malloc((gsize)n * DESC_BYTES);

  memcpy(info->kp, raw_kp, (size_t)n * sizeof(OrbKeypoint));
  g_free(raw_kp);

  /* Compute BRIEF descriptors on original image -- the driver's unsharp
   * mask has already enhanced ridge contrast; re-blurring would undo it. */
  for (int i = 0; i < n; i++)
    compute_descriptor(pix, width, &info->kp[i], info->desc + i * DESC_BYTES);

  return info;
}

int
sigfm_match_score(SigfmImgInfo *frame, SigfmImgInfo *enrolled)
{
  if (!frame || !enrolled || frame->n_kp == 0 || enrolled->n_kp == 0)
    return 0;

  int max_m = frame->n_kp;
  Match *matches = g_malloc((gsize)max_m * sizeof(Match));

  int n = knn_match(frame, enrolled, matches, max_m);
  n = cross_check_filter(frame, enrolled, matches, n);
  if (n < MIN_MATCH)
    {
      g_free(matches);
      return 0;
    }

  int score = ransac_score(frame, enrolled, matches, n);
  g_free(matches);
  return score;
}

void
sigfm_free_info(SigfmImgInfo *info)
{
  if (!info)
    return;
  g_free(info->kp);
  g_free(info->desc);
  g_free(info);
}

SigfmImgInfo *
sigfm_copy_info(SigfmImgInfo *info)
{
  if (!info)
    return NULL;

  SigfmImgInfo *copy = g_new (SigfmImgInfo, 1);

  copy->n_kp = info->n_kp;
  copy->kp = g_malloc((gsize)info->n_kp * sizeof(OrbKeypoint));
  copy->desc = g_malloc((gsize)info->n_kp * DESC_BYTES);

  memcpy(copy->kp, info->kp, (size_t)info->n_kp * sizeof(OrbKeypoint));
  memcpy(copy->desc, info->desc, (size_t)info->n_kp * DESC_BYTES);
  return copy;
}

int
sigfm_keypoints_count(SigfmImgInfo *info)
{
  return info ? info->n_kp : 0;
}

/* -------------------------------------------------------------------------
 * Binary serialisation
 *
 * Format (little-endian):
 *   [4]  magic  0x4D474653  ("SIGM")
 *   [4]  version = 2  (1 was the C++ binary format)
 *   [4]  n_kp
 *   [n_kp * 12]  keypoints: float x, float y, float response
 *   [n_kp * 32]  descriptors
 * ---------------------------------------------------------------------- */

#define SIGFM_MAGIC 0x4D474653u
#define SIGFM_VERSION 2u

static void
write_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xffu);
  p[1] = (uint8_t)((v >> 8) & 0xffu);
  p[2] = (uint8_t)((v >> 16) & 0xffu);
  p[3] = (uint8_t)((v >> 24) & 0xffu);
}

static uint32_t
read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

unsigned char *
sigfm_serialize_binary(SigfmImgInfo *info, int *outlen)
{
  if (!info)
    return NULL;

  int kp_bytes = info->n_kp * (int)(3 * sizeof(float));
  int desc_bytes = info->n_kp * DESC_BYTES;
  int total = 12 + kp_bytes + desc_bytes; /* 3 x u32 header */

  uint8_t *buf = g_malloc((gsize)total);

  write_u32_le(buf + 0, SIGFM_MAGIC);
  write_u32_le(buf + 4, SIGFM_VERSION);
  write_u32_le(buf + 8, (uint32_t)info->n_kp);

  uint8_t *p = buf + 12;
  for (int i = 0; i < info->n_kp; i++)
    {
      memcpy(p, &info->kp[i].x, sizeof(float));
      p += sizeof(float);
      memcpy(p, &info->kp[i].y, sizeof(float));
      p += sizeof(float);
      memcpy(p, &info->kp[i].response, sizeof(float));
      p += sizeof(float);
    }
  memcpy(p, info->desc, (size_t)desc_bytes);

  *outlen = total;
  return buf;
}

SigfmImgInfo *
sigfm_deserialize_binary(const unsigned char *bytes, int len)
{
  if (!bytes || len < 12)
    return NULL;

  if (read_u32_le(bytes) != SIGFM_MAGIC || read_u32_le(bytes + 4) != SIGFM_VERSION)
    return NULL;

  int n_kp = (int)read_u32_le(bytes + 8);
  if (n_kp <= 0 || n_kp > SIGFM_DESERIALIZE_KP_LIMIT)
    return NULL;

  int kp_bytes = n_kp * (int)(3 * sizeof(float));
  int desc_bytes = n_kp * DESC_BYTES;
  if (len < 12 + kp_bytes + desc_bytes)
    return NULL;

  SigfmImgInfo *info = g_new (SigfmImgInfo, 1);

  info->n_kp = n_kp;
  info->kp = g_malloc((gsize)n_kp * sizeof(OrbKeypoint));
  info->desc = g_malloc((gsize)desc_bytes);

  const uint8_t *p = bytes + 12;
  for (int i = 0; i < n_kp; i++)
    {
      memcpy(&info->kp[i].x, p, sizeof(float));
      p += sizeof(float);
      memcpy(&info->kp[i].y, p, sizeof(float));
      p += sizeof(float);
      memcpy(&info->kp[i].response, p, sizeof(float));
      p += sizeof(float);
    }
  memcpy(info->desc, p, (size_t)desc_bytes);

  return info;
}
