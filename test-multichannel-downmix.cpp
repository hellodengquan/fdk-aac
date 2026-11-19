/* -----------------------------------------------------------------------------
 * Multichannel AAC decoding / PCM downmix unit tests.
 *
 * These tests exercise the PCM downmix module (pcmdmx_lib.cpp) and the full
 * AAC encoder -> decoder pipeline with restricted output channel counts.
 *
 * The guiding rule throughout is: the channel *labels*
 * (AUDIO_CHANNEL_TYPE / channel index reported in CStreamInfo or passed into
 * pcmDmx_ApplyFrame) and the actual PCM sample order must always describe the
 * same data. A reorder of the label table without a matching reorder of the
 * audio samples (or vice versa) is a failure. Downmix weights, amplitude
 * scaling and saturation are checked against documented fixed-point behaviour
 * so that the numeric results stay explainable.
 * ---------------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aacdecoder_lib.h"
#include "aacenc_lib.h"
#include "FDK_audio.h"
#include "pcmdmx_lib.h"
#include "syslib_channelMapDescr.h"

/* ----------------------------- test framework ----------------------------- */

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, ...)                                                     \
  do {                                                                       \
    g_checks++;                                                              \
    if (!(cond)) {                                                           \
      g_failures++;                                                          \
      printf("  FAIL [%s:%d] ", __FILE__, __LINE__);                         \
      printf(__VA_ARGS__);                                                   \
      printf("\n");                                                          \
    }                                                                        \
  } while (0)

static void section(const char *name) {
  printf("\n=== %s ===\n", name);
}

/* Q1.31 (FIXP_DBL) helpers. The downmix module stores PCM in FIXP_DBL. */
static double dbl_to_real(FIXP_DBL v) {
  return (double)v / (double)(1L << 31);
}

static FIXP_DBL real_to_dbl(double v) {
  if (v >= 1.0) return (FIXP_DBL)0x7fffffff;
  if (v <= -1.0) return (FIXP_DBL)0x80000000;
  return (FIXP_DBL)lround(v * (double)(1L << 31));
}

static const char *ch_type_name(AUDIO_CHANNEL_TYPE t) {
  switch (t) {
    case ACT_NONE: return "NONE";
    case ACT_FRONT: return "FRONT";
    case ACT_SIDE: return "SIDE";
    case ACT_BACK: return "BACK";
    case ACT_LFE: return "LFE";
    case ACT_FRONT_TOP: return "FRONT_TOP";
    case ACT_SIDE_TOP: return "SIDE_TOP";
    case ACT_BACK_TOP: return "BACK_TOP";
    default: return "??";
  }
}

/* ------------------------- channel layout helpers ------------------------ */

/*
 * A labelled multichannel frame. The label arrays (type/index) describe the
 * semantic speaker position of each PCM slot. Tests keep an independent copy
 * of the labels so they can prove that the module updates labels and PCM
 * consistently.
 */
struct LabeledLayout {
  int nChannels;
  AUDIO_CHANNEL_TYPE type[8];
  UCHAR index[8];
  const char *name[8];
};

/* 5.1 in MPEG PCE order: C, L, R, Ls, Rs, LFE (channelConfiguration 6). */
static LabeledLayout layout_51_mpeg(void) {
  LabeledLayout l = {};
  l.nChannels = 6;
  l.type[0] = ACT_FRONT; l.index[0] = 0; l.name[0] = "C";
  l.type[1] = ACT_FRONT; l.index[1] = 1; l.name[1] = "L";
  l.type[2] = ACT_FRONT; l.index[2] = 2; l.name[2] = "R";
  l.type[3] = ACT_BACK;  l.index[3] = 0; l.name[3] = "Ls";
  l.type[4] = ACT_BACK;  l.index[4] = 1; l.name[4] = "Rs";
  l.type[5] = ACT_LFE;   l.index[5] = 0; l.name[5] = "LFE";
  return l;
}

/* 5.1 in WAV order: L, R, C, LFE, Ls, Rs (what AAC_PCM_OUTPUT_CHANNEL_MAPPING
 * = 1 produces). */
static LabeledLayout layout_51_wav(void) {
  LabeledLayout l = {};
  l.nChannels = 6;
  l.type[0] = ACT_FRONT; l.index[0] = 1; l.name[0] = "L";
  l.type[1] = ACT_FRONT; l.index[1] = 2; l.name[1] = "R";
  l.type[2] = ACT_FRONT; l.index[2] = 0; l.name[2] = "C";
  l.type[3] = ACT_LFE;   l.index[3] = 0; l.name[3] = "LFE";
  l.type[4] = ACT_BACK;  l.index[4] = 0; l.name[4] = "Ls";
  l.type[5] = ACT_BACK;  l.index[5] = 1; l.name[5] = "Rs";
  return l;
}

/* Mono: single front channel. */
static LabeledLayout layout_mono(void) {
  LabeledLayout l = {};
  l.nChannels = 1;
  l.type[0] = ACT_FRONT; l.index[0] = 0; l.name[0] = "C";
  return l;
}

/*
 * Fill one input channel with a distinctive level (one channel active at a
 * time). The level is kept well below full scale so ordinary downmixing never
 * saturates. Returns the real-valued amplitude used.
 */
static double fill_impulse_channel(FIXP_DBL *buf /* non-interleaved */,
                                   int frameSize, int nChannels,
                                   int activeChannel, double amplitude) {
  for (int ch = 0; ch < nChannels; ch++) {
    FIXP_DBL v = (ch == activeChannel) ? real_to_dbl(amplitude) : (FIXP_DBL)0;
    for (int s = 0; s < frameSize; s++) buf[ch * frameSize + s] = v;
  }
  return amplitude;
}

/*
 * Run the downmix module once on a labelled, non-interleaved buffer and
 * snapshot the output labels. mapPassThrough selects MPEG (1) vs WAV (0)
 * output ordering.
 */
struct DmxRunResult {
  PCMDMX_ERROR err;
  int outChannels;
  AUDIO_CHANNEL_TYPE outType[8];
  UCHAR outIndex[8];
  INT outScale;
};

static DmxRunResult run_dmx(HANDLE_PCM_DOWNMIX h, FIXP_DBL *buf, int bufSize,
                            int frameSize, LabeledLayout in,
                            int mapPassThrough) {
  FDK_channelMapDescr mapDescr;
  FDK_chMapDescr_init(&mapDescr, NULL, 0, mapPassThrough ? 1 : 0);

  DmxRunResult r = {};
  AUDIO_CHANNEL_TYPE chType[8];
  UCHAR chIndex[8];
  for (int i = 0; i < in.nChannels; i++) {
    chType[i] = in.type[i];
    chIndex[i] = in.index[i];
  }
  INT nCh = in.nChannels;
  INT scale = 0;
  r.err = pcmDmx_ApplyFrame(h, buf, bufSize, frameSize, &nCh,
                            0 /* non-interleaved */, chType, chIndex,
                            &mapDescr, &scale);
  r.outChannels = nCh;
  r.outScale = scale;
  memcpy(r.outType, chType, sizeof(r.outType));
  memcpy(r.outIndex, chIndex, sizeof(r.outIndex));
  return r;
}

/* Find the PCM slot whose label matches (type,index); returns -1 if absent. */
static int find_labelled_slot(const DmxRunResult &r, AUDIO_CHANNEL_TYPE type,
                              UCHAR index) {
  for (int i = 0; i < r.outChannels; i++) {
    if (r.outType[i] == type && r.outIndex[i] == index) return i;
  }
  return -1;
}

/* ----------------- Section 1: label / PCM order consistency --------------- */

/*
 * For every 5.1 -> stereo downmix, each of the six input channels is excited
 * on its own. The output slot that actually carries the contribution must be
 * the slot whose *label* names the corresponding downmix speaker (L or R).
 * This proves the format labels and the PCM ordering agree, for both WAV and
 * MPEG input ordering.
 */
static void test_51_stereo_label_pcm_consistency(int mapPassThrough,
                                                 const char *orderName) {
  section(orderName);
  const int frameSize = 256;
  const int nIn = 6;
  const double amp = 0.25;

  HANDLE_PCM_DOWNMIX h = NULL;
  PCMDMX_ERROR err = pcmDmx_Open(&h);
  CHECK(err == PCMDMX_OK, "pcmDmx_Open failed: %d", err);
  err = pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2);
  CHECK(err == PCMDMX_OK, "set MAX=2 failed: %d", err);

  LabeledLayout in = mapPassThrough ? layout_51_mpeg() : layout_51_wav();

  for (int active = 0; active < nIn; active++) {
    FIXP_DBL buf[8 * 256] = {};
    fill_impulse_channel(buf, frameSize, nIn, active, amp);

    DmxRunResult r = run_dmx(h, buf, 8 * frameSize, frameSize, in,
                             mapPassThrough);
    CHECK(r.err == PCMDMX_OK, "channel %s: ApplyFrame err %d",
          in.name[active], r.err);
    CHECK(r.outChannels == 2, "channel %s: expected 2 out, got %d",
          in.name[active], r.outChannels);

    int slotL = find_labelled_slot(r, ACT_FRONT, 0);
    int slotR = find_labelled_slot(r, ACT_FRONT, 1);
    CHECK(slotL >= 0 && slotR >= 0,
          "channel %s: stereo labels missing (L slot %d R slot %d)",
          in.name[active], slotL, slotR);
    CHECK(slotL != slotR, "channel %s: L and R resolved to same slot %d",
          in.name[active], slotL);

    double lvl[2] = {0.0, 0.0};
    for (int s = 0; s < frameSize; s++) {
      lvl[0] += fabs(dbl_to_real(buf[slotL * frameSize + s]));
      lvl[1] += fabs(dbl_to_real(buf[slotR * frameSize + s]));
    }

    /* LFE must be discarded in the default MPEG AMD4 stereo downmix. */
    if (in.type[active] == ACT_LFE) {
      CHECK(lvl[0] == 0.0 && lvl[1] == 0.0,
            "LFE leaked into stereo output (L=%g R=%g)", lvl[0], lvl[1]);
      continue;
    }

    int totalNonZero = (lvl[0] > 0.0) + (lvl[1] > 0.0);
    CHECK(totalNonZero > 0,
          "channel %s: contribution vanished after downmix", in.name[active]);

    /* The active channel's energy must appear in the slot labelled with the
     * speaker it is mixed into, never only in the other/unlabelled slot. */
    if (in.type[active] == ACT_FRONT && in.index[active] == 1) {
      /* L -> only labelled L slot */
      CHECK(lvl[0] > 0.0 && lvl[1] == 0.0,
            "L source landed outside L-labelled slot (L=%g R=%g)", lvl[0],
            lvl[1]);
    } else if (in.type[active] == ACT_FRONT && in.index[active] == 2) {
      /* R -> only labelled R slot */
      CHECK(lvl[1] > 0.0 && lvl[0] == 0.0,
            "R source landed outside R-labelled slot (L=%g R=%g)", lvl[0],
            lvl[1]);
    } else if (in.type[active] == ACT_FRONT && in.index[active] == 0) {
      /* C -> both L and R slots, symmetrically */
      CHECK(lvl[0] > 0.0 && lvl[1] > 0.0,
            "C source missing from one stereo slot (L=%g R=%g)", lvl[0],
            lvl[1]);
      double ratio = fabs(lvl[0] - lvl[1]) / (lvl[0] + lvl[1]);
      CHECK(ratio < 0.02, "C pan not symmetric, L/R relative diff %g", ratio);
    } else if (in.type[active] == ACT_BACK && in.index[active] == 0) {
      /* Ls -> L slot only (non-pseudo-surround default) */
      CHECK(lvl[0] > 0.0 && lvl[1] == 0.0,
            "Ls source landed outside L-labelled slot (L=%g R=%g)", lvl[0],
            lvl[1]);
    } else if (in.type[active] == ACT_BACK && in.index[active] == 1) {
      /* Rs -> R slot only */
      CHECK(lvl[1] > 0.0 && lvl[0] == 0.0,
            "Rs source landed outside R-labelled slot (L=%g R=%g)", lvl[0],
            lvl[1]);
    }
  }
  pcmDmx_Close(&h);
}

/*
 * 5.1 -> mono: the single output slot must be labelled as a front channel and
 * the mono mix must contain L+R contributions (center and surrounds are folded
 * via the stereo stage, LFE discarded by the default path).
 */
static void test_51_mono_label_pcm_consistency(void) {
  section("5.1 -> mono label/PCM consistency");
  const int frameSize = 256;
  const int nIn = 6;
  const double amp = 0.25;

  HANDLE_PCM_DOWNMIX h = NULL;
  CHECK(pcmDmx_Open(&h) == PCMDMX_OK, "open");
  CHECK(pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 1) == PCMDMX_OK,
        "set MAX=1");

  LabeledLayout in = layout_51_wav();
  double sawL = 0.0, sawR = 0.0, sawLfe = 0.0;
  for (int active = 0; active < nIn; active++) {
    FIXP_DBL buf[8 * 256] = {};
    fill_impulse_channel(buf, frameSize, nIn, active, amp);
    DmxRunResult r = run_dmx(h, buf, 8 * frameSize, frameSize, in, 0);
    CHECK(r.err == PCMDMX_OK, "ch %s mono err %d", in.name[active], r.err);
    CHECK(r.outChannels == 1, "ch %s expected 1 out got %d", in.name[active],
          r.outChannels);
    CHECK(r.outType[0] == ACT_FRONT, "mono output not labelled FRONT (%s)",
          ch_type_name(r.outType[0]));
    double lvl = 0.0;
    for (int s = 0; s < frameSize; s++)
      lvl += fabs(dbl_to_real(buf[s]));
    if (in.type[active] == ACT_FRONT &&
        (in.index[active] == 1 || in.index[active] == 2)) {
      CHECK(lvl > 0.0, "mono mix lost discrete L/R source %s", in.name[active]);
    }
    if (in.type[active] == ACT_LFE) sawLfe += lvl;
    if (in.type[active] == ACT_FRONT && in.index[active] == 1) sawL += lvl;
    if (in.type[active] == ACT_FRONT && in.index[active] == 2) sawR += lvl;
  }
  CHECK(sawLfe == 0.0, "LFE leaked into mono output (%g)", sawLfe);
  CHECK(sawL > 0.0 && sawR > 0.0,
        "mono output does not contain both L and R energy (L=%g R=%g)", sawL,
        sawR);
  pcmDmx_Close(&h);
}

/*
 * The channel map descriptor itself must express the WAV <-> MPEG reorder.
 * pcmDmx_ApplyFrame only consults it while downmixing/upmixing; in bypass it
 * returns early and leaves the buffer untouched, so the reorder contract is
 * checked here against the documented default (WAV) table for channel
 * configuration 6: MPEG order C,L,R,Ls,Rs,LFE maps to WAV slots
 * 2,0,1,4,5,3 (L,R,C,Ls,Rs,LFE).
 */
static void test_channel_map_descriptor_reorder(void) {
  section("channel map descriptor: WAV table vs MPEG pass-through");
  FDK_channelMapDescr wav, mpeg;
  FDK_chMapDescr_init(&wav, NULL, 0, 0 /* apply WAV table */);
  FDK_chMapDescr_init(&mpeg, NULL, 0, 1 /* pass-through MPEG order */);
  CHECK(FDK_chMapDescr_isValid(&wav) && FDK_chMapDescr_isValid(&mpeg),
        "map descriptors invalid");

  /* chCfg 6 (5.1): MPEG semantic index -> WAV PCM slot. */
  const UCHAR expectedWav[6] = {2, 0, 1, 4, 5, 3};
  int moved = 0;
  for (UCHAR ch = 0; ch < 6; ch++) {
    UCHAR wavSlot = FDK_chMapDescr_getMapValue(&wav, ch, 6);
    UCHAR mpegSlot = FDK_chMapDescr_getMapValue(&mpeg, ch, 6);
    CHECK(wavSlot == expectedWav[ch],
          "chCfg6 MPEG ch %d -> WAV slot %d, expected %d", ch, wavSlot,
          expectedWav[ch]);
    CHECK(mpegSlot == ch, "pass-through changed ch %d to slot %d", ch,
          mpegSlot);
    if (wavSlot != mpegSlot) moved++;
  }
  CHECK(moved > 0, "WAV table identical to MPEG pass-through");

  /* The WAV map must be a permutation (unique destinations). */
  int seen[6] = {};
  for (UCHAR ch = 0; ch < 6; ch++)
    seen[FDK_chMapDescr_getMapValue(&wav, ch, 6)]++;
  for (int i = 0; i < 6; i++)
    CHECK(seen[i] == 1, "WAV chCfg6 map not a permutation at slot %d (%d)", i,
          seen[i]);
}

/* ------------- Section 2: downmix weights are explainable ---------------- */

/*
 * Extract the effective 5.1 -> stereo coefficient for one input channel by
 * exciting it alone. The measured real-valued coefficient must match the
 * documented MPEG-4 AMD4 default (clev = slev = 0.707, identity for L/R,
 * zero for LFE) within fixed-point tolerance.
 */
static void test_51_stereo_default_weights(void) {
  section("5.1 -> stereo default downmix weights (MPEG AMD4)");
  const int frameSize = 128;
  const int nIn = 6;
  const double amp = 0.5;

  HANDLE_PCM_DOWNMIX h = NULL;
  CHECK(pcmDmx_Open(&h) == PCMDMX_OK, "open");
  CHECK(pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "MAX=2");

  LabeledLayout in = layout_51_mpeg();
  struct {
    const char *name;
    AUDIO_CHANNEL_TYPE type;
    UCHAR idx;
    double expectL;
    double expectR;
  } cases[] = {
      {"L", ACT_FRONT, 1, 1.0, 0.0},   {"R", ACT_FRONT, 2, 0.0, 1.0},
      {"C", ACT_FRONT, 0, 0.707, 0.707}, {"Ls", ACT_BACK, 0, 0.707, 0.0},
      {"Rs", ACT_BACK, 1, 0.0, 0.707}, {"LFE", ACT_LFE, 0, 0.0, 0.0},
  };

  for (auto &c : cases) {
    int active = -1;
    for (int i = 0; i < nIn; i++)
      if (in.type[i] == c.type && in.index[i] == c.idx) active = i;
    CHECK(active >= 0, "layout missing %s", c.name);
    FIXP_DBL buf[8 * 128] = {};
    fill_impulse_channel(buf, frameSize, nIn, active, amp);
    DmxRunResult r = run_dmx(h, buf, 8 * frameSize, frameSize, in, 1);
    CHECK(r.err == PCMDMX_OK, "%s ApplyFrame err %d", c.name, r.err);

    int slotL = find_labelled_slot(r, ACT_FRONT, 0);
    int slotR = find_labelled_slot(r, ACT_FRONT, 1);
    double outL = dbl_to_real(buf[slotL * frameSize]);
    double outR = dbl_to_real(buf[slotR * frameSize]);
    /* With an external scale pointer the module leaves the final 2^scale
     * headroom shift to the caller, so the effective coefficient is
     * (output / input) * 2^outScale. */
    double scaleComp = (double)(1L << r.outScale);
    double coefL = outL / amp * scaleComp;
    double coefR = outR / amp * scaleComp;
    const double tol = 0.02;
    CHECK(fabs(coefL - c.expectL) <= tol,
          "%s -> L coefficient %g, expected %g (outScale %d)", c.name, coefL,
          c.expectL, r.outScale);
    CHECK(fabs(coefR - c.expectR) <= tol,
          "%s -> R coefficient %g, expected %g (outScale %d)", c.name, coefR,
          c.expectR, r.outScale);
    if (c.expectL > 0.0 || c.expectR > 0.0) {
      CHECK(outL >= 0.0 && outR >= 0.0,
            "%s downmix inverted sign unexpectedly", c.name);
    }
  }
  pcmDmx_Close(&h);
}

/*
 * Linearity: the output for two simultaneous sources must equal the sum of
 * the individually measured outputs (same fixed-point rounding path). This
 * guards against hidden per-channel clamps that would make the matrix
 * weights meaningless.
 */
static void test_downmix_linearity(void) {
  section("downmix linearity (L + C together == L alone + C alone)");
  const int frameSize = 128;
  const double ampL = 0.2, ampC = 0.15;

  HANDLE_PCM_DOWNMIX h1 = NULL, h2 = NULL;
  CHECK(pcmDmx_Open(&h1) == PCMDMX_OK, "open1");
  CHECK(pcmDmx_Open(&h2) == PCMDMX_OK, "open2");
  CHECK(pcmDmx_SetParam(h1, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "max1");
  CHECK(pcmDmx_SetParam(h2, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "max2");

  LabeledLayout in = layout_51_mpeg();
  FIXP_DBL combined[8 * 128] = {};
  int slotL_in = 1, slotC_in = 0; /* MPEG order: C=0, L=1 */
  for (int s = 0; s < frameSize; s++) {
    combined[slotL_in * frameSize + s] = real_to_dbl(ampL);
    combined[slotC_in * frameSize + s] = real_to_dbl(ampC);
  }

  DmxRunResult rComb = run_dmx(h1, combined, 8 * frameSize, frameSize, in, 1);
  CHECK(rComb.err == PCMDMX_OK, "combined err %d", rComb.err);

  /* Individual contributions into fresh buffers. */
  FIXP_DBL onlyL[8 * 128] = {}, onlyC[8 * 128] = {};
  for (int s = 0; s < frameSize; s++) {
    onlyL[slotL_in * frameSize + s] = real_to_dbl(ampL);
    onlyC[slotC_in * frameSize + s] = real_to_dbl(ampC);
  }
  DmxRunResult rL = run_dmx(h2, onlyL, 8 * frameSize, frameSize, in, 1);
  HANDLE_PCM_DOWNMIX h3 = NULL;
  CHECK(pcmDmx_Open(&h3) == PCMDMX_OK, "open3");
  CHECK(pcmDmx_SetParam(h3, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "max3");
  DmxRunResult rC = run_dmx(h3, onlyC, 8 * frameSize, frameSize, in, 1);
  CHECK(rL.err == PCMDMX_OK && rC.err == PCMDMX_OK, "solo errs");

  int slotL = find_labelled_slot(rComb, ACT_FRONT, 0);
  for (int s = 0; s < frameSize; s++) {
    double a = dbl_to_real(combined[slotL * frameSize + s]);
    double b = dbl_to_real(onlyL[slotL * frameSize + s]) +
               dbl_to_real(onlyC[slotL * frameSize + s]);
    CHECK(fabs(a - b) <= 2.0 / (double)(1L << 31),
          "linearity broken at sample %d: %g vs %g", s, a, b);
  }
  pcmDmx_Close(&h1);
  pcmDmx_Close(&h2);
  pcmDmx_Close(&h3);
}

/* ----------- Section 3: output channel count limit enforcement ------------ */

static void test_output_channel_limits(void) {
  section("MIN/MAX output channel count enforcement");
  const int frameSize = 64;
  HANDLE_PCM_DOWNMIX h = NULL;
  CHECK(pcmDmx_Open(&h) == PCMDMX_OK, "open");

  /* Invalid values must be rejected rather than silently accepted. */
  CHECK(pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 3) ==
            PCMDMX_UNABLE_TO_SET_PARAM,
        "MAX=3 must be rejected");
  CHECK(pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 5) ==
            PCMDMX_UNABLE_TO_SET_PARAM,
        "MAX=5 must be rejected");
  CHECK(pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "MAX=2 accepted");
  CHECK(pcmDmx_SetParam(h, MIN_NUMBER_OF_OUTPUT_CHANNELS, 8) == PCMDMX_OK,
        "MIN=8 accepted");

  INT minV = 0, maxV = 0;
  pcmDmx_GetParam(h, MIN_NUMBER_OF_OUTPUT_CHANNELS, &minV);
  pcmDmx_GetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, &maxV);
  /* MIN > MAX forces MAX up to MIN per API contract. */
  CHECK(maxV == 8, "MIN>MAX should raise MAX to 8, got %d", maxV);
  CHECK(minV == 8, "MIN readback %d", minV);

  /* Reset to a known state: cap 5.1 at stereo. */
  CHECK(pcmDmx_Reset(h, PCMDMX_RESET_PARAMS) == PCMDMX_OK, "reset params");
  CHECK(pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "MAX=2 again");

  LabeledLayout in51 = layout_51_wav();
  FIXP_DBL buf[8 * 64] = {};
  for (int ch = 0; ch < 6; ch++)
    for (int s = 0; s < frameSize; s++)
      buf[ch * frameSize + s] = real_to_dbl(0.1 * (ch + 1));
  DmxRunResult r = run_dmx(h, buf, 8 * frameSize, frameSize, in51, 0);
  CHECK(r.err == PCMDMX_OK, "5.1 capped err %d", r.err);
  CHECK(r.outChannels == 2, "5.1 not capped to 2, got %d", r.outChannels);
  CHECK(r.outType[0] == ACT_FRONT && r.outType[1] == ACT_FRONT,
        "capped stereo labels wrong");

  /* Undersized output buffer must be reported, not overrun. Stereo output
   * needs 2*frameSize samples; provide room for only one channel. */
  FIXP_DBL smallBuf[1 * 64] = {};
  for (int s = 0; s < frameSize; s++) smallBuf[s] = real_to_dbl(0.1);
  INT nCh = 6;
  AUDIO_CHANNEL_TYPE ct[8];
  UCHAR ci[8];
  for (int i = 0; i < 6; i++) { ct[i] = in51.type[i]; ci[i] = in51.index[i]; }
  FDK_channelMapDescr md;
  FDK_chMapDescr_init(&md, NULL, 0, 0);
  INT scale = 0;
  PCMDMX_ERROR smallErr =
      pcmDmx_ApplyFrame(h, smallBuf, frameSize /* too small for stereo */,
                        frameSize, &nCh, 0, ct, ci, &md, &scale);
  CHECK(smallErr == PCMDMX_OUTPUT_BUFFER_TOO_SMALL,
        "small buffer returned %d", smallErr);

  pcmDmx_Close(&h);
}

/* Channel expansion (mono -> stereo/6ch): added slots labelled ACT_NONE and
 * carry silence, existing content keeps its label and samples. */
static void test_channel_expansion(void) {
  section("channel expansion (mono up to 2 and 6 channels)");
  const int frameSize = 64;
  const double amp = 0.4;

  HANDLE_PCM_DOWNMIX h = NULL;
  CHECK(pcmDmx_Open(&h) == PCMDMX_OK, "open");
  CHECK(pcmDmx_SetParam(h, MIN_NUMBER_OF_OUTPUT_CHANNELS, 6) == PCMDMX_OK,
        "MIN=6");

  LabeledLayout mono = layout_mono();
  FIXP_DBL buf[8 * 64] = {};
  for (int s = 0; s < frameSize; s++) buf[s] = real_to_dbl(amp);

  DmxRunResult r = run_dmx(h, buf, 8 * frameSize, frameSize, mono, 0);
  CHECK(r.err == PCMDMX_OK, "expand err %d", r.err);
  CHECK(r.outChannels == 6, "expected 6 expanded channels, got %d",
        r.outChannels);

  int contentSlots = 0, silentSlots = 0;
  for (int slot = 0; slot < 6; slot++) {
    double lvl = fabs(dbl_to_real(buf[slot * frameSize]));
    if (r.outType[slot] == ACT_NONE) {
      silentSlots++;
      CHECK(lvl == 0.0, "ACT_NONE slot %d not silent (%g)", slot, lvl);
    } else {
      contentSlots++;
      CHECK(lvl > 0.0, "labelled slot %d lost its content", slot);
    }
  }
  /* Mono is duplicated into an L/R pair during the first expansion stage, so
   * two labelled slots carry content and the remaining four are silent. */
  CHECK(contentSlots == 2, "expected 2 duplicated content slots, got %d",
        contentSlots);
  CHECK(silentSlots == 4, "expected 4 silent extension slots, got %d",
        silentSlots);

  /* Every non-NONE label must be unique (no duplicate speaker identity). */
  for (int i = 0; i < 6; i++) {
    if (r.outType[i] == ACT_NONE) continue;
    for (int j = i + 1; j < 6; j++) {
      CHECK(!(r.outType[j] == r.outType[i] &&
              r.outIndex[j] == r.outIndex[i]),
            "duplicate non-NONE label at slots %d and %d", i, j);
    }
  }
  pcmDmx_Close(&h);
}

/* -------- Section 4: center/LFE semantics and saturation behaviour ------- */

static void test_center_and_lfe_handling(void) {
  section("center and LFE specific handling");
  const int frameSize = 64;

  /* Center must always map to a slot labelled FRONT index 0; LFE must keep an
   * ACT_LFE label while present and be dropped (not relabelled) on a stereo
   * downmix. */
  HANDLE_PCM_DOWNMIX h = NULL;
  CHECK(pcmDmx_Open(&h) == PCMDMX_OK, "open");
  CHECK(pcmDmx_SetParam(h, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "MAX=2");

  LabeledLayout in = layout_51_wav();
  FIXP_DBL buf[8 * 64] = {};
  /* Excite only center and LFE. */
  for (int s = 0; s < frameSize; s++) {
    buf[2 * frameSize + s] = real_to_dbl(0.3); /* C at WAV slot 2 */
    buf[3 * frameSize + s] = real_to_dbl(0.3); /* LFE at WAV slot 3 */
  }
  DmxRunResult r = run_dmx(h, buf, 8 * frameSize, frameSize, in, 0);
  CHECK(r.err == PCMDMX_OK, "err %d", r.err);
  CHECK(find_labelled_slot(r, ACT_LFE, 0) < 0,
        "LFE label still present after stereo downmix");
  int cL = find_labelled_slot(r, ACT_FRONT, 0);
  int cR = find_labelled_slot(r, ACT_FRONT, 1);
  CHECK(cL >= 0 && cR >= 0, "stereo labels missing");
  double centerL = fabs(dbl_to_real(buf[cL * frameSize]));
  double centerR = fabs(dbl_to_real(buf[cR * frameSize]));
  CHECK(centerL > 0.0 && centerR > 0.0,
        "center content missing after downmix (%g/%g)", centerL, centerR);
  pcmDmx_Close(&h);
}

/*
 * Saturation must clamp to the FIXP_DBL range instead of wrapping.
 *
 * Two contracts are tested:
 *  - Internal scaling (pDmxOutScale == NULL): the module applies the headroom
 *    shift itself through SATURATE_SHIFT, so full-scale positive/negative
 *    stereo sums must come back clamped at the FIXP_DBL maxima with no sign
 *    wrap.
 *  - External scaling (pDmxOutScale != NULL): the raw sum is returned with a
 *    reported exponent. Applying that shift with the same SATURATE_SHIFT rule
 *    the decoder uses must again clamp, never wrap. The reported exponent is
 *    explainable headroom, not an arbitrary attenuation.
 */
static void run_mono_fullscale(HANDLE_PCM_DOWNMIX h, FIXP_DBL *buf,
                               int frameSize, INT *externalScale,
                               PCMDMX_ERROR *err) {
  /* The library always touches all 8 label cells internally, so these arrays
   * must stay 8 elements even for a 2-channel call. */
  AUDIO_CHANNEL_TYPE chType[8] = {ACT_FRONT, ACT_FRONT};
  UCHAR chIndex[8] = {0, 1};
  FDK_channelMapDescr md;
  FDK_chMapDescr_init(&md, NULL, 0, 1);
  INT nCh = 2;
  *err = pcmDmx_ApplyFrame(h, buf, 8 * frameSize, frameSize, &nCh, 0, chType,
                           chIndex, &md, externalScale);
}

static void test_saturation_is_clamped(void) {
  section("amplitude scaling and saturation are explainable");
  const int frameSize = 64;

  /* ---- internal scaling: NULL scale pointer ---- */
  HANDLE_PCM_DOWNMIX hInt = NULL;
  CHECK(pcmDmx_Open(&hInt) == PCMDMX_OK, "open internal");
  CHECK(pcmDmx_SetParam(hInt, MAX_NUMBER_OF_OUTPUT_CHANNELS, 1) == PCMDMX_OK,
        "MAX=1 internal");

  FIXP_DBL posBuf[8 * 64] = {};
  for (int s = 0; s < frameSize; s++) {
    posBuf[0 * frameSize + s] = (FIXP_DBL)0x7fffffff;
    posBuf[1 * frameSize + s] = (FIXP_DBL)0x7fffffff;
  }
  PCMDMX_ERROR err;
  run_mono_fullscale(hInt, posBuf, frameSize, NULL, &err);
  CHECK(err == PCMDMX_OK, "internal positive err %d", err);
  int wrapped = 0;
  FIXP_DBL peak = 0;
  for (int s = 0; s < frameSize; s++) {
    FIXP_DBL v = posBuf[s];
    if (v < 0) wrapped++;
    if (v > peak) peak = v;
  }
  CHECK(wrapped == 0, "positive full-scale sum wrapped negative (%d samples)",
        wrapped);
  CHECK(peak == (FIXP_DBL)0x7fffffff,
        "internal mode did not clamp at positive max (0x%08x)",
        (unsigned)peak);

  FIXP_DBL negBuf[8 * 64] = {};
  for (int s = 0; s < frameSize; s++) {
    negBuf[0 * frameSize + s] = (FIXP_DBL)0x80000000;
    negBuf[1 * frameSize + s] = (FIXP_DBL)0x80000000;
  }
  run_mono_fullscale(hInt, negBuf, frameSize, NULL, &err);
  CHECK(err == PCMDMX_OK, "internal negative err %d", err);
  int posWrap = 0;
  FIXP_DBL trough = 0;
  for (int s = 0; s < frameSize; s++) {
    FIXP_DBL v = negBuf[s];
    if (v > 0) posWrap++;
    if (s == 0 || v < trough) trough = v;
  }
  CHECK(posWrap == 0, "negative full-scale sum wrapped positive");
  CHECK(trough == (FIXP_DBL)0x80000000,
        "internal mode did not clamp at negative min (0x%08x)",
        (unsigned)trough);
  pcmDmx_Close(&hInt);

  /* ---- external scaling: exponent reported, caller shifts with saturation
   * ---- */
  HANDLE_PCM_DOWNMIX hExt = NULL;
  CHECK(pcmDmx_Open(&hExt) == PCMDMX_OK, "open external");
  CHECK(pcmDmx_SetParam(hExt, MAX_NUMBER_OF_OUTPUT_CHANNELS, 1) == PCMDMX_OK,
        "MAX=1 external");

  FIXP_DBL extBuf[8 * 64] = {};
  for (int s = 0; s < frameSize; s++) {
    extBuf[0 * frameSize + s] = (FIXP_DBL)0x7fffffff;
    extBuf[1 * frameSize + s] = (FIXP_DBL)0x7fffffff;
  }
  INT extScale = 0;
  run_mono_fullscale(hExt, extBuf, frameSize, &extScale, &err);
  CHECK(err == PCMDMX_OK, "external err %d", err);
  CHECK(extScale >= 0 && extScale <= 3, "unexpected headroom exponent %d",
        extScale);
  /* Apply the same final operation the decoder performs. */
  int extWrapped = 0;
  FIXP_DBL extPeak = 0;
  for (int s = 0; s < frameSize; s++) {
    FIXP_DBL v = (FIXP_DBL)SATURATE_SHIFT(extBuf[s], -extScale, DFRACT_BITS);
    if (v < 0) extWrapped++;
    if (v > extPeak) extPeak = v;
  }
  CHECK(extWrapped == 0, "caller-side shift wrapped sign");
  CHECK(extPeak == (FIXP_DBL)0x7fffffff,
        "caller-side scaling did not clamp (0x%08x, scale %d)",
        (unsigned)extPeak, extScale);
  pcmDmx_Close(&hExt);
}

/* Small-signal gain must remain proportional (no surprise attenuation). */
static void test_small_signal_scaling(void) {
  section("small-signal amplitude scaling stays proportional");
  const int frameSize = 64;

  HANDLE_PCM_DOWNMIX h1 = NULL, h2 = NULL;
  CHECK(pcmDmx_Open(&h1) == PCMDMX_OK && pcmDmx_Open(&h2) == PCMDMX_OK,
        "open");
  CHECK(pcmDmx_SetParam(h1, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "max1");
  CHECK(pcmDmx_SetParam(h2, MAX_NUMBER_OF_OUTPUT_CHANNELS, 2) == PCMDMX_OK,
        "max2");

  LabeledLayout in = layout_51_mpeg();
  FIXP_DBL b1[8 * 64] = {}, b2[8 * 64] = {};
  double a1 = 0.1, a2 = 0.4;
  for (int s = 0; s < frameSize; s++) {
    b1[1 * frameSize + s] = real_to_dbl(a1); /* L */
    b2[1 * frameSize + s] = real_to_dbl(a2);
  }
  DmxRunResult r1 = run_dmx(h1, b1, 8 * frameSize, frameSize, in, 1);
  DmxRunResult r2 = run_dmx(h2, b2, 8 * frameSize, frameSize, in, 1);
  int slot1 = find_labelled_slot(r1, ACT_FRONT, 0);
  int slot2 = find_labelled_slot(r2, ACT_FRONT, 0);
  double o1 = dbl_to_real(b1[slot1 * frameSize]);
  double o2 = dbl_to_real(b2[slot2 * frameSize]);
  CHECK(o1 > 0.0 && o2 > 0.0, "no output for small signal");
  double ratio = (o2 / a2) / (o1 / a1);
  CHECK(fabs(ratio - 1.0) < 0.01,
        "gain not constant across amplitudes (ratio %g)", ratio);
  pcmDmx_Close(&h1);
  pcmDmx_Close(&h2);
}

/* ------- Section 5: full AAC encode/decode pipeline with downmix --------- */

struct EncDecResult {
  int ok;
  int outChannels;
  AUDIO_CHANNEL_TYPE type[8];
  UCHAR index[8];
  /* Average absolute PCM level per output slot. */
  double level[8];
};

struct OneShotEncoder {
  HANDLE_AACENCODER enc;
  AACENC_InfoStruct info;
  int sampleRate;
  int nChannels;
  int frameLen;
};

/* Encode one frame of multichannel sine waves; returns payload size or 0. */
static int encode_sine_frame(OneShotEncoder &e, int frameIndex, UCHAR *bsBuf,
                             int bsBufCap) {
  const double baseFreq = 220.0, freqStep = 110.0;
  INT_PCM *inPcm =
      (INT_PCM *)calloc(e.nChannels * e.frameLen, sizeof(INT_PCM));
  for (int s = 0; s < e.frameLen; s++) {
    for (int ch = 0; ch < e.nChannels; ch++) {
      double freq = baseFreq + freqStep * ch;
      double v = 0.2 * sin(2.0 * M_PI * freq *
                           (s + frameIndex * e.frameLen) / e.sampleRate);
      inPcm[s * e.nChannels + ch] = (INT_PCM)real_to_dbl(v);
    }
  }
  void *inPtr = inPcm;
  int inSize = e.nChannels * e.frameLen * (int)sizeof(INT_PCM);
  int inElSize = sizeof(INT_PCM);
  int inId = IN_AUDIO_DATA;
  AACENC_BufDesc inBuf = {};
  inBuf.numBufs = 1;
  inBuf.bufs = &inPtr;
  inBuf.bufferIdentifiers = &inId;
  inBuf.bufSizes = &inSize;
  inBuf.bufElSizes = &inElSize;

  void *outPtr = bsBuf;
  int outSize = bsBufCap;
  int outElSize = 1;
  int outId = OUT_BITSTREAM_DATA;
  AACENC_BufDesc outBufDesc = {};
  outBufDesc.numBufs = 1;
  outBufDesc.bufs = &outPtr;
  outBufDesc.bufferIdentifiers = &outId;
  outBufDesc.bufSizes = &outSize;
  outBufDesc.bufElSizes = &outElSize;
  AACENC_InArgs inArgs = {};
  inArgs.numInSamples = e.nChannels * e.frameLen;
  AACENC_OutArgs outArgs = {};
  AACENC_ERROR eErr =
      aacEncEncode(e.enc, &inBuf, &outBufDesc, &inArgs, &outArgs);
  free(inPcm);
  if (eErr != AACENC_OK) return 0;
  return outArgs.numOutBytes;
}

static int open_encoder(OneShotEncoder &e, int nChannels,
                        CHANNEL_MODE chMode, int wavInputOrder) {
  memset(&e, 0, sizeof(e));
  e.sampleRate = 44100;
  e.nChannels = nChannels;
  if (aacEncOpen(&e.enc, 0, nChannels) != AACENC_OK) return 0;
  aacEncoder_SetParam(e.enc, AACENC_AOT, 2);
  aacEncoder_SetParam(e.enc, AACENC_SAMPLERATE, e.sampleRate);
  aacEncoder_SetParam(e.enc, AACENC_CHANNELMODE, chMode);
  aacEncoder_SetParam(e.enc, AACENC_CHANNELORDER, wavInputOrder ? 1 : 0);
  aacEncoder_SetParam(e.enc, AACENC_BITRATE, 192000);
  aacEncoder_SetParam(e.enc, AACENC_TRANSMUX, 0);
  aacEncoder_SetParam(e.enc, AACENC_AFTERBURNER, 1);
  if (aacEncEncode(e.enc, NULL, NULL, NULL, NULL) != AACENC_OK) return 0;
  if (aacEncInfo(e.enc, &e.info) != AACENC_OK) return 0;
  e.frameLen = e.info.frameLength;
  return 1;
}

static void close_encoder(OneShotEncoder &e) { aacEncClose(&e.enc); }

/*
 * Decode an already encoded bitstream with the requested max-output-channel
 * limit and channel mapping. The encoder ASC is reused so the same payload can
 * be fed to multiple decoders.
 */
static EncDecResult decode_with_limit(const OneShotEncoder &e,
                                      const UCHAR *payloads[],
                                      const int payloadSizes[], int numFrames,
                                      int maxOutChannels, int wavMapping) {
  EncDecResult res = {};
  HANDLE_AACDECODER dec = aacDecoder_Open(TT_MP4_RAW, 1);
  UCHAR *confArray[1] = {(UCHAR *)e.info.confBuf};
  UINT confSize = e.info.confSize;
  if (aacDecoder_ConfigRaw(dec, confArray, &confSize) != AAC_DEC_OK) {
    aacDecoder_Close(dec);
    return res;
  }
  aacDecoder_SetParam(dec, AAC_PCM_MAX_OUTPUT_CHANNELS, maxOutChannels);
  aacDecoder_SetParam(dec, AAC_PCM_OUTPUT_CHANNEL_MAPPING, wavMapping ? 1 : 0);
  aacDecoder_SetParam(dec, AAC_PCM_LIMITER_ENABLE, 0);

  INT_PCM *outBuf = (INT_PCM *)calloc(8 * e.frameLen, sizeof(INT_PCM));
  int decodedFrames = 0;
  int snapshotFrame = numFrames / 2;
  int limitViolated = 0;

  for (int f = 0; f < numFrames; f++) {
    UCHAR *bs = (UCHAR *)payloads[f];
    UINT bsSize = payloadSizes[f], bsValid = bsSize;
    if (aacDecoder_Fill(dec, &bs, &bsSize, &bsValid) != AAC_DEC_OK) continue;
    AAC_DECODER_ERROR dErr =
        aacDecoder_DecodeFrame(dec, outBuf, 8 * e.frameLen, 0);
    if (dErr != AAC_DEC_OK) continue;
    CStreamInfo *si = aacDecoder_GetStreamInfo(dec);
    if (maxOutChannels > 0 && si->numChannels > maxOutChannels)
      limitViolated = 1;
    if (f == snapshotFrame) {
      res.outChannels = si->numChannels;
      memcpy(res.type, si->pChannelType,
             si->numChannels * sizeof(AUDIO_CHANNEL_TYPE));
      memcpy(res.index, si->pChannelIndices,
             si->numChannels * sizeof(UCHAR));
      for (int ch = 0; ch < si->numChannels; ch++) {
        double acc = 0.0;
        for (int s = 0; s < si->frameSize; s++)
          acc += (double)labs(outBuf[s * si->numChannels + ch]);
        res.level[ch] = acc / si->frameSize;
      }
    }
    decodedFrames++;
  }
  res.ok = (decodedFrames > 0) && !limitViolated;
  free(outBuf);
  aacDecoder_Close(dec);
  return res;
}

/*
 * Encode `nChannels` channels of distinct sine waves (one unique frequency per
 * semantic channel), then decode with the requested max-output-channel limit
 * and channel mapping. Per-channel frequency tagging lets the test identify
 * which input speaker each output PCM slot actually contains, and compare that
 * against the reported labels.
 */
static EncDecResult pipeline_downmix(int nChannels, CHANNEL_MODE chMode,
                                     int maxOutChannels, int wavMapping) {
  EncDecResult res = {};
  const int frames = 12;
  OneShotEncoder e;
  if (!open_encoder(e, nChannels, chMode, wavMapping)) return res;

  UCHAR *payloads[32];
  int payloadSizes[32];
  int numFrames = 0;
  for (int f = 0; f < frames && numFrames < 32; f++) {
    UCHAR *bs = (UCHAR *)malloc(8192);
    int n = encode_sine_frame(e, f, bs, 8192);
    if (n > 0) {
      payloads[numFrames] = bs;
      payloadSizes[numFrames] = n;
      numFrames++;
    } else {
      free(bs);
    }
  }
  if (numFrames > 0)
    res = decode_with_limit(e, (const UCHAR **)payloads, payloadSizes,
                            numFrames, maxOutChannels, wavMapping);
  for (int f = 0; f < numFrames; f++) free(payloads[f]);
  close_encoder(e);
  return res;
}

static void test_pipeline_51_stereo_and_mono(void) {
  section("full pipeline: 5.1 encode -> decode capped to stereo and mono");

  /* WAV order 5.1 -> stereo */
  EncDecResult stereo =
      pipeline_downmix(6, MODE_1_2_2_1, 2 /* max */, 1 /* WAV */);
  CHECK(stereo.ok, "5.1->stereo pipeline failed");
  CHECK(stereo.outChannels == 2, "stereo output count %d", stereo.outChannels);
  if (stereo.outChannels == 2) {
    CHECK(stereo.type[0] == ACT_FRONT && stereo.index[0] == 0,
          "slot0 not labelled L (type %s idx %d)",
          ch_type_name(stereo.type[0]), stereo.index[0]);
    CHECK(stereo.type[1] == ACT_FRONT && stereo.index[1] == 1,
          "slot1 not labelled R (type %s idx %d)",
          ch_type_name(stereo.type[1]), stereo.index[1]);
    /* Both output channels must contain real signal. */
    CHECK(stereo.level[0] > 0.0 && stereo.level[1] > 0.0,
          "stereo output slot silent (%g/%g)", stereo.level[0],
          stereo.level[1]);
    /* No LFE label may survive a stereo restriction. */
    int lfe = 0;
    for (int i = 0; i < stereo.outChannels; i++)
      if (stereo.type[i] == ACT_LFE) lfe++;
    CHECK(lfe == 0, "LFE label survived stereo downmix");
  }

  /* WAV order 5.1 -> mono */
  EncDecResult mono = pipeline_downmix(6, MODE_1_2_2_1, 1 /* max */, 1);
  CHECK(mono.ok, "5.1->mono pipeline failed");
  CHECK(mono.outChannels == 1, "mono output count %d", mono.outChannels);
  if (mono.outChannels == 1) {
    CHECK(mono.type[0] == ACT_FRONT, "mono label %s",
          ch_type_name(mono.type[0]));
    CHECK(mono.level[0] > 0.0, "mono output silent");
  }
}

static void test_pipeline_wav_mpeg_label_order(void) {
  section("full pipeline: WAV vs MPEG label/PCM order for 5.1");
  /* Encode once, then decode the identical payload twice with different
   * output ordering, so any level difference can only come from reordering. */
  const int frames = 12;
  OneShotEncoder e;
  CHECK(open_encoder(e, 6, MODE_1_2_2_1, 1 /* WAV input */), "encoder open");

  UCHAR *payloads[32];
  int payloadSizes[32];
  int numFrames = 0;
  for (int f = 0; f < frames && numFrames < 32; f++) {
    UCHAR *bs = (UCHAR *)malloc(8192);
    int n = encode_sine_frame(e, f, bs, 8192);
    if (n > 0) {
      payloads[numFrames] = bs;
      payloadSizes[numFrames] = n;
      numFrames++;
    } else {
      free(bs);
    }
  }
  CHECK(numFrames > 0, "no frames encoded");

  EncDecResult wav = decode_with_limit(e, (const UCHAR **)payloads,
                                       payloadSizes, numFrames, 0 /* no cap */,
                                       1 /* WAV */);
  EncDecResult mpeg = decode_with_limit(e, (const UCHAR **)payloads,
                                        payloadSizes, numFrames, 0,
                                        0 /* MPEG */);
  for (int f = 0; f < numFrames; f++) free(payloads[f]);
  close_encoder(e);

  CHECK(wav.ok && mpeg.ok, "decode failed wav=%d mpeg=%d", wav.ok, mpeg.ok);
  CHECK(wav.outChannels == 6 && mpeg.outChannels == 6,
        "channel count changed (%d/%d)", wav.outChannels, mpeg.outChannels);
  if (!wav.ok || !mpeg.ok) return;

  /* WAV: L,R,C,LFE,Ls,Rs ; MPEG: C,L,R,Ls,Rs,LFE */
  CHECK(wav.type[0] == ACT_FRONT && wav.index[0] == 1, "WAV slot0 != L");
  CHECK(wav.type[1] == ACT_FRONT && wav.index[1] == 2, "WAV slot1 != R");
  CHECK(wav.type[2] == ACT_FRONT && wav.index[2] == 0, "WAV slot2 != C");
  CHECK(wav.type[3] == ACT_LFE, "WAV slot3 != LFE");
  CHECK(mpeg.type[0] == ACT_FRONT && mpeg.index[0] == 0, "MPEG slot0 != C");
  CHECK(mpeg.type[1] == ACT_FRONT && mpeg.index[1] == 1, "MPEG slot1 != L");
  CHECK(mpeg.type[2] == ACT_FRONT && mpeg.index[2] == 2, "MPEG slot2 != R");
  CHECK(mpeg.type[5] == ACT_LFE, "MPEG slot5 != LFE");

  /* Same payload -> same semantic speaker must carry the same energy no matter
   * which slot it lands in. Labels move with the PCM; no signal is remixed by
   * merely switching the format marker. */
  for (int t = 0; t < 6; t++) {
    AUDIO_CHANNEL_TYPE type = wav.type[t];
    UCHAR idx = wav.index[t];
    int mpegSlot = -1;
    for (int j = 0; j < 6; j++)
      if (mpeg.type[j] == type && mpeg.index[j] == idx) mpegSlot = j;
    CHECK(mpegSlot >= 0, "WAV label (%s,%d) absent in MPEG output",
          ch_type_name(type), idx);
    if (mpegSlot >= 0 && wav.level[t] > 0.0) {
      double rel = fabs(wav.level[t] - mpeg.level[mpegSlot]) / wav.level[t];
      CHECK(rel < 0.02,
            "energy for (%s,%d) diverges between WAV/MPEG slots (%g vs %g)",
            ch_type_name(type), idx, wav.level[t], mpeg.level[mpegSlot]);
    }
  }
}

/* --------------------------------- main ---------------------------------- */

int main(void) {
  printf("FDK-AAC multichannel downmix unit tests");

  /* Section 1: labels and PCM order must always agree. */
  test_51_stereo_label_pcm_consistency(0 /* WAV */,
                                       "5.1(WAV) -> stereo label/PCM order");
  test_51_stereo_label_pcm_consistency(1 /* MPEG */,
                                       "5.1(MPEG) -> stereo label/PCM order");
  test_51_mono_label_pcm_consistency();
  test_channel_map_descriptor_reorder();

  /* Section 2: downmix weights. */
  test_51_stereo_default_weights();
  test_downmix_linearity();

  /* Section 3: output channel count limits and expansion. */
  test_output_channel_limits();
  test_channel_expansion();

  /* Section 4: center/LFE and saturation/scaling. */
  test_center_and_lfe_handling();
  test_saturation_is_clamped();
  test_small_signal_scaling();

  /* Section 5: encode/decode pipeline. */
  test_pipeline_51_stereo_and_mono();
  test_pipeline_wav_mpeg_label_order();

  printf("\n--------------------------------------------------\n");
  printf("checks: %d, failures: %d\n", g_checks, g_failures);
  if (g_failures == 0)
    printf("ALL MULTICHANNEL DOWNMIX TESTS PASSED\n");
  else
    printf("SOME TESTS FAILED\n");
  return g_failures ? 1 : 0;
}
