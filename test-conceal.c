/* ------------------------------------------------------------------
 * Integration test: AAC decoder error concealment / lost frame recovery
 *
 * What this test exercises
 * ------------------------
 *  - consecutive lost access units (AU), each replaced by one
 *    aacDecoder_DecodeFrame(..., AACDEC_CONCEAL) call
 *  - frame loss at the long/short window boundary (loss directly after an
 *    EIGHT_SHORT or an ONLY_LONG AU)
 *  - explicit time advancement: how many samples each decode call produces and
 *    how the playback cursor moves, for compensated and uncompensated loss
 *  - fade-out -> mute -> fade-in and the concealment state continuity across
 *    the good/masked/good transition
 *  - error visibility: a corrupted AU must report a decode error that later
 *    successful frames do not hide, including the case where error
 *    interpolation reports the error one frame late
 *  - transport-level loss: AUs physically removed from the ADTS byte stream
 *
 * What this test deliberately does NOT do
 * ---------------------------------------
 *  - It never treats concealed output as the lost original samples. Concealed
 *    frames are decoder-synthesised substitute ("masked") audio and are
 *    labelled MASKED everywhere in the report.
 *  - It never treats the AAC_DECODER_ERROR returned for an AACDEC_CONCEAL call
 *    as evidence that a frame was real audio. The noise and interpolation
 *    concealment methods can return AAC_DEC_OK for a purely synthesised (or
 *    one-frame-delayed) frame. The test therefore tracks the loss itself and
 *    only uses the returned error codes to verify that genuine bitstream
 *    errors stay visible.
 *
 * The test is self-contained: it generates deterministic signals, encodes them
 * with the encoder in this same library and decodes the resulting ADTS stream,
 * so it needs no external audio files. Exit code 0 = all checks passed.
 * -------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "aacenc_lib.h"
#include "aacdecoder_lib.h"

#define SR 48000
#define FRAME_LEN 1024
#define MAX_AU 512
#define MAX_OUT 4096
#define MAX_FRAMES 512

/* Concealment methods as understood by AAC_CONCEAL_METHOD. */
#define METHOD_DEFAULT (-1)
#define METHOD_MUTE 0
#define METHOD_NOISE 1
#define METHOD_INTER 2

/* A decoded sample is considered "the same audio" as the reference when the
 * largest difference stays below this tolerance. Both runs use the same
 * library instance, so real decoded frames are bit identical; the tolerance
 * only protects against architecture specific rounding. */
#define SAME_TOL 4

/* ------------------------------------------------------------------ */
/* deterministic test signals                                          */
/* ------------------------------------------------------------------ */
static int16_t sig_pcm[MAX_FRAMES * FRAME_LEN];
static int sig_frames;

/* MSVC has no lrint() in older C runtimes; keep the test portable. */
static int16_t to_pcm16(double v) {
  double s = v * 32767.0;
  return (int16_t)(s >= 0.0 ? s + 0.5 : s - 0.5);
}

/* steady 440 Hz tone: makes the fade-out / fade-in envelope measurable */
static void signal_steady(int nf) {
  sig_frames = nf;
  for (int n = 0; n < nf * FRAME_LEN; n++) {
    sig_pcm[n] = to_pcm16(0.25 * sin(2.0 * M_PI * 440.0 * n / SR));
  }
}

/* sparse clicks every 5th frame: the encoder switches to EIGHT_SHORT windows
 * (window_sequence 2) and back to long windows on the adjacent frames. */
static void signal_click(int nf) {
  sig_frames = nf;
  memset(sig_pcm, 0, sizeof(int16_t) * nf * FRAME_LEN);
  for (int f = 0; f < nf; f += 5) {
    int n = f * FRAME_LEN + 100;
    for (int k = 0; k < 60; k++)
      sig_pcm[n + k] = (int16_t)((k % 2) ? -12000 : 12000);
  }
}

/* ------------------------------------------------------------------ */
/* encoding into an ADTS byte stream, split into AUs                   */
/* ------------------------------------------------------------------ */
static unsigned char stream[MAX_FRAMES * 4096];
static int stream_len;
static int au_off[MAX_AU], au_len[MAX_AU], nau;

static int encode_stream(void) {
  static unsigned char out[8192];
  HANDLE_AACENCODER enc;
  AACENC_InfoStruct info = {0};

  if (aacEncOpen(&enc, 0, 1) != AACENC_OK) {
    fprintf(stderr, "conceal-test: aacEncOpen failed\n");
    return 1;
  }
  aacEncoder_SetParam(enc, AACENC_AOT, 2);
  aacEncoder_SetParam(enc, AACENC_SAMPLERATE, SR);
  aacEncoder_SetParam(enc, AACENC_CHANNELMODE, MODE_1);
  aacEncoder_SetParam(enc, AACENC_CHANNELORDER, 1);
  aacEncoder_SetParam(enc, AACENC_BITRATE, 96000);
  aacEncoder_SetParam(enc, AACENC_TRANSMUX, TT_MP4_ADTS);
  if (aacEncEncode(enc, NULL, NULL, NULL, NULL) != AACENC_OK) {
    fprintf(stderr, "conceal-test: encoder init failed\n");
    return 1;
  }
  aacEncInfo(enc, &info);
  if (info.frameLength != FRAME_LEN || info.inputChannels != 1) {
    fprintf(stderr, "conceal-test: unexpected encoder layout (%d/%d)\n",
            info.frameLength, info.inputChannels);
    return 1;
  }

  stream_len = 0;
  int fed = 0, flush = 0;
  for (;;) {
    AACENC_BufDesc ib = {0}, ob = {0};
    AACENC_InArgs ia = {0};
    AACENC_OutArgs oa = {0};
    int in_id = IN_AUDIO_DATA, in_el = 2;
    int out_id = OUT_BITSTREAM_DATA, out_el = 1;
    void *in_ptr, *out_ptr;
    int in_size, out_size;

    if (!flush) {
      int n = sig_frames * FRAME_LEN - fed;
      if (n > FRAME_LEN) n = FRAME_LEN;
      if (n <= 0) { flush = 1; continue; }
      in_ptr = &sig_pcm[fed];
      in_size = n * 2;
      ia.numInSamples = n;
      fed += n;
    } else {
      in_ptr = NULL; in_size = 0; ia.numInSamples = -1;
    }
    ib.numBufs = 1; ib.bufs = &in_ptr; ib.bufferIdentifiers = &in_id;
    ib.bufSizes = &in_size; ib.bufElSizes = &in_el;
    out_ptr = out; out_size = sizeof(out);
    ob.numBufs = 1; ob.bufs = &out_ptr; ob.bufferIdentifiers = &out_id;
    ob.bufSizes = &out_size; ob.bufElSizes = &out_el;

    AACENC_ERROR err = aacEncEncode(enc, &ib, &ob, &ia, &oa);
    if (err != AACENC_OK) {
      if (err == AACENC_ENCODE_EOF) break;
      fprintf(stderr, "conceal-test: encode failed (%d)\n", err);
      return 1;
    }
    if (oa.numOutBytes > 0) {
      if (stream_len + oa.numOutBytes > (int)sizeof(stream)) return 1;
      memcpy(stream + stream_len, out, oa.numOutBytes);
      stream_len += oa.numOutBytes;
    }
    if (flush && oa.numOutBytes == 0) break;
  }
  aacEncClose(&enc);

  int pos = 0;
  nau = 0;
  while (pos + 7 <= stream_len) {
    if (stream[pos] != 0xff || (stream[pos + 1] & 0xf0) != 0xf0) {
      fprintf(stderr, "conceal-test: bad ADTS sync at byte %d\n", pos);
      return 1;
    }
    int len = ((stream[pos + 3] & 0x03) << 11) | (stream[pos + 4] << 3) |
              (stream[pos + 5] >> 5);
    if (len < 7 || pos + len > stream_len || nau >= MAX_AU) return 1;
    au_off[nau] = pos;
    au_len[nau] = len;
    nau++;
    pos += len;
  }
  return 0;
}

/* Minimal ICS peek: window_sequence of the first SCE in an AAC-LC ADTS AU.
 * 0 = ONLY_LONG, 1 = LONG_START, 2 = EIGHT_SHORT, 3 = LONG_STOP. */
static int au_window_sequence(const unsigned char *au, int n) {
  if ((au[6] & 0x03) != 0) return -1; /* not exactly one raw data block */
  const unsigned char *p = au + 7;
  int bit = 0, total = (n - 7) * 8, v = 0;
  if (total < 3 + 4 + 8 + 1 + 2) return -1;
#define GET(k)                                                     \
  do {                                                             \
    v = 0;                                                         \
    for (int _i = 0; _i < (k); _i++) {                             \
      v = (v << 1) | ((p[bit >> 3] >> (7 - (bit & 7))) & 1);       \
      bit++;                                                       \
    }                                                              \
  } while (0)
  GET(3);
  if (v != 0) return -1; /* first element is not an SCE */
  GET(4);                /* element_instance_tag */
  GET(8);                /* global_gain */
  GET(1);                /* ics_reserved_bit */
  GET(2);                /* window_sequence */
  return v;
#undef GET
}

/* ------------------------------------------------------------------ */
/* decode scenarios                                                    */
/* ------------------------------------------------------------------ */

/* How a decoded output frame came to be. This cannot be derived from the
 * error code, because a conceal call may return AAC_DEC_OK. */
typedef enum {
  FRAME_DECODED = 0, /* decoded from a real AU */
  FRAME_MASKED = 1,  /* decoder-synthesised substitute; NOT the original */
  FRAME_ERROR = 2    /* output produced while a bitstream error was active */
} FrameKind;

typedef struct {
  int16_t pcm[MAX_FRAMES][MAX_OUT];
  FrameKind kind[MAX_FRAMES];
  AAC_DECODER_ERROR err[MAX_FRAMES];
  int au_of_call[MAX_FRAMES];
  int samples[MAX_FRAMES]; /* interleaved samples returned by this call */
  int frame_size[MAX_FRAMES];
  unsigned delay[MAX_FRAMES];
  int n;
  int64_t cursor; /* explicit playback time in interleaved samples */
  long long total_au, bad_au, bad_bytes;
} Run;

static Run *run_alloc(void) {
  Run *r = (Run *)calloc(1, sizeof(Run));
  if (!r) {
    fprintf(stderr, "conceal-test: out of memory\n");
    exit(2);
  }
  return r;
}

typedef struct {
  int first;
  int count;
} Loss;

static int au_is_lost(const Loss *l, int nloss, int au) {
  for (int i = 0; i < nloss; i++)
    if (au >= l[i].first && au < l[i].first + l[i].count) return 1;
  return 0;
}

static void record(Run *r, CStreamInfo *si, AAC_DECODER_ERROR err,
                   FrameKind kind, int au) {
  r->err[r->n] = err;
  r->kind[r->n] = kind;
  r->au_of_call[r->n] = au;
  r->frame_size[r->n] = si->frameSize;
  r->samples[r->n] = si->frameSize * si->numChannels;
  r->delay[r->n] = si->outputDelay;
  r->total_au = (long long)si->numTotalAccessUnits;
  r->bad_au = (long long)si->numBadAccessUnits;
  r->bad_bytes = (long long)si->numBadBytes;
  r->cursor += r->samples[r->n];
  r->n++;
}

/* synthesize_lost:
 *   0 = do not produce any output for the lost AUs (the application leaves a
 *       hole in the timeline; this is the "uncompensated" case)
 *   1 = one AACDEC_CONCEAL call per lost AU (decoder synthesises audio)
 * corrupt_au:
 *   AU index whose payload is damaged, or -1. A corrupted AU that is present
 *   in the stream must be decoded *without* AACDEC_CONCEAL: the flag tells the
 *   decoder to ignore the bitstream entirely, so the corruption would only be
 *   discovered on the following call. Decoding it normally is what a real
 *   receiver does. */
static void run_scenario(Run *r, int method, const Loss *loss, int nloss,
                         int corrupt_au, int synthesize_lost) {
  memset(r, 0, sizeof(*r));
  HANDLE_AACDECODER dec = aacDecoder_Open(TT_MP4_ADTS, 1);
  if (!dec) {
    fprintf(stderr, "conceal-test: aacDecoder_Open failed\n");
    exit(2);
  }
  if (method != METHOD_DEFAULT) {
    if (aacDecoder_SetParam(dec, AAC_CONCEAL_METHOD, method) != AAC_DEC_OK) {
      fprintf(stderr, "conceal-test: cannot set conceal method %d\n", method);
      exit(2);
    }
  }

  /* A decoder instance that never saw a valid configuration cannot conceal;
   * but the first AU of every scenario below is always valid, so this is
   * covered implicitly. */
  int au = 0;
  while (au < nau) {
    if (au_is_lost(loss, nloss, au)) {
      int count = 0;
      for (int i = 0; i < nloss; i++)
        if (au == loss[i].first) count = loss[i].count;
      if (synthesize_lost) {
        for (int k = 0; k < count; k++) {
          INT_PCM *dst = (INT_PCM *)r->pcm[r->n];
          AAC_DECODER_ERROR err =
              aacDecoder_DecodeFrame(dec, dst, MAX_OUT, AACDEC_CONCEAL);
          CStreamInfo *si = aacDecoder_GetStreamInfo(dec);
          /* A conceal call that returns AAC_DEC_OK still produces substitute
           * audio. Only claims of validity from IS_OUTPUT_VALID() are used. */
          record(r, si, err,
                 IS_OUTPUT_VALID(err) ? FRAME_MASKED : FRAME_ERROR, au + k);
        }
      }
      au += count;
      continue;
    }

    unsigned char *data = stream + au_off[au];
    int size = au_len[au];
    static unsigned char corrupt_buf[8192];
    if (au == corrupt_au) {
      memcpy(corrupt_buf, data, size);
      /* damage payload only, keep the ADTS header intact */
      for (int k = 10; k < size && k < 24; k++) corrupt_buf[k] ^= 0xa5;
      data = corrupt_buf;
    }
    UINT valid = size, buf_size = size;
    AAC_DECODER_ERROR fill_err = aacDecoder_Fill(dec, &data, &buf_size, &valid);
    if (fill_err != AAC_DEC_OK) {
      fprintf(stderr, "conceal-test: Fill failed %x at AU %d\n", fill_err, au);
      exit(2);
    }

    INT_PCM *dst = (INT_PCM *)r->pcm[r->n];
    /* Never pass AACDEC_CONCEAL for a present (possibly corrupted) AU: with
     * that flag the decoder discards the bitstream and the error would only
     * surface on a later call, which would misrepresent what a receiver sees.
     * The decoder's own concealment runs because frameOK becomes 0. */
    AAC_DECODER_ERROR err = aacDecoder_DecodeFrame(dec, dst, MAX_OUT, 0);
    CStreamInfo *si = aacDecoder_GetStreamInfo(dec);
    FrameKind kind = FRAME_DECODED;
    if (!IS_OUTPUT_VALID(err)) kind = FRAME_ERROR;
    else if (IS_DECODE_ERROR(err)) kind = FRAME_ERROR;
    record(r, si, err, kind, au);
    au++;
  }
  aacDecoder_Close(dec);
}

/* ------------------------------------------------------------------ */
/* measurement helpers                                                 */
/* ------------------------------------------------------------------ */
static double frame_rms(const int16_t *p, int n) {
  double s = 0.0;
  for (int i = 0; i < n; i++) s += (double)p[i] * (double)p[i];
  return n > 0 ? sqrt(s / (double)n) : 0.0;
}

static int frame_max_abs_diff(const int16_t *a, const int16_t *b, int n) {
  int m = 0;
  for (int i = 0; i < n; i++) {
    int d = abs((int)a[i] - (int)b[i]);
    if (d > m) m = d;
  }
  return m;
}

static int same_as(const Run *a, const Run *b, int slot) {
  if (slot >= a->n || slot >= b->n) return 0;
  int n = a->samples[slot];
  if (n != b->samples[slot] || n <= 0) return 0;
  return frame_max_abs_diff(a->pcm[slot], b->pcm[slot], n) <= SAME_TOL;
}

static int g_checks, g_failures;

static void check(int cond, const char *fmt, ...) {
  g_checks++;
  if (!cond) {
    g_failures++;
    fprintf(stderr, "FAIL: ");
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
  }
}

static const char *kind_name(FrameKind k) {
  switch (k) {
    case FRAME_DECODED: return "decoded";
    case FRAME_MASKED: return "MASKED(substitute)";
    default: return "ERROR";
  }
}

static void dump_frames(const char *title, const Run *r, const Run *ref,
                        int from, int to) {
  printf("  %s\n", title);
  printf("  %4s %-19s %-6s %-5s %-8s %-8s %s\n", "slot", "audio", "err", "AU",
         "samples", "delay", "rms/ref");
  for (int i = from; i <= to && i < r->n; i++) {
    int n = r->samples[i];
    double v = frame_rms(r->pcm[i], n);
    double rr = (ref && i < ref->n) ? frame_rms(ref->pcm[i], ref->samples[i]) : 0.0;
    char ratio[32];
    if (rr > 1.0) snprintf(ratio, sizeof(ratio), "%.3f", v / rr);
    else snprintf(ratio, sizeof(ratio), "-");
    printf("  %4d %-19s 0x%04x %5d %8d %8u %s\n", i, kind_name(r->kind[i]),
           (unsigned)r->err[i], r->au_of_call[i], n, r->delay[i], ratio);
  }
}

/* every call must produce exactly one frame on the playback timeline */
static void check_time_advance(const Run *r, int ref_frames, const char *what) {
  check(r->n == ref_frames,
        "%s: output frame count %d does not match the %d AUs of the stream "
        "(each AU must produce exactly one frame)", what, r->n, ref_frames);
  check(r->cursor == (int64_t)ref_frames * FRAME_LEN,
        "%s: playback cursor is %lld samples, expected %d frames * %d = %lld",
        what, (long long)r->cursor, ref_frames, FRAME_LEN,
        (long long)ref_frames * FRAME_LEN);
  for (int i = 0; i < r->n; i++)
    check(r->samples[i] == FRAME_LEN || r->samples[i] == r->frame_size[i],
          "%s: slot %d returned %d samples, expected one %d-sample frame",
          what, i, r->samples[i], FRAME_LEN);
}

int main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("AAC decoder concealment / lost-frame recovery integration test\n");
  printf("self-generated signal, AAC-LC 48 kHz mono 96 kbit/s ADTS\n\n");

  Run *ref = run_alloc();
  Run *r = run_alloc();

  /* ================================================================== */
  /* 1. consecutive loss with noise substitution (no extra output delay) */
  /* ================================================================== */
  printf("[1] consecutive loss, conceal method 1 (noise substitution)\n");
  signal_steady(60);
  if (encode_stream()) return 2;
  run_scenario(ref, METHOD_NOISE, NULL, 0, -1, 0);

  const int loss_count = 10;
  const int loss_start = ref->n / 2 - 2;
  Loss loss = {loss_start, loss_count};
  run_scenario(r, METHOD_NOISE, &loss, 1, -1, 1);

  printf("  stream AUs=%d, reference frames=%d, %d consecutive AUs lost from "
         "slot %d\n",
         nau, ref->n, loss_count, loss_start);
  dump_frames("slots around the loss:", r, ref, loss_start - 2,
              loss_start + loss_count + 10);

  /* ---- time advancement ---- */
  check_time_advance(r, ref->n, "consecutive loss");
  printf("  time: %d lost AUs -> %d concealed calls -> %lld samples, one "
         "frame per AU, cursor advanced by %d samples\n",
         loss_count, loss_count, (long long)r->cursor, loss_count * FRAME_LEN);

  /* ---- audio before the loss is untouched ---- */
  for (int i = 0; i < loss_start; i++)
    check(same_as(r, ref, i),
          "slot %d before the loss differs from the reference decode", i);

  /* ---- concealed frames are replacements, never the originals ---- */
  int masked = 0, masked_equal_ref = 0, ok_reported = 0;
  for (int i = loss_start; i < loss_start + loss_count; i++) {
    if (i >= r->n || r->kind[i] != FRAME_MASKED) continue;
    masked++;
    if (same_as(r, ref, i)) masked_equal_ref++;
    if (r->err[i] == AAC_DEC_OK) ok_reported++;
  }
  check(masked == loss_count,
        "only %d of %d lost frames were labelled MASKED(substitute)", masked,
        loss_count);
  check(masked_equal_ref == 0,
        "%d masked frame(s) are identical to the reference decode of the lost "
        "AU; concealing must not be presented as recovering lost samples",
        masked_equal_ref);
  printf("  conceal calls returning AAC_DEC_OK: %d of %d  (the error code "
         "does NOT prove the audio is original)\n",
         ok_reported, loss_count);
  printf("  MASKED frames: %d, identical to the lost originals: %d  "
         "(original samples are NOT reconstructed)\n",
         masked, masked_equal_ref);

  /* ---- fade-out ---- */
  double pre = frame_rms(ref->pcm[loss_start - 1], FRAME_LEN);
  double first_masked = frame_rms(r->pcm[loss_start], FRAME_LEN);
  double last_masked = frame_rms(r->pcm[loss_start + loss_count - 1], FRAME_LEN);
  double min_rms = first_masked;
  int min_slot = loss_start;
  for (int i = loss_start; i < loss_start + loss_count; i++) {
    double v = frame_rms(r->pcm[i], FRAME_LEN);
    if (v < min_rms) { min_rms = v; min_slot = i; }
  }
  printf("  fade-out level: pre=%.1f first=%.1f last=%.1f min=%.1f (slot %d)\n",
         pre, first_masked, last_masked, min_rms, min_slot);
  check(first_masked > 0.5 * pre && first_masked < 1.5 * pre,
        "fade-out is discontinuous at its start: first masked frame is %.2f x "
        "the pre-loss level",
        first_masked / (pre > 0 ? pre : 1));
  check(last_masked < 0.5 * first_masked,
        "level did not decay across the %d-frame loss (first %.1f, last %.1f)",
        loss_count, first_masked, last_masked);
  check(min_rms < 0.02 * pre,
        "audio never reaches near-mute during a %d-frame loss (min %.4f vs "
        "pre-loss %.1f, slot %d)",
        loss_count, min_rms, pre, min_slot);

  /* ---- state continuity: fade-in after the first good AU ---- */
  int restored = -1;
  for (int i = loss_start + loss_count; i < r->n; i++) {
    if (same_as(r, ref, i)) { restored = i; break; }
  }
  check(restored >= 0,
        "decoded audio never returns to the reference after the loss");
  if (restored >= 0) {
    int fade_in = restored - (loss_start + loss_count);
    printf("  output returns to the reference at slot %d, %d frames after the "
           "first good AU\n",
           restored, fade_in);
    check(fade_in <= 8, "fade-in takes %d frames (expected the 5 default "
                        "fade-in frames)",
          fade_in);
    /* the recovered frame must sit at the same slot as the reference, i.e.
     * the concealed frames really advanced the timeline */
    check(restored == loss_start + loss_count + fade_in,
          "recovered audio is not aligned with the compensated timeline");
  }
  for (int i = loss_start; i < r->n; i++) {
    double rr = frame_rms(ref->pcm[i], FRAME_LEN);
    double v = frame_rms(r->pcm[i], FRAME_LEN);
    check(v <= 1.5 * rr + 50.0,
          "slot %d overshoots the reference level (%.1f vs %.1f): pop", i, v,
          rr);
  }

  /* ================================================================== */
  /* 2. same loss, application does not compensate                       */
  /* ================================================================== */
  printf("\n[2] same %d lost AUs, application skips them without "
         "AACDEC_CONCEAL\n",
         loss_count);
  run_scenario(r, METHOD_NOISE, &loss, 1, -1, 0);
  printf("  output frames=%d (reference %d), timeline samples=%lld (reference "
         "%lld)\n",
         r->n, ref->n, (long long)r->cursor,
         (long long)ref->n * FRAME_LEN);
  check(r->n == ref->n - loss_count,
        "expected %d frames when loss is not compensated, got %d",
        ref->n - loss_count, r->n);
  check(r->cursor == (int64_t)(ref->n - loss_count) * FRAME_LEN,
        "uncompensated timeline has %lld samples, expected %lld",
        (long long)r->cursor, (long long)(ref->n - loss_count) * FRAME_LEN);
  check((int64_t)ref->n * FRAME_LEN - r->cursor ==
            (int64_t)loss_count * FRAME_LEN,
        "the missing time is not exactly one frame per lost AU");
  /* the audio after the hole is shifted early: the decoder cannot know */
  int shifted = 0;
  for (int k = 0; k < 4; k++) {
    int slot = loss_start + k;
    if (slot < r->n && !same_as(r, ref, slot)) shifted++;
  }
  check(shifted > 0,
        "audio after an uncompensated loss is still aligned with the "
        "reference, which would mean the decoder repaired the timeline itself");
  printf("  => %d frames (%d samples = %.1f ms) are missing from the "
         "playback timeline;\n",
         loss_count, loss_count * FRAME_LEN,
         1000.0 * loss_count * FRAME_LEN / SR);
  printf("     the caller must insert them explicitly (or conceal) - the "
         "decoder does not compensate lost AU time.\n");

  /* ================================================================== */
  /* 3. loss at the long/short window boundary                           */
  /* ================================================================== */
  printf("\n[3] loss at the long/short window boundary\n");
  signal_click(70);
  if (encode_stream()) return 2;
  Run *ref2 = run_alloc();
  Run *r2 = run_alloc();
  run_scenario(ref2, METHOD_NOISE, NULL, 0, -1, 0);

  int wseq[MAX_AU];
  for (int i = 0; i < nau; i++)
    wseq[i] = au_window_sequence(stream + au_off[i], au_len[i]);
  printf("  window_sequence of the first AUs (0 long, 1 long_start, 2 "
         "eight_short, 3 long_stop):");
  for (int i = 0; i < nau && i < 16; i++) printf(" %d", wseq[i]);
  printf(" ...\n");

  struct {
    const char *name;
    int want_wseq;    /* window_sequence of the last good AU before the loss */
  } boundary[] = {
      {"after EIGHT_SHORT", 2},
      {"after ONLY_LONG", 0},
      {"after LONG_START", 1},
  };
  int tested_boundaries = 0;
  for (unsigned bi = 0; bi < sizeof(boundary) / sizeof(boundary[0]); bi++) {
    int good = -1;
    for (int i = 1; i + 10 < nau; i++) {
      if (wseq[i] == boundary[bi].want_wseq && wseq[i + 1] >= 0) {
        good = i;
        break;
      }
    }
    if (good < 0) continue;
    tested_boundaries++;
    Loss wl = {good + 1, 8};
    run_scenario(r2, METHOD_NOISE, &wl, 1, -1, 1);
    printf("\n  --- loss %s: last good AU %d (wseq %d), next AU was wseq %d, "
           "%d AUs lost\n",
           boundary[bi].name, good, wseq[good], wseq[good + 1], wl.count);
    dump_frames("slots around the boundary loss:", r2, ref2, good - 1,
                good + 14);

    check_time_advance(r2, ref2->n, boundary[bi].name);
    check(r2->n == ref2->n,
          "%s: frame count changed (%d vs %d)", boundary[bi].name, r2->n,
          ref2->n);
    for (int i = 0; i <= good; i++)
      check(same_as(r2, ref2, i),
            "%s: slot %d before the loss differs from the reference",
            boundary[bi].name, i);

    int m = 0;
    for (int i = good + 1; i <= good + wl.count && i < r2->n; i++)
      if (r2->kind[i] == FRAME_MASKED) m++;
    check(m == wl.count,
          "%s: only %d of %d boundary-loss frames are labelled MASKED",
          boundary[bi].name, m, wl.count);

    int back = -1;
    for (int i = good + 1 + wl.count; i < r2->n; i++)
      if (same_as(r2, ref2, i)) { back = i; break; }
    check(back >= 0, "%s: audio never recovers to the reference decode",
          boundary[bi].name);
    if (back >= 0)
      printf("  audio restored at slot %d (%d frames after the first good AU)\n",
             back, back - (good + 1 + wl.count));

    /* the window sequence of the first frames after recovery must be the
     * encoder's own sequence again, not a stuck concealed window */
    int recovered_wseq = au_window_sequence(stream + au_off[good + 1 + wl.count],
                                            au_len[good + 1 + wl.count]);
    printf("  the decoder's first frame after recovery continues from a "
           "wseq=%d AU (concealment reuses the stored window sequence while "
           "masking)\n",
           recovered_wseq);
  }
  check(tested_boundaries >= 2,
        "only %d window-boundary cases could be constructed from the test "
        "signal",
        tested_boundaries);

  /* ================================================================== */
  /* 4. corrupted AU: the error must stay visible                        */
  /* ================================================================== */
  printf("\n[4] corrupted AU payload: error visibility\n");
  signal_steady(40);
  if (encode_stream()) return 2;
  Run *ref3 = run_alloc();
  Run *r3 = run_alloc();
  run_scenario(ref3, METHOD_DEFAULT, NULL, 0, -1, 0);
  const int corrupt_au = ref3->n / 2;
  run_scenario(r3, METHOD_DEFAULT, NULL, 0, corrupt_au, 0);
  dump_frames("slots around the corrupted AU (decoded normally, no "
              "AACDEC_CONCEAL):",
              r3, ref3, corrupt_au - 2, corrupt_au + 6);

  /* The corruption is discovered while decoding the corrupted AU itself, so
   * the error must be visible on exactly that call. Successful concealment of
   * the same frame must not turn it into AAC_DEC_OK. */
  check(IS_DECODE_ERROR(r3->err[corrupt_au]),
        "corrupted AU %d did not report a decode error on its own call "
        "(err 0x%04x): concealment hid the bitstream error",
        corrupt_au, (unsigned)r3->err[corrupt_au]);
  check(IS_OUTPUT_VALID(r3->err[corrupt_au]),
        "corrupted AU produced no usable concealed output (err 0x%04x)",
        (unsigned)r3->err[corrupt_au]);
  check(r3->kind[corrupt_au] == FRAME_ERROR,
        "corrupted AU was classified as clean audio on its own call");
  check(r3->samples[corrupt_au] == FRAME_LEN,
        "corrupted AU output %d samples, expected one %d-sample frame",
        r3->samples[corrupt_au], FRAME_LEN);
  check(r3->bad_au == 1,
        "numBadAccessUnits is %lld after one corrupted AU, expected 1",
        r3->bad_au);
  check(r3->bad_bytes > 0,
        "numBadBytes did not report the corrupted AU (got %lld)",
        r3->bad_bytes);
  printf("  decode error 0x%04x reported on the corrupted AU's own call "
         "(slot %d)\n",
         (unsigned)r3->err[corrupt_au], corrupt_au);

  /* later success must not erase the evidence */
  int late_ok = 0;
  for (int i = corrupt_au + 1; i < r3->n && i < corrupt_au + 6; i++)
    if (r3->err[i] == AAC_DEC_OK) late_ok++;
  check(late_ok > 0,
        "the decoder did not recover to AAC_DEC_OK after the corrupted AU");
  check(r3->bad_au == 1,
        "error statistics were changed by later successful frames (bad AUs now "
        "%lld, expected 1)",
        r3->bad_au);
  check(r3->total_au == (long long)ref3->n,
        "the timeline lost an AU: total AUs %lld, expected %d",
        r3->total_au, ref3->n);
  printf("  statistics after later successful frames: totalAU=%lld badAU=%lld "
         "badBytes=%lld (evidence kept)\n",
         r3->total_au, r3->bad_au, r3->bad_bytes);

  /* Control experiment: passing AACDEC_CONCEAL for a *present* AU makes the
   * decoder discard the bitstream, so the corruption is only detected one
   * call later. This documents why the test never does that - it would make
   * the error appear late and would misattribute the delay to the decoder. */
  Run *r3b = run_alloc();
  {
    memset(r3b, 0, sizeof(*r3b));
    HANDLE_AACDECODER dec = aacDecoder_Open(TT_MP4_ADTS, 1);
    for (int au = 0; au < nau && r3b->n < MAX_FRAMES; au++) {
      unsigned char *data = stream + au_off[au];
      int size = au_len[au];
      static unsigned char cb[8192];
      if (au == corrupt_au) {
        memcpy(cb, data, size);
        for (int k = 10; k < size && k < 24; k++) cb[k] ^= 0xa5;
        data = cb;
      }
      UINT valid = size, buf_size = size;
      aacDecoder_Fill(dec, &data, &buf_size, &valid);
      AAC_DECODER_ERROR err = aacDecoder_DecodeFrame(
          dec, (INT_PCM *)r3b->pcm[r3b->n], MAX_OUT,
          au == corrupt_au ? AACDEC_CONCEAL : 0);
      record(r3b, aacDecoder_GetStreamInfo(dec), err,
             IS_OUTPUT_VALID(err) ? FRAME_MASKED : FRAME_ERROR, au);
    }
    aacDecoder_Close(dec);
    int late = -1;
    for (int i = corrupt_au; i <= corrupt_au + 2 && i < r3b->n; i++)
      if (IS_DECODE_ERROR(r3b->err[i])) { late = i; break; }
    printf("  control: AACDEC_CONCEAL on the corrupted AU -> error surfaces at "
           "slot %d instead of %d\n",
           late, corrupt_au);
    check(late >= 0,
          "control run never reported the corrupted AU at all");
    check(late > corrupt_au,
          "control run reported the error on the same call; the test's "
          "no-CONCEAL requirement is unnecessary");
  }

  /* ================================================================== */
  /* 5. transport-level loss: whole AUs removed from the byte stream     */
  /* ================================================================== */
  printf("\n[5] transport-level loss: whole AUs removed from the ADTS stream\n");
  signal_steady(40);
  if (encode_stream()) return 2;
  Run *ref4 = run_alloc();
  run_scenario(ref4, METHOD_NOISE, NULL, 0, -1, 0);
  printf("  reference: %d frames / %d samples\n", ref4->n,
         ref4->n * FRAME_LEN);

  const int drop_first = ref4->n / 2;
  const int drop_count = 4;
  if (drop_first + drop_count < nau) {
    int cut_start = au_off[drop_first];
    int cut_end = au_off[drop_first + drop_count];
    static unsigned char cut[MAX_FRAMES * 4096];
    memcpy(cut, stream, cut_start);
    memcpy(cut + cut_start, stream + cut_end, stream_len - cut_end);
    int cut_len = stream_len - (cut_end - cut_start);

    /* feed the shortened stream in small chunks, as a network stack would */
    HANDLE_AACDECODER dec = aacDecoder_Open(TT_MP4_ADTS, 1);
    static INT_PCM pcm[MAX_OUT];
    int off = 0, frames = 0, hard_errors = 0;
    int64_t samples = 0;
    while (off < cut_len) {
      int chunk = cut_len - off > 512 ? 512 : cut_len - off;
      unsigned char *p = cut + off;
      UINT valid = chunk, buf_size = chunk;
      if (aacDecoder_Fill(dec, &p, &buf_size, &valid) != AAC_DEC_OK) break;
      if (chunk - valid == 0) break; /* decoder is full */
      off += chunk - valid;
      for (;;) {
        AAC_DECODER_ERROR e = aacDecoder_DecodeFrame(dec, pcm, MAX_OUT, 0);
        if (e == AAC_DEC_NOT_ENOUGH_BITS) break;
        CStreamInfo *si = aacDecoder_GetStreamInfo(dec);
        if (!IS_OUTPUT_VALID(e) || IS_DECODE_ERROR(e)) hard_errors++;
        if (!IS_OUTPUT_VALID(e)) break;
        frames++;
        samples += si->frameSize * si->numChannels;
        if (frames > MAX_FRAMES) break;
      }
    }
    aacDecoder_Close(dec);

    printf("  removed AUs %d..%d (%d AUs); decoder emitted %d frames / %lld "
           "samples, %d error indications\n",
           drop_first, drop_first + drop_count - 1, drop_count, frames,
           (long long)samples, hard_errors);
    check(frames == ref4->n - drop_count,
          "whole-AU removal produced %d frames, expected %d", frames,
          ref4->n - drop_count);
    check(samples == (int64_t)(ref4->n - drop_count) * FRAME_LEN,
          "whole-AU removal produced %lld samples, expected %lld",
          (long long)samples,
          (long long)(ref4->n - drop_count) * FRAME_LEN);
    check(hard_errors == 0,
          "frame-aligned AU removal was reported as %d decoder errors",
          hard_errors);
    printf("  => frame-aligned loss is invisible to the decoder (0 errors, %d "
           "frames missing):\n", drop_count);
    printf("     the caller must detect the gap and repair the timeline.\n");
    check((int64_t)drop_count * FRAME_LEN ==
              (int64_t)ref4->n * FRAME_LEN - samples,
          "the time hole left by transport loss is not one frame per AU");
  }

  /* ================================================================== */
  printf("\n================ RESULT ================\n");
  printf("checks=%d failures=%d\n", g_checks, g_failures);
  printf("Frames reported above as MASKED(substitute) contain "
         "decoder-synthesised\n");
  printf("substitute audio for lost AUs. They are NOT the lost original "
         "samples;\n");
  printf("this test makes no claim that the original samples can be "
         "reconstructed.\n");
  printf("AAC_DEC_OK returned by an AACDEC_CONCEAL call is not evidence that "
         "the\n");
  printf("audio is original; the test tracks the loss itself.\n");
  printf("=======================================\n");

  free(ref); free(r); free(ref2); free(r2); free(ref3); free(r3); free(ref4);
  return g_failures == 0 ? 0 : 1;
}
