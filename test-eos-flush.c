/* ------------------------------------------------------------------
 * Copyright (C) 2026
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * -------------------------------------------------------------------
 */

/*
 * Integration test for the AAC encoder end-of-stream path.
 *
 * The focus is what happens when the PCM input ends in the middle of an
 * encoder frame (a length that is NOT a multiple of info.frameLength):
 *
 *   1. The encoder is fed the full signal in arbitrary small chunks so the
 *      internal PCM input buffer is exercised (sub-frame accumulation). Each
 *      call may consume only part of the chunk (documented modulo-buffer
 *      behaviour of aacEncEncode); unconsumed samples are retained and fed
 *      again on the next call.
 *   2. The end of stream is signalled explicitly with numInSamples = -1 and
 *      the encoder is drained repeatedly until it returns AACENC_ENCODE_EOF.
 *      On this path the encoder zero-fills the incomplete last frame and
 *      appends up to nDelay zero samples (see nZerosAppended in
 *      cpp/libAACenc/src/aacenc_lib.cpp).
 *   3. Every emitted access unit is decoded again, one packet per decoder
 *      call, and the decoded samples are partitioned into three strictly
 *      separated categories by position per channel:
 *
 *        - head priming : the first nDelay samples/channel, caused by codec
 *                          delay, not part of the audible programme;
 *        - audible      : exactly one decoded sample per originally fed PCM
 *                          sample/channel (the actual programme duration);
 *        - tail padding : zero samples the encoder appended to complete the
 *                          last frame and to flush its delay lines. These are
 *                          coding padding, NOT original audio.
 *
 * The test asserts the accounting identity
 *
 *	total_decoded_per_channel = nDelay + input_per_channel + tail_padding
 *
 * with the tail padding pinned to the deterministic frame-alignment remainder
 *
 *	tail_padding = frameLength - ((nDelay + input) mod frameLength)
 *	             = 0 when (nDelay + input) is already frame aligned
 *
 * and verifies that the tail-padding region carries no programme energy
 * (only filter-bank ringing around the appended zeros), while the audible
 * region really carries the input signal. It deliberately never claims that
 * all decoded samples are original audio: the container duration (all decoded
 * samples) and the effective duration (audible samples only) are reported and
 * checked separately.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aacenc_lib.h"
#include "aacdecoder_lib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* A decoded sample whose absolute value is below this threshold is treated
 * as silence. The encoder input is a full-scale-ish sine wave (~8500 LSB at
 * its quietest phase), so genuine programme samples are far above this. */
#define SILENCE_THRESHOLD 64

/* Upper bound for the number of explicit drain calls; the real flush takes a
 * handful of frames. This only guards against an infinite loop on a bug. */
#define MAX_DRAIN_CALLS 64

/* Initial number of packet descriptor slots. */
#define INITIAL_PACKETS 16

/* One emitted access unit: pointer into the shared storage and its length,
 * keeping packet boundaries so the decoder can be fed one AU at a time. */
typedef struct {
	size_t offset;
	int length;
	int from_drain; /* 1 if produced while explicitly draining */
} Packet;

typedef struct {
	uint8_t *data;
	size_t size;
	size_t capacity;
	Packet *packets;
	int num_packets;
	int packet_capacity;
	int drain_packets;
	size_t drain_bytes;
} PacketBuffer;

static void pb_init(PacketBuffer *pb) {
	pb->capacity = 8192;
	pb->data = (uint8_t *)malloc(pb->capacity);
	pb->packet_capacity = INITIAL_PACKETS;
	pb->packets = (Packet *)malloc(pb->packet_capacity * sizeof(Packet));
	if (!pb->data || !pb->packets)
		abort();
	pb->size = 0;
	pb->num_packets = 0;
	pb->drain_packets = 0;
	pb->drain_bytes = 0;
}

static void pb_free(PacketBuffer *pb) {
	free(pb->data);
	free(pb->packets);
	pb->data = NULL;
	pb->packets = NULL;
}

static void pb_append(PacketBuffer *pb, const uint8_t *buf, int bytes,
                      int draining) {
	if (pb->size + (size_t)bytes > pb->capacity) {
		while (pb->size + (size_t)bytes > pb->capacity)
			pb->capacity *= 2;
		uint8_t *p = (uint8_t *)realloc(pb->data, pb->capacity);
		if (!p)
			abort();
		pb->data = p;
	}
	if (pb->num_packets >= pb->packet_capacity) {
		pb->packet_capacity *= 2;
		Packet *p = (Packet *)realloc(pb->packets,
		                              pb->packet_capacity * sizeof(Packet));
		if (!p)
			abort();
		pb->packets = p;
	}
	memcpy(pb->data + pb->size, buf, bytes);
	pb->packets[pb->num_packets].offset = pb->size;
	pb->packets[pb->num_packets].length = bytes;
	pb->packets[pb->num_packets].from_drain = draining;
	pb->size += bytes;
	pb->num_packets++;
	if (draining) {
		pb->drain_packets++;
		pb->drain_bytes += bytes;
	}
}

/* Synthetic 440 Hz sine wave at ~37% full scale. A fixed pi/4 phase offset
 * guarantees that even sample index 0 is non-zero; the channel term keeps
 * stereo channels distinguishable. */
static int16_t make_sample(int sample_index, int channel) {
	double phase = 2.0 * M_PI * 440.0 * sample_index / 32000.0 + M_PI / 4.0 +
	               0.13 * channel;
	double v = sin(phase);
	return (int16_t)(v * 12000.0);
}

static const char *aot_name(int aot) {
	switch (aot) {
	case 2:
		return "AAC-LC";
	case 23:
		return "AAC-LD";
	case 39:
		return "AAC-ELD";
	default:
		return "unknown";
	}
}

/*
 * Run one encode -> drain -> decode case.
 *
 * aot            : audio object type (2 LC, 23 LD, 39 ELD)
 * channels       : 1 (mono) or 2 (stereo)
 * sample_rate    : core sample rate in Hz
 * input_per_ch   : number of deliberately generated PCM samples per channel;
 *                  chosen NOT to be a multiple of the encoder frame length.
 */
static int run_case(int aot, int channels, int sample_rate, int input_per_ch) {
	CHANNEL_MODE mode = (channels == 2) ? MODE_2 : MODE_1;
	/* ADTS only carries AAC-LC here; LD/ELD use raw frames plus an ASC handed
	 * to the decoder through aacDecoder_ConfigRaw. */
	int use_adts = (aot == 2);
	TRANSPORT_TYPE transport = use_adts ? TT_MP4_ADTS : TT_MP4_RAW;
	HANDLE_AACENCODER encoder = NULL;
	HANDLE_AACDECODER decoder = NULL;
	AACENC_InfoStruct info;
	AACENC_ERROR enc_err;
	AAC_DECODER_ERROR dec_err;
	PacketBuffer packets;
	int16_t *pcm = NULL;
	int16_t *decoded = NULL;
	int ret = 1; /* fail unless we reach the success return */

	int total_in_samples = input_per_ch * channels;
	int consumed_total = 0;
	int drain_calls = 0;
	int got_eof = 0;

	memset(&info, 0, sizeof(info));
	pb_init(&packets);

	pcm = (int16_t *)malloc((size_t)total_in_samples * sizeof(int16_t));
	if (!pcm)
		abort();
	for (int i = 0; i < input_per_ch; i++)
		for (int c = 0; c < channels; c++)
			pcm[i * channels + c] = make_sample(i, c);

	/* ---- encoder setup --------------------------------------------------- */
	if (aacEncOpen(&encoder, 0, channels) != AACENC_OK) {
		fprintf(stderr, "  [FAIL] aacEncOpen\n");
		goto cleanup;
	}
	if (aacEncoder_SetParam(encoder, AACENC_AOT, aot) != AACENC_OK ||
	    aacEncoder_SetParam(encoder, AACENC_SAMPLERATE, sample_rate) !=
	        AACENC_OK ||
	    aacEncoder_SetParam(encoder, AACENC_CHANNELMODE, mode) != AACENC_OK ||
	    aacEncoder_SetParam(encoder, AACENC_CHANNELORDER, 1) != AACENC_OK ||
	    aacEncoder_SetParam(encoder, AACENC_BITRATE, 64000) != AACENC_OK ||
	    aacEncoder_SetParam(encoder, AACENC_TRANSMUX, use_adts ? 2 : 0) !=
	        AACENC_OK ||
	    aacEncoder_SetParam(encoder, AACENC_AFTERBURNER, 1) != AACENC_OK) {
		fprintf(stderr, "  [FAIL] encoder parameter setup\n");
		goto cleanup;
	}
	if (aacEncEncode(encoder, NULL, NULL, NULL, NULL) != AACENC_OK) {
		fprintf(stderr, "  [FAIL] encoder initialization\n");
		goto cleanup;
	}
	if (aacEncInfo(encoder, &info) != AACENC_OK) {
		fprintf(stderr, "  [FAIL] aacEncInfo\n");
		goto cleanup;
	}

	printf("  %s %d Hz %s: frameLength=%u nDelay=%u input/channel=%d\n",
	       aot_name(aot), sample_rate, channels == 2 ? "stereo" : "mono",
	       info.frameLength, info.nDelay, input_per_ch);

	/* ---- phase 1: feed the signal in arbitrary small chunks -------------- */
	{
		const int chunk_per_ch = 37; /* deliberately not frame-aligned */
		int feed_pos = 0; /* next unconsumed sample in pcm */
		while (feed_pos < total_in_samples) {
			int remaining = total_in_samples - feed_pos;
			int chunk = chunk_per_ch * channels;
			if (chunk > remaining)
				chunk = remaining;

			AACENC_BufDesc in_buf = {0}, out_buf = {0};
			AACENC_InArgs in_args = {0};
			AACENC_OutArgs out_args = {0};
			int in_id = IN_AUDIO_DATA, out_id = OUT_BITSTREAM_DATA;
			int in_size = chunk * (int)sizeof(int16_t), in_el = 2;
			uint8_t outbuf[20480];
			int out_size = (int)sizeof(outbuf), out_el = 1;
			void *in_ptr = pcm + feed_pos, *out_ptr = outbuf;

			in_buf.numBufs = 1;
			in_buf.bufs = &in_ptr;
			in_buf.bufferIdentifiers = &in_id;
			in_buf.bufSizes = &in_size;
			in_buf.bufElSizes = &in_el;
			out_buf.numBufs = 1;
			out_buf.bufs = &out_ptr;
			out_buf.bufferIdentifiers = &out_id;
			out_buf.bufSizes = &out_size;
			out_buf.bufElSizes = &out_el;
			in_args.numInSamples = chunk;

			enc_err = aacEncEncode(encoder, &in_buf, &out_buf, &in_args,
			                       &out_args);
			if (enc_err != AACENC_OK) {
				fprintf(stderr,
				        "  [FAIL] aacEncEncode during feeding: 0x%x\n",
				        enc_err);
				goto cleanup;
			}
			if (out_args.numInSamples < 0 ||
			    out_args.numInSamples > chunk) {
				fprintf(stderr,
				        "  [FAIL] implausible consumed count %d for chunk "
				        "of %d\n",
				        out_args.numInSamples, chunk);
				goto cleanup;
			}
			/* Retain any samples the encoder did not take; they are re-fed
			 * on the next iteration (modulo input buffer behaviour). */
			feed_pos += out_args.numInSamples;
			consumed_total += out_args.numInSamples;
			if (out_args.numOutBytes > 0)
				pb_append(&packets, outbuf, out_args.numOutBytes, 0);
		}
	}

	/* ---- phase 2: explicit draining (numInSamples = -1) until EOF -------- */
	do {
		AACENC_BufDesc out_buf = {0};
		AACENC_InArgs in_args = {0};
		AACENC_OutArgs out_args = {0};
		int out_id = OUT_BITSTREAM_DATA;
		uint8_t outbuf[20480];
		int out_size = (int)sizeof(outbuf), out_el = 1;
		void *out_ptr = outbuf;

		out_buf.numBufs = 1;
		out_buf.bufs = &out_ptr;
		out_buf.bufferIdentifiers = &out_id;
		out_buf.bufSizes = &out_size;
		out_buf.bufElSizes = &out_el;
		in_args.numInSamples = -1; /* explicit end-of-stream / flush */

		enc_err = aacEncEncode(encoder, NULL, &out_buf, &in_args, &out_args);
		drain_calls++;
		if (enc_err == AACENC_ENCODE_EOF) {
			got_eof = 1;
			if (out_args.numOutBytes != 0) {
				fprintf(stderr,
				        "  [FAIL] EOF returned with %d non-empty output "
				        "bytes\n",
				        out_args.numOutBytes);
				goto cleanup;
			}
			break;
		}
		if (enc_err != AACENC_OK) {
			fprintf(stderr, "  [FAIL] encoder drain error: 0x%x\n", enc_err);
			goto cleanup;
		}
		if (out_args.numOutBytes > 0)
			pb_append(&packets, outbuf, out_args.numOutBytes, 1);
	} while (drain_calls < MAX_DRAIN_CALLS);

	if (!got_eof) {
		fprintf(stderr,
		        "  [FAIL] encoder never signalled AACENC_ENCODE_EOF within "
		        "%d drain calls\n",
		        MAX_DRAIN_CALLS);
		goto cleanup;
	}
	if (packets.drain_packets <= 0) {
		fprintf(stderr,
		        "  [FAIL] explicit draining produced no access units; the "
		        "tail path was not exercised\n");
		goto cleanup;
	}
	if (consumed_total != total_in_samples) {
		fprintf(stderr,
		        "  [FAIL] encoder consumed %d samples, expected %d\n",
		        consumed_total, total_in_samples);
		goto cleanup;
	}

	/* ---- phase 3: decode every emitted access unit ----------------------- */
	decoder = aacDecoder_Open(transport, 1);
	if (!decoder) {
		fprintf(stderr, "  [FAIL] aacDecoder_Open\n");
		goto cleanup;
	}
	if (!use_adts) {
		UCHAR *conf_array[] = {info.confBuf};
		UINT conf_size = info.confSize;
		dec_err = aacDecoder_ConfigRaw(decoder, conf_array, &conf_size);
		if (dec_err != AAC_DEC_OK) {
			fprintf(stderr,
			        "  [FAIL] aacDecoder_ConfigRaw error: 0x%x\n", dec_err);
			goto cleanup;
		}
	}
	aacDecoder_SetParam(decoder, AAC_CONCEAL_METHOD, 1);
	aacDecoder_SetParam(decoder, AAC_PCM_LIMITER_ENABLE, 0);

	{
		const int decode_capacity = info.frameLength * channels;
		int decoded_per_ch = 0;
		int decoded_frames = 0;
		int head_per_ch = (int)info.nDelay;
		int tail_pad_per_ch;
		int total_per_ch;
		int audible_count = 0;
		int tail_above_threshold = 0, tail_count = 0;
		int tail_max = 0, audible_min = 32767;
		int64_t tail_sum = 0, audible_sum = 0;

		decoded = (int16_t *)malloc((size_t)decode_capacity *
		                            sizeof(int16_t));
		if (!decoded)
			abort();

		for (int p = 0; p < packets.num_packets; p++) {
			UINT valid, buffer_size;
			UCHAR *ptr = packets.data + packets.packets[p].offset;
			CStreamInfo *si;

			buffer_size = valid = (UINT)packets.packets[p].length;
			dec_err = aacDecoder_Fill(decoder, &ptr, &buffer_size, &valid);
			if (dec_err != AAC_DEC_OK) {
				fprintf(stderr,
				        "  [FAIL] aacDecoder_Fill error: 0x%x\n", dec_err);
				goto cleanup;
			}
			if (valid != 0) {
				fprintf(stderr,
				        "  [FAIL] decoder did not consume packet %d (%u "
				        "bytes left)\n",
				        p, valid);
				goto cleanup;
			}

			dec_err = aacDecoder_DecodeFrame(decoder, (INT_PCM *)decoded,
			                                 decode_capacity, 0);
			if (dec_err != AAC_DEC_OK) {
				fprintf(stderr,
				        "  [FAIL] aacDecoder_DecodeFrame error: 0x%x\n",
				        dec_err);
				goto cleanup;
			}
			si = aacDecoder_GetStreamInfo(decoder);
			if (si->numChannels != channels) {
				fprintf(stderr,
				        "  [FAIL] decoded channel count %d != %d\n",
				        si->numChannels, channels);
				goto cleanup;
			}
			if (si->frameSize != (int)info.frameLength) {
				fprintf(stderr,
				        "  [FAIL] decoded frameSize %d != encoder "
				        "frameLength %u\n",
				        si->frameSize, info.frameLength);
				goto cleanup;
			}

			/* Classify each sample of this frame. The partition is purely
			 * positional and keeps original audio and coding padding apart. */
			for (int s = 0; s < si->frameSize; s++) {
				int global_per_ch = decoded_per_ch + s;
				for (int c = 0; c < channels; c++) {
					int v = decoded[s * channels + c];
					if (global_per_ch < head_per_ch) {
						/* codec priming delay: not programme audio */
						continue;
					} else if (global_per_ch < head_per_ch + input_per_ch) {
						audible_sum += abs(v);
						audible_count++;
						if (abs(v) < audible_min)
							audible_min = abs(v);
					} else {
						tail_sum += abs(v);
						tail_count++;
						if (abs(v) > tail_max)
							tail_max = abs(v);
						if (abs(v) > SILENCE_THRESHOLD)
							tail_above_threshold++;
					}
				}
			}
			decoded_per_ch += si->frameSize;
			decoded_frames++;
		}

		/* ---- accounting -------------------------------------------------- */
		total_per_ch = decoded_per_ch;
		tail_pad_per_ch = total_per_ch - head_per_ch - input_per_ch;

		/* Deterministic expectation. The encoder emits one frame whenever it
		 * has frameLength samples buffered, and the input is padded until
		 * nDelay zeros have been appended, so the total number of frames is
		 * ceil((nDelay + N) / frameLength). The tail padding is therefore the
		 * frame-alignment remainder of (nDelay + N); it is zero exactly when
		 * nDelay + N is a multiple of frameLength (a frame-aligned LC input,
		 * where nDelay itself is a whole number of frames). */
		{
			int sum = head_per_ch + input_per_ch;
			int remainder = sum % (int)info.frameLength;
			int expected_tail =
			    remainder ? (int)info.frameLength - remainder : 0;
			int expected_total = sum + expected_tail;

			printf("    access units           : %d (%d from explicit "
			       "draining, %zu drain bytes)\n",
			       packets.num_packets, packets.drain_packets,
			       packets.drain_bytes);
			printf("    drain calls until EOF  : %d\n", drain_calls);
			printf("    encoder consumed       : %d samples (%d/channel) "
			       "== input: %s\n",
			       consumed_total, consumed_total / channels,
			       consumed_total == total_in_samples ? "yes" : "NO");
			printf("    decoded frames         : %d\n", decoded_frames);
			printf("    head priming (delay)   : %d samples/channel (%.3f "
			       "ms) [not audible]\n",
			       head_per_ch, 1000.0 * head_per_ch / sample_rate);
			printf("    audible programme      : %d samples/channel (%.3f "
			       "ms) [original audio]\n",
			       input_per_ch,
			       1000.0 * input_per_ch / sample_rate);
			printf("    tail coding padding    : %d samples/channel (%.3f "
			       "ms) [zero fill, not original audio]\n",
			       tail_pad_per_ch,
			       1000.0 * tail_pad_per_ch / sample_rate);
			printf("    container total        : %d samples/channel (%.3f "
			       "ms) = delay + audible + padding\n",
			       total_per_ch,
			       1000.0 * total_per_ch / sample_rate);
			printf("    effective duration     : %.3f ms of original audio "
			       "out of %.3f ms container duration\n",
			       1000.0 * input_per_ch / sample_rate,
			       1000.0 * total_per_ch / sample_rate);

			if (tail_pad_per_ch != expected_tail) {
				fprintf(stderr,
				        "  [FAIL] tail padding %d != frame-alignment "
				        "remainder %d (nDelay=%d input=%d frameLength=%u)\n",
				        tail_pad_per_ch, expected_tail, head_per_ch,
				        input_per_ch, info.frameLength);
				goto cleanup;
			}
			if (total_per_ch != expected_total ||
			    total_per_ch !=
			        decoded_frames * (int)info.frameLength) {
				fprintf(stderr,
				        "  [FAIL] decoded sample accounting inconsistent "
				        "(total=%d expected=%d frames*frameLength=%d)\n",
				        total_per_ch, expected_total,
				        decoded_frames * (int)info.frameLength);
				goto cleanup;
			}
		}

		/* The tail carries no programme. Low-delay filter banks can leave
		 * substantial overlap/ringing energy in the first padding samples, so
		 * a per-sample silence count is not reliable; the robust criterion is
		 * the mean-level separation between programme and padding. */
		if (tail_count > 0) {
			double audible_mean = (double)audible_sum / audible_count;
			double tail_mean = (double)tail_sum / tail_count;
			printf("    tail padding level     : mean |sample|=%.4f, max=%d "
			       "(%d/%d above silence threshold %d)\n",
			       tail_mean, tail_max, tail_above_threshold, tail_count,
			       SILENCE_THRESHOLD);
			printf("    audible mean |sample|  : %.2f (min=%d)\n",
			       audible_mean, audible_min);
			printf("    level separation       : %.1fx\n",
			       audible_mean / (tail_mean + 1.0));
			if (audible_mean < 15.0 * (tail_mean + 1.0)) {
				fprintf(stderr,
				        "  [FAIL] audible/padding level separation too "
				        "small (audible mean=%.2f, padding mean=%.2f)\n",
				        audible_mean, tail_mean);
				goto cleanup;
			}
		} else {
			printf("    tail padding level     : none (frame-aligned "
			       "input, flush fully covered by nDelay)\n");
			printf("    audible mean |sample|  : %.2f\n",
			       (double)audible_sum / audible_count);
		}

		/* The audible region must contain exactly input_per_ch samples per
		 * channel and must carry the programme, not silence. */
		if (audible_count != input_per_ch * channels) {
			fprintf(stderr,
			        "  [FAIL] audible region has %d samples, expected "
			        "%d\n",
			        audible_count, input_per_ch * channels);
			goto cleanup;
		}
		if ((double)audible_sum / audible_count < 100.0) {
			fprintf(stderr,
			        "  [FAIL] audible region is near silence "
			        "(mean |sample|=%.2f); input signal lost?\n",
			        (double)audible_sum / audible_count);
			goto cleanup;
		}

		printf("  [PASS] %s: %d original + %d delay + %d padding "
		       "samples/channel kept separate\n",
		       aot_name(aot), input_per_ch, head_per_ch, tail_pad_per_ch);
	}

	ret = 0;

cleanup:
	free(decoded);
	free(pcm);
	pb_free(&packets);
	if (decoder)
		aacDecoder_Close(decoder);
	if (encoder)
		aacEncClose(&encoder);
	return ret;
}

int main(void) {
	int failures = 0;
	int cases = 0;

	/*
	 * Most input lengths are deliberately chosen to be non-frame-aligned for
	 * the LC frame length (1024), the LD length (512) and the ELD length
	 * (480/512), including sub-frame inputs shorter than a single encoder
	 * frame. A few frame-aligned control inputs are included to verify that
	 * their tail padding equals nDelay exactly.
	 */
	struct {
		int aot;
		int channels;
		int sample_rate;
		int input_per_ch;
	} table[] = {
		/* AAC-LC mono: frameLength 1024, nDelay 2048.
		 * 1 and 100 are shorter than a single encoder frame. */
		{2, 1, 32000, 1},
		{2, 1, 32000, 100},
		{2, 1, 32000, 1025},     /* one frame + a sub-frame remainder */
		{2, 1, 32000, 2500},     /* spans several frames, ends mid-frame */
		{2, 1, 44100, 777},
		/* AAC-LC stereo, same accounting per channel. */
		{2, 2, 32000, 100},
		{2, 2, 44100, 3001},
		/* Frame-aligned control inputs: tail padding must equal nDelay. */
		{2, 1, 32000, 1024},
		{2, 1, 32000, 2048},
		/* AAC-LD mono: frameLength 512, nDelay 512, raw transport. */
		{23, 1, 32000, 1},
		{23, 1, 32000, 333},
		{23, 1, 32000, 512},  /* aligned control: padding == nDelay */
		{23, 1, 44100, 1537},
		/* AAC-ELD mono: frameLength 480 at 44.1 kHz / 512 at 32 kHz,
		 * nDelay half a frame, raw transport. */
		{39, 1, 32000, 1},
		{39, 1, 32000, 250},
		{39, 1, 32000, 512},  /* aligned control: padding == nDelay */
		{39, 1, 44100, 961},
	};

	int n = (int)(sizeof(table) / sizeof(table[0]));
	for (int i = 0; i < n; i++) {
		cases++;
		printf("case %d/%d:\n", i + 1, n);
		failures += run_case(table[i].aot, table[i].channels,
		                     table[i].sample_rate,
		                     table[i].input_per_ch);
	}

	printf("\n%d cases, %d failures\n", cases, failures);
	return failures ? 1 : 0;
}
