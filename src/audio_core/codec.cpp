// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <vector>

#include "audio_core/codec.h"

#include "common/common_types.h"
#include "common/math_util.h"

namespace Codec {

std::vector<s16> DecodeADPCM(const u8 * const data, const size_t sample_count, const std::array<s16, 16>& adpcm_coeff, AdpcmState& state) {
    // GC-ADPCM with scale factor and variable coefficients.
    // Frames are 8 bytes long containing 14 samples each.
    // Samples are 4 bits (one nybble) long.

    constexpr size_t FRAME_LEN = 8;
    constexpr size_t SAMPLES_PER_FRAME = 14;
    constexpr std::array<int, 16> SIGNED_NYBBLES {{ 0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1 }};

    std::vector<s16> ret(sample_count % 2 == 0 ? sample_count : sample_count + 1); // Ensure ret.size() is a multiple of two.
    int yn1 = state.yn1,
        yn2 = state.yn2;

    const int NUM_FRAMES = (sample_count + (SAMPLES_PER_FRAME - 1)) / SAMPLES_PER_FRAME; // Round up.
    for (int framei = 0; framei < NUM_FRAMES; framei++) {
        const int frame_header = data[framei * FRAME_LEN];
        const int scale = 1 << (frame_header & 0xF);
        const int idx = (frame_header >> 4) & 0x7;

        // Coefficients are fixed point with 11 bits fractional part.
        const int coef1 = adpcm_coeff[idx * 2 + 0];
        const int coef2 = adpcm_coeff[idx * 2 + 1];

        // Decodes an audio sample. One nybble produces one sample.
        const auto decode_sample = [&](int nybble) -> s16 {
            int xn = nybble * scale;
            // We first transform everything into 11 bit fixed point, perform the second order digital filter, then transform back.
            // 0x400 == 0.5 in 11 bit fixed point.
            // Filter: y[n] = x[n] + 0.5 + c1 * y[n-1] + c2 * y[n-2]
            int val = ((xn << 11) + 0x400 + coef1 * yn1 + coef2 * yn2) >> 11;
            // Clamp to output range.
            val = MathUtil::Clamp(val, -32768, 32767);
            // Advance output feedback.
            yn2 = yn1;
            yn1 = val;
            return (s16)val;
        };

        int outputi = framei * SAMPLES_PER_FRAME;
        int datai = framei * FRAME_LEN + 1;
        for (int i = 0; i < SAMPLES_PER_FRAME && outputi < sample_count; i += 2) {
            ret[outputi++] = decode_sample(SIGNED_NYBBLES[data[datai] & 0xF]);
            ret[outputi++] = decode_sample(SIGNED_NYBBLES[data[datai] >> 4]);
            datai++;
        }
    }

    state.yn1 = yn1;
    state.yn2 = yn2;

    return ret;
}

std::vector<s16> DecodePCM8(const u8 * const data, const size_t sample_count) {
    std::vector<s16> ret(sample_count);
    for (size_t i = 0; i < sample_count; i++) {
        // Sign-extend value to 16 bits.
        ret[i] = (s16)(s8)data[i];
    }
    return ret;
}

std::vector<s16> DecodePCM16(const u8 * const data, const size_t sample_count) {
    std::vector<s16> ret(sample_count);
    std::memcpy(ret.data(), data, sample_count * sizeof(s16));
    return ret;
}

};
