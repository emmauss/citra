// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#define _USE_MATH_DEFINES
#include <cmath>

#include "audio_core/interpolate.h"

#include "common/assert.h"
#include "common/math_util.h"

namespace AudioInterp {

/// Kaiser window with alpha=2.4, N=11
static const std::array<double, required_history/2> kaiser_window {{
    0.327949,
    0.521302,
    0.707379,
    0.861996,
    0.964250
}};

struct BiquadLpf {
    /// Calculate coefficients required for a biquad filter to behave as a low-pass filter.
    void Init(double freq) {
        const double w0 = 2 * M_PI * freq;
        const double Q = 0.707;
        const double a = sin(w0) / (2.0*Q);
        double a0;

        b0 = 0.5 * (1.0 - cos(w0));
        b1 =       (1.0 - cos(w0));
        b2 = 0.5 * (1.0 - cos(w0));
        a0 = 1.0 + a;
        a1 = -2.0 * cos(w0);
        a2 = 1.0 - a;

        // Normalize;
        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;
    }
    inline s32 Process(s32 x) {
        double xn0 = x;
        double yn0 = b0 * xn0 + b1 * xn1 + b2 * xn2 - a1 * yn1 - a2 * yn2;

        // Advance state
        xn2 = xn1;
        xn1 = xn0;
        yn2 = yn1;
        yn1 = yn0;

        return (s32)yn0;
    }
private:
    double a1, a2;
    double b0, b1, b2;
    double xn1 = 0.0, xn2 = 0.0, yn1 = 0.0, yn2 = 0.0;
};

double sinc(double x) {
    DEBUG_ASSERT(x != 0);
    return sin(x) / x;
}

std::tuple<size_t, bool> KaiserSinc(State& state, DSP::HLE::QuadFrame32& output, std::array<std::vector<s16>, 2>& input, const float rate_change) {
    ASSERT(input[0].size() == input[1].size());
    ASSERT(input[0].size() > required_history);

    size_t position = 0;
    double& position_fractional = state.position_fractional;

    for (int j = 0; j < 2; j++) {
        input[j].insert(input[j].begin(), state.history[j].begin(), state.history[j].end());
    }

    std::array<BiquadLpf, 2> lpf;
    const double lpf_cutoff = std::min(0.5 * rate_change, 0.5 / rate_change);
    lpf[0].Init(lpf_cutoff);
    lpf[1].Init(lpf_cutoff);

    auto step = [&](size_t i) -> s32 {
        auto& in = input[i];
        s32 sample = 0;
        sample += kaiser_window[0] * sinc(-5.0 - position_fractional) * in[position + 0];
        sample += kaiser_window[1] * sinc(-4.0 - position_fractional) * in[position + 1];
        sample += kaiser_window[2] * sinc(-3.0 - position_fractional) * in[position + 2];
        sample += kaiser_window[3] * sinc(-2.0 - position_fractional) * in[position + 3];
        sample += kaiser_window[4] * sinc(-1.0 - position_fractional) * in[position + 4];
        sample +=                                                       in[position + 5];
        sample += kaiser_window[4] * sinc(+1.0 - position_fractional) * in[position + 6];
        sample += kaiser_window[3] * sinc(+2.0 - position_fractional) * in[position + 7];
        sample += kaiser_window[2] * sinc(+3.0 - position_fractional) * in[position + 8];
        sample += kaiser_window[1] * sinc(+4.0 - position_fractional) * in[position + 9];
        sample += kaiser_window[0] * sinc(+5.0 - position_fractional) * in[position + 10];
        //sample = lpf[i].Process(sample);
        return sample;
    };

    const size_t position_stop = input[0].size() - required_history;
    while (state.output_position < output[0].size() && position < position_stop) {
        s32 sample0 = step(0);
        s32 sample1 = step(1);

        output[0][state.output_position] = sample0;
        output[1][state.output_position] = sample0;
        output[2][state.output_position] = sample1;
        output[3][state.output_position] = sample1;

        position_fractional += rate_change;
        position += (size_t)position_fractional;
        position_fractional -= (size_t)position_fractional;

        state.output_position++;
    }

    bool continue_feeding_me = true;
    if (state.output_position >= output[0].size()) {
        state.output_position = 0;
        continue_feeding_me = false;
    }

    for (int j = 0; j < 2; j++) {
        std::copy(input[j].begin() + position,
                  input[j].begin() + position + required_history,
                  state.history[j].begin());
        if (position + required_history >= input[j].size()) {
            input[j].clear();
        } else {
            input[j].erase(input[j].begin(),
                           input[j].begin() + position + required_history);
        }
    }

    ASSERT(input[0].size() == input[1].size());

    return std::make_tuple(position, continue_feeding_me);
}

std::tuple<size_t, bool> Linear(State& state, DSP::HLE::QuadFrame32& output, std::array<std::vector<s16>, 2>& input, const float rate_change) {
    ASSERT(input[0].size() == input[1].size());
    while (input[0].size() < 2) {
        input[0].emplace_back(0);
        input[1].emplace_back(0);
    }

    size_t position = 0;
    double& position_fractional = state.position_fractional;

    auto step = [&](size_t i) -> s32 {
        auto& in = input[i];
        s32 sample = 0;
        sample = position_fractional * in[position + 0] + (1.0 - position_fractional) * in[position + 1];
        return sample;
    };

    const size_t position_stop = input[0].size() - 1;
    while (state.output_position < output[0].size() && position < position_stop) {
        s32 sample0 = step(0);
        s32 sample1 = step(1);

        output[0][state.output_position] = sample0;
        output[1][state.output_position] = sample0;
        output[2][state.output_position] = sample1;
        output[3][state.output_position] = sample1;

        position_fractional += rate_change;
        position += (size_t)position_fractional;
        position_fractional -= (size_t)position_fractional;

        state.output_position++;
    }

    bool continue_feeding_me = true;
    if (state.output_position >= output[0].size()) {
        state.output_position = 0;
        continue_feeding_me = false;
    }

    for (int j = 0; j < 2; j++) {
        if (position >= input[j].size()) {
            input[j].clear();
        } else {
            input[j].erase(input[j].begin(),
                           input[j].begin() + position);
        }
    }

    ASSERT(input[0].size() == input[1].size());

    return std::make_tuple(position, continue_feeding_me);
}

std::tuple<size_t, bool> None(State& state, DSP::HLE::QuadFrame32& output, std::array<std::vector<s16>, 2>& input, const float rate_change) {
    ASSERT(input[0].size() == input[1].size());

    size_t position = 0;
    double& position_fractional = state.position_fractional;

    auto step = [&](size_t i) -> s32 {
        return input[i][position];
    };

    const size_t position_stop = input[0].size();
    while (state.output_position < output[0].size() && position < position_stop) {
        s32 sample0 = step(0);
        s32 sample1 = step(1);

        output[0][state.output_position] = sample0;
        output[1][state.output_position] = sample0;
        output[2][state.output_position] = sample1;
        output[3][state.output_position] = sample1;

        position_fractional += rate_change;
        position += (size_t)position_fractional;
        position_fractional -= (size_t)position_fractional;

        state.output_position++;
    }

    bool continue_feeding_me = true;
    if (state.output_position >= output[0].size()) {
        state.output_position = 0;
        continue_feeding_me = false;
    }

    for (int j = 0; j < 2; j++) {
        if (position >= input[j].size()) {
            input[j].clear();
        } else {
            input[j].erase(input[j].begin(), input[j].begin() + position);
        }
    }

    ASSERT(input[0].size() == input[1].size());

    return std::make_tuple(position, continue_feeding_me);
}

}
