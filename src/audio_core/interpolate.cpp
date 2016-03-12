// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

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

    auto step = [&](const std::vector<s16>& in) -> s32 {
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
        return sample;
    };

    const size_t position_stop = input[0].size() - required_history;
    while (state.output_position < output[0].size() && position < position_stop) {
        s32 sample0 = step(input[0]);
        s32 sample1 = step(input[1]);

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

}
