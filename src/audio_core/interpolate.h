// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <vector>

#include "audio_core/hle/common.h"

#include "common/common_types.h"

namespace AudioInterp {

constexpr size_t required_history = 11;

struct State {
    double position_fractional = 0;
    size_t output_position = 0;
    std::array<std::array<s16, required_history>, 2> history = {};
};

std::tuple<size_t, bool> KaiserSinc(State& state, DSP::HLE::QuadFrame32& output, std::array<std::vector<s16>, 2>& input, const float rate_change);

std::tuple<size_t, bool> Linear(State& state, DSP::HLE::QuadFrame32& output, std::array<std::vector<s16>, 2>& input, const float rate_change);

std::tuple<size_t, bool> None(State& state, DSP::HLE::QuadFrame32& output, std::vector<std::array<s16, 2>>& input, const float rate_change);

}
