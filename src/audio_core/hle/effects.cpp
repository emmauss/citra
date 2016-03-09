// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "audio_core/hle/common.h"
#include "audio_core/hle/effects.h"

namespace DSP {
namespace HLE {

struct State {
    QuadFrame32 current_frame;
};

static std::array<State, 3> state;

void EffectsInit() {
    state = {};
}

void EffectsUpdate(const DspConfiguration& config, IntermediateMixSamples& samples) {}

const QuadFrame32& IntermediateMixFrame(int mix_id) {
    return state[mix_id].current_frame;
}

}
}
