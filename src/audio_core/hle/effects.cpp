// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "audio_core/audio_core.h"
#include "audio_core/hle/common.h"
#include "audio_core/hle/effects.h"
#include "audio_core/hle/source.h"

namespace DSP {
namespace HLE {

struct State {
    QuadFrame32 current_frame;
};

static std::array<State, 3> state;

void EffectsInit() {
    state = {};
}

void EffectsUpdate(const DspConfiguration& config, IntermediateMixSamples& samples) {
    state[0].current_frame.fill({});
    state[1].current_frame.fill({});
    state[2].current_frame.fill({});

    for (size_t source_id = 0; source_id < AudioCore::num_sources; source_id++) {
        SourceFrameMixInto(state[0].current_frame, source_id, 0);
        SourceFrameMixInto(state[1].current_frame, source_id, 1);
        SourceFrameMixInto(state[2].current_frame, source_id, 2);
    }

    // TODO: Delay
    // TODO: Reverb

}

const QuadFrame32& IntermediateMixFrame(int mix_id) {
    return state[mix_id].current_frame;
}

}
}
