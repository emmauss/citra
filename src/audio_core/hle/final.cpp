// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "audio_core/hle/common.h"
#include "audio_core/hle/effects.h"
#include "audio_core/hle/final.h"

namespace DSP {
namespace HLE {

struct State {
    StereoFrame16 current_frame;
};

static State state;

void FinalInit() {
    state = {};
}

void FinalUpdate(const DspConfiguration& config, DspStatus& status, FinalMixSamples& samples) {
    // TODO: Final processing

    std::array<QuadFrame32, 3> mix;
    for (int k = 0; k < 3; k++) {
        mix[k] = IntermediateMixFrame(k);
    }

    for (int i = 0; i < AudioCore::samples_per_frame; i++) {
        for (int j = 0; j < 2; j++) {
            state.current_frame[j][i] = 0;
            for (int k = 0; k < 3; k++) {
                state.current_frame[j][i] += 0.5 * config.volume[0] * mix[k][j + 0][i];
                state.current_frame[j][i] += 0.5 * config.volume[0] * mix[k][j + 2][i];
            }
       }
    }
}

const StereoFrame16& FinalFrame() {
    return state.current_frame;
}

}
}
