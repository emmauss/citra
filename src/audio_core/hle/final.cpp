// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "audio_core/hle/common.h"
#include "audio_core/hle/effects.h"
#include "audio_core/hle/final.h"

#include "common/logging/log.h"
#include "common/math_util.h"

namespace DSP {
namespace HLE {

struct State {
    StereoFrame16 current_frame;
    std::array<float, 3> volumes = {1.0, 0.0, 0.0};
};

static State state;

void FinalInit() {
    state = {};
}

void FinalUpdate(const DspConfiguration& config, DspStatus& status, FinalMixSamples& samples) {
    // TODO: Final processing

    bool clipping = false;

    std::array<QuadFrame32, 3> mix;
    for (int k = 0; k < 3; k++) {
        mix[k] = IntermediateMixFrame(k);
    }

    for (int i = 0; i < AudioCore::samples_per_frame; i++) {
        for (int j = 0; j < 2; j++) {
            s32 value = 0;
            for (int k = 0; k < 3; k++) {
                value += 0.2 * state.volumes[0] * mix[k][i][j + 0];
                value += 0.2 * state.volumes[0] * mix[k][i][j + 2];
            }

            if (value > 0x8000 || value < -0x7FFF) {
                clipping = true;
            }

            state.current_frame[i][j] = static_cast<s16>(MathUtil::Clamp(value, -0x7FFF, 0x8000));
       }
    }

    if (clipping) {
        LOG_ERROR(Audio_DSP, "AUDIO IS CLIPPING");
    }
}

const StereoFrame16& FinalFrame() {
    return state.current_frame;
}

}
}
