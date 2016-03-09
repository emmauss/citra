// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "audio_core/hle/common.h"
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

void FinalUpdate(const DspConfiguration& config, DspStatus& status, FinalMixSamples& samples) {}

const StereoFrame16& FinalFrame() {
    return state.current_frame;
}

}
}
