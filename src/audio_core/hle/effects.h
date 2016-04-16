// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "audio_core/hle/common.h"
#include "audio_core/hle/dsp.h"

namespace DSP {
namespace HLE {

void EffectsInit();

void EffectsUpdate(const DspConfiguration& config, IntermediateMixSamples& samples);

const QuadFrame32& IntermediateMixFrame(int mix_id);

}
}
