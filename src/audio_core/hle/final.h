// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "audio_core/hle/dsp.h"

namespace DSP {
namespace HLE {

void FinalInit();

void FinalUpdate(const DspConfiguration& config, DspStatus& status, FinalMixSamples& samples);

const Frame16& FinalFrame();

}
}
