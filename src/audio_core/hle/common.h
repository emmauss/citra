// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <vector>

#include "audio_core/audio_core.h"

#include "common/common_types.h"

namespace DSP {
namespace HLE {

/// The final output to the speakers is stereo.
using StereoFrame16  = std::array<std::array<s16, AudioCore::samples_per_frame>, 2>;

/// The DSP is quadraphonic internally.
using QuadFrame32    = std::array<std::array<s32, AudioCore::samples_per_frame>, 4>;

}
}
