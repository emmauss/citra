// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <functional>
#include <vector>

#include "common/common_types.h"

namespace TimeStretch {

void Init();
void Shutdown();

void Tick(unsigned samples_in_queue);
void AddSamples(const std::array<std::array<s16, 2>, AudioCore::samples_per_frame>& samples);
void OutputSamples(std::function<void(const std::vector<s16>&)> fn);

}
