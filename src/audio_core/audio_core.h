// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

namespace Kernel {
class VMManager;
}

namespace AudioCore {

constexpr size_t num_sources = 24;
constexpr size_t samples_per_frame = 160; ///< Samples per audio frame at native sample rate
constexpr unsigned native_sample_rate = 32728; ///< 32kHz

/// Initialise Audio Core
void Init();

/// Add DSP address spaces to a Process.
void AddAddressSpace(Kernel::VMManager& vm_manager);

/// Shutdown Audio Core
void Shutdown();

} // namespace
