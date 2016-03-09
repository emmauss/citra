// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "audio_core/audio_core.h"
#include "audio_core/hle/effects.h"
#include "audio_core/hle/dsp.h"
#include "audio_core/hle/final.h"
#include "audio_core/hle/pipe.h"
#include "audio_core/hle/source.h"

#include "core/hle/service/dsp_dsp.h"

namespace DSP {
namespace HLE {

SharedMemory g_region0;
SharedMemory g_region1;

void Init() {
    ResetPipes();
    SourceInit();
    EffectsInit();
    FinalInit();
}

void Shutdown() {
}

static bool next_region_is_ready = true;

bool Tick() {
    if (GetDspState() != DspState::On || !DSP_DSP::SemaphoreSignalled())
        return false;

    auto& region = CurrentRegion();

    for (int i = 0; i < AudioCore::num_sources; i++) {
        auto& config = region.source_configurations.config[i];
        auto& coeffs = region.adpcm_coefficients.coeff[i];
        auto& status = region.source_statuses.status[i];

        SourceUpdate(i, config, coeffs, status);
    }

    EffectsUpdate(region.dsp_configuration, region.intermediate_mix_samples);

    FinalUpdate(region.dsp_configuration, region.dsp_status, region.final_samples);

    return true;
}

SharedMemory& CurrentRegion() {
    // The region with the higher frame counter is chosen unless there is wraparound.

    if (g_region0.frame_counter == 0xFFFFu && g_region1.frame_counter != 0xFFFEu) {
        // Wraparound has occured.
        return g_region1;
    }

    if (g_region1.frame_counter == 0xFFFFu && g_region0.frame_counter != 0xFFFEu) {
        // Wraparound has occured.
        return g_region0;
    }

    return (g_region0.frame_counter > g_region1.frame_counter) ? g_region0 : g_region1;
}

} // namespace HLE
} // namespace DSP
