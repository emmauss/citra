// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <atomic>
#include <thread>

#include "audio_core/audio_core.h"
#include "audio_core/hle/effects.h"
#include "audio_core/hle/dsp.h"
#include "audio_core/hle/final.h"
#include "audio_core/hle/pipe.h"
#include "audio_core/hle/source.h"
#include "audio_core/sink.h"
#include "audio_core/time_stretch.h"

#include "common/thread.h"

#include "core/hle/service/dsp_dsp.h"

namespace DSP {
namespace HLE {

SharedMemory g_region0;
SharedMemory g_region1;

static void ThreadFunc();
static Common::Barrier ThreadFunc_barrier(2);
static std::atomic<bool> ThreadFunc_quit = true;

void Init() {
    ResetPipes();
    SourceInit();
    EffectsInit();
    FinalInit();
    TimeStretch::Init();

    ThreadFunc_quit = false;
    std::thread thread(ThreadFunc);
    thread.detach();
}

void Shutdown() {
    TimeStretch::Shutdown();

    if (!ThreadFunc_quit) {
        ThreadFunc_quit = true;
        ThreadFunc_barrier.Sync();
    }
}

static bool next_region_is_ready = true;

unsigned num_frames = 500;
double time_for_a_frame = 0.005;

static void ThreadFunc() {
    while (true) {
        ThreadFunc_barrier.Sync();
        if (ThreadFunc_quit) {
            break;
        }

        auto& region = CurrentRegion();

        for (int i = 0; i < AudioCore::num_sources; i++) {
            auto& config = region.source_configurations.config[i];
            auto& coeffs = region.adpcm_coefficients.coeff[i];
            auto& status = region.source_statuses.status[i];

            SourceUpdate(i, config, coeffs, status);
        }

        EffectsUpdate(region.dsp_configuration, region.intermediate_mix_samples);

        FinalUpdate(region.dsp_configuration, region.dsp_status, region.final_samples);

        StereoFrame16 samples = FinalFrame();

#if 0
        std::vector<s16> output;
        output.reserve(AudioCore::samples_per_frame * 2);
        for (int i = 0; i < AudioCore::samples_per_frame; i++) {
            output.push_back(samples[0][i]);
            output.push_back(samples[1][i]);
        }
        AudioCore::sink->EnqueueSamples(output);
#else
        TimeStretch::Tick(AudioCore::sink->SamplesInQueue());
        TimeStretch::AddSamples(samples);
        TimeStretch::OutputSamples([&](const std::vector<s16>& output) {
            if (AudioCore::sink->SamplesInQueue() < 16000) {
                AudioCore::sink->EnqueueSamples(output);
            }
        });
#endif
    }
}



bool Tick() {
    if (GetDspState() != DspState::On || !DSP_DSP::SemaphoreSignalled())
        return false;

    ThreadFunc_barrier.Sync();

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
