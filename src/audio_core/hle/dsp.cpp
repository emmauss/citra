// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <condition_variable>
#include <mutex>
#include <thread>

#include "audio_core/audio_core.h"
#include "audio_core/hle/effects.h"
#include "audio_core/hle/dsp.h"
#include "audio_core/hle/final.h"
#include "audio_core/hle/pipe.h"
#include "audio_core/hle/source.h"
#include "audio_core/interpolate.h"
#include "audio_core/sink.h"
#include "audio_core/time_stretch.h"

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
    TimeStretch::Init();
}

void Shutdown() {
    TimeStretch::Shutdown();
}

static bool next_region_is_ready = true;

unsigned num_frames = 500;
double time_for_a_frame = 0.005;

static std::mutex mtx_start;
static std::condition_variable cv_start;
static volatile bool start = false;

static void ThreadFunc() {
    std::unique_lock<std::mutex> lck(mtx_start);
    while (true) {
        start = false;
        while (!start)
            cv_start.wait(lck);

        auto& region = CurrentRegion();

        for (int i = 0; i < AudioCore::num_sources; i++) {
            auto& config = region.source_configurations.config[i];
            auto& coeffs = region.adpcm_coefficients.coeff[i];
            auto& status = region.source_statuses.status[i];

            SourceUpdate(i, config, coeffs, status);
        }

        EffectsUpdate(region.dsp_configuration, region.intermediate_mix_samples);

        FinalUpdate(region.dsp_configuration, region.dsp_status, region.final_samples);

        const double sample_scale = (double)AudioCore::sink->GetNativeSampleRate() / (double)AudioCore::native_sample_rate;
        const double time_scale = time_for_a_frame / ((double)AudioCore::samples_per_frame / (double)AudioCore::native_sample_rate);
        double total_scale = sample_scale * time_scale;

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

static std::thread thread(ThreadFunc);

bool Tick() {
    if (GetDspState() != DspState::On || !DSP_DSP::SemaphoreSignalled())
        return false;

    std::unique_lock<std::mutex> lck(mtx_start);
    start = true;
    cv_start.notify_all();

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
