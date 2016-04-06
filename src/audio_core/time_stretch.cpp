// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <list>
#include <numeric>
#include <vector>

#include <SoundTouch.h>

#include "audio_core/audio_core.h"
#include "audio_core/sink.h"
#include "audio_core/time_stretch.h"

#include "common/common_types.h"
#include "common/math_util.h"
#include "common/logging/log.h"

namespace TimeStretch {

static soundtouch::SoundTouch soundtouch;

using steady_clock = std::chrono::steady_clock;

steady_clock::time_point frame_timer = steady_clock::now();
double smooth_ratio = 1.0;
void Tick(unsigned samples_in_queue) {
    const steady_clock::time_point now = steady_clock::now();
    const std::chrono::duration<double> duration = now - frame_timer;
    frame_timer = now;

    constexpr double native_frame_time = (double)AudioCore::samples_per_frame / (double)AudioCore::native_sample_rate;
    const double actual_frame_time = duration.count();

    double ratio = actual_frame_time / native_frame_time;
    ratio = MathUtil::Clamp<double>(ratio, 0.01, 100.0);
    
    // TODO: Uhh was just reading this and this seems super wonky double-check your logic wtf are you thinking.
    if (samples_in_queue < 4096) {
        ratio = ratio > 1.0 ? ratio * ratio : 1.0;
        ratio = MathUtil::Clamp<double>(ratio, 0.01, 100.0);
    } else if (AudioCore::sink->SamplesInQueue() < 16000) {
        ratio = ratio > 1.0 ? sqrt(ratio) : 0.01;
        ratio = MathUtil::Clamp<double>(ratio, 0.01, 100.0);
    }

    smooth_ratio = 0.99 * smooth_ratio + 0.01 * ratio;
    smooth_ratio = MathUtil::Clamp<double>(smooth_ratio, 0.01, 100.0);

    soundtouch.setTempo(1.0 / smooth_ratio);
}

void Init() {
    soundtouch.setTempo(1.0);
    soundtouch.setChannels(2);
}

void Shutdown() {
    soundtouch.setTempo(1.0);
}

void AddSamples(const std::array<std::array<s16, 2>, AudioCore::samples_per_frame>& samples) {
    // FIXME: lol don't do this c-style cast
    soundtouch.putSamples((s16*)samples.data(), AudioCore::samples_per_frame);
}

void OutputSamples(std::function<void(const std::vector<s16>&)> fn) {
    size_t available = soundtouch.numSamples();

    std::vector<s16> output(available * 2);

    soundtouch.receiveSamples(output.data(), available);

    fn(output);
}

}
