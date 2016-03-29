// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <chrono>
#include <functional>
#include <list>
#include <numeric>
#include <vector>

#include <RubberBandStretcher.h>

#include "audio_core/audio_core.h"
#include "audio_core/sink.h"
#include "audio_core/time_stretch.h"

#include "common/common_types.h"
#include "common/math_util.h"
#include "common/logging/log.h"

namespace TimeStretch {

const RubberBand::RubberBandStretcher::Options options =
    RubberBand::RubberBandStretcher::OptionProcessRealTime |  // Must be used since we stretch in real-time
    RubberBand::RubberBandStretcher::OptionStretchPrecise |   // Must be used since we stretch in real-time
    RubberBand::RubberBandStretcher::OptionTransientsSmooth | // (Can be adjusted later.) Muddy sounding but eh.
    RubberBand::RubberBandStretcher::OptionDetectorCompound | // (Can be adjusted later.) Select the transients detector
    RubberBand::RubberBandStretcher::OptionPhaseLaminar |     // (Can be adjusted later.) I have no idea what this does
    RubberBand::RubberBandStretcher::OptionThreadingAuto |    // Use a processing thread where possible
    RubberBand::RubberBandStretcher::OptionWindowStandard |   // Tradeoff between latency and smoothness
    RubberBand::RubberBandStretcher::OptionSmoothingOff |     // Don't allow smoothing in the t-domain
    RubberBand::RubberBandStretcher::OptionFormantShifted |   // We don't do pitch-shifting
    RubberBand::RubberBandStretcher::OptionPitchHighSpeed |   // We don't do pitch-shifting
    RubberBand::RubberBandStretcher::OptionChannelsApart;     // This increases stereo spacing but results in clearer audio

static RubberBand::RubberBandStretcher stretcher(AudioCore::native_sample_rate, /* number of channels = */ 2, options);

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
    if (samples_in_queue < 4096) {
        ratio = ratio > 1.0 ? ratio * ratio : 1.0;
        ratio = MathUtil::Clamp<double>(ratio, 0.01, 100.0);
        printf("underflow\n");
    }

    smooth_ratio = 0.99 * smooth_ratio + 0.01 * ratio;
    smooth_ratio = MathUtil::Clamp<double>(smooth_ratio, 0.01, 100.0);

    stretcher.setTimeRatio(smooth_ratio);
}

void Init() {
    stretcher.reset();
    stretcher.setMaxProcessSize(AudioCore::samples_per_frame);
    stretcher.setPitchScale(1.0);
    stretcher.setTimeRatio(1.0);
}

void Shutdown() {
    stretcher.reset();
}

void AddSamples(const std::array<std::array<s16, AudioCore::samples_per_frame>, 2>& samples) {
    std::array<float, AudioCore::samples_per_frame> left;
    std::array<float, AudioCore::samples_per_frame> right;

    std::transform(samples[0].begin(), samples[0].end(), left.begin(), [](s16 sample) { return (float)sample / 32768.f; });
    std::transform(samples[1].begin(), samples[1].end(), right.begin(), [](s16 sample) { return (float)sample / 32768.f; });

    float* fsamples[2] = { left.data(), right.data() };
    stretcher.process(fsamples, AudioCore::samples_per_frame, /*final=*/false);
}

void OutputSamples(std::function<void(const std::vector<s16>&)> fn) {
    size_t num_samples = stretcher.available();
    std::vector<float> left(num_samples);
    std::vector<float> right(num_samples);

    float* fsamples[2] = { left.data(), right.data() };
    stretcher.retrieve(fsamples, num_samples);

    std::vector<s16> output(num_samples * 2);
    for (int i = 0; i < num_samples; i++) {
        output[i * 2 + 0] = MathUtil::Clamp<s16>(left[i] * 32768, -32768, 32767);
        output[i * 2 + 1] = MathUtil::Clamp<s16>(right[i] * 32768, -32768, 32767);
    }

    fn(output);
}

}
