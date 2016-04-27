// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <chrono>
#include <cmath>
#include <vector>

#include <SoundTouch.h>

#include "audio_core/audio_core.h"
#include "audio_core/time_stretch.h"

#include "common/common_types.h"
#include "common/logging/log.h"
#include "common/math_util.h"

using steady_clock = std::chrono::steady_clock;

namespace AudioCore {

constexpr double MIN_RATIO = 0.1;
constexpr double MAX_RATIO = 100.0;

constexpr double MIN_DELAY_TIME = 0.1; // Units: seconds
constexpr double MAX_DELAY_TIME = 0.4; // Units: seconds
constexpr size_t DROP_FRAMES_SAMPLE_DELAY = 16000; // Units: samples

constexpr double SMOOTHING_FACTOR = 0.007;

struct TimeStretcher::Impl {
    soundtouch::SoundTouch soundtouch;

    steady_clock::time_point frame_timer = steady_clock::now();

    double smoothed_ratio = 1.0;

    size_t samples_queued = 0;

    double sample_rate = static_cast<double>(AudioCore::native_sample_rate);
};

std::vector<s16> TimeStretcher::Process(size_t samples_in_queue) {
    double ratio = CalculateCurrentRatio();

    // This is a very simple algorithm, but it works and is stable.

    ratio = CorrectForUnderAndOverflow(ratio, samples_in_queue);
    impl->smoothed_ratio = (1.0 - SMOOTHING_FACTOR) * impl->smoothed_ratio + SMOOTHING_FACTOR * ratio;
    impl->smoothed_ratio = ClampRatio(impl->smoothed_ratio);

    impl->soundtouch.setTempo(1.0 / impl->smoothed_ratio);

    std::vector<s16> samples = GetSamples();
    if (samples_in_queue >= DROP_FRAMES_SAMPLE_DELAY) {
        samples.clear();
        LOG_DEBUG(Audio, "Dropping frames!");
    }
    return samples;
}

TimeStretcher::TimeStretcher() : impl(new Impl) {
    impl->soundtouch.setTempo(1.0);
    impl->soundtouch.setPitch(1.0);
    impl->soundtouch.setRate(1.0);
    impl->soundtouch.setChannels(2);
    impl->soundtouch.setSampleRate(AudioCore::native_sample_rate);
}

TimeStretcher::~TimeStretcher() {
    impl->soundtouch.clear();
}

void TimeStretcher::SetOutputSampleRate(unsigned int sample_rate) {
    impl->sample_rate = static_cast<double>(sample_rate);
    impl->soundtouch.setRate(impl->sample_rate / static_cast<double>(AudioCore::native_sample_rate));
}

void TimeStretcher::AddSamples(const s16* buffer, size_t num_samples) {
    // FIXME: lol don't do this c-style cast
    impl->soundtouch.putSamples(buffer, static_cast<uint>(num_samples));
    impl->samples_queued += num_samples;
}

void TimeStretcher::NullSamples() {
    impl->soundtouch.flush();
}

double TimeStretcher::ClampRatio(double ratio) {
    return MathUtil::Clamp<double>(ratio, MIN_RATIO, MAX_RATIO);
}

double TimeStretcher::CalculateCurrentRatio() {
    const steady_clock::time_point now = steady_clock::now();
    const std::chrono::duration<double> duration = now - impl->frame_timer;

    const double expected_time = static_cast<double>(impl->samples_queued) / static_cast<double>(AudioCore::native_sample_rate);
    const double actual_time = duration.count();

    double ratio;
    if (expected_time != 0) {
        ratio = ClampRatio(actual_time / expected_time);
    } else {
        ratio = 1.0;
    }

    impl->frame_timer = now;
    impl->samples_queued = 0;

    return ratio;
}

double TimeStretcher::CorrectForUnderAndOverflow(double ratio, size_t sample_delay) const {
    const size_t min_sample_delay = static_cast<size_t>(MIN_DELAY_TIME * impl->sample_rate);
    const size_t max_sample_delay = static_cast<size_t>(MAX_DELAY_TIME * impl->sample_rate);

    const size_t ideal_sample_delay = (min_sample_delay + max_sample_delay) / 2;
    const double delay_window = static_cast<double>((max_sample_delay - min_sample_delay) / 2);

    if (sample_delay < min_sample_delay) {
        ratio = ratio > 1.0 ? ratio * ratio : sqrt(ratio);
    } else if (sample_delay > max_sample_delay) {
        ratio = ratio > 1.0 ? sqrt(ratio) : ratio * ratio;
    } else if (sample_delay > ideal_sample_delay) {
        double delta = static_cast<double>(sample_delay - ideal_sample_delay);
        ratio *= 1.0 - (0.1 + delta / delay_window);
    } else if (sample_delay < ideal_sample_delay) {
        double delta = static_cast<double>(ideal_sample_delay - sample_delay);
        ratio *= 1.0 + (0.1 + delta / delay_window);
    }

    return ClampRatio(ratio);
}

std::vector<short> TimeStretcher::GetSamples() {
    uint available = impl->soundtouch.numSamples();

    std::vector<s16> output(static_cast<size_t>(available) * 2);

    impl->soundtouch.receiveSamples(output.data(), available);

    return output;
}

}
