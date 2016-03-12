// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <chrono>
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

namespace TimeStretch {

static soundtouch::SoundTouch* sound_touch;

double time_per_sample;
/// The amount of delay that this algorithm aims for. (Units: seconds)
constexpr double ideal_audio_delay = 0.2;
double dynamic_delay = 0.2;

/// Sliding-window filter that drops extreme values.
struct {
private:
    std::array<double, 11> buffer;
    int ptr = 0;
public:
    void Reset() {
        buffer.fill(ideal_audio_delay);
        ptr = 0;
    }
    void AddSample(double time) {
        buffer[ptr] = time;
        ptr = (ptr + 1) % buffer.size();
    }
    double GetAverage() {
        std::vector<double> sorted(buffer.begin(), buffer.end());
        std::sort(sorted.begin(), sorted.end());
        return sorted[buffer.size() / 2];
    }
} averager;

static int written_samples = 0;
static std::chrono::time_point<std::chrono::steady_clock> time = std::chrono::steady_clock::now();
double audio_delay = ideal_audio_delay;

void Tick(unsigned samples_in_queue) {
    auto endtime = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = endtime - time;
    written_samples -= duration.count() / time_per_sample + 1;
    if (written_samples < 0) written_samples = 0;
    time = endtime;

    double current_delay = (written_samples + samples_in_queue) * time_per_sample;
    audio_delay += duration.count() * (current_delay - audio_delay) / 0.2;

    double tempo = audio_delay / ideal_audio_delay;
    tempo = MathUtil::Clamp(tempo, 0.01, 20.0);

    sound_touch->setTempo(tempo);
}

void Init() {
    time_per_sample = 1.0 / (double)AudioCore::sink->GetNativeSampleRate();
    averager.Reset();

    sound_touch = new soundtouch::SoundTouch();

    sound_touch->setSampleRate(AudioCore::native_sample_rate);
    sound_touch->setChannels(2); // Stereo

    sound_touch->setSetting(SETTING_USE_AA_FILTER, 1);
    sound_touch->setSetting(SETTING_USE_QUICKSEEK, 0);

    // These numbers are tweakable.
    sound_touch->setSetting(SETTING_SEQUENCE_MS, 100);
    sound_touch->setSetting(SETTING_SEEKWINDOW_MS, 50);
    sound_touch->setSetting(SETTING_OVERLAP_MS, 20);

    sound_touch->setTempo(1.0);

    written_samples = 1;
}

void Shutdown() {
    sound_touch->clear();
}

void AddSamples(const std::array<std::array<s16, AudioCore::samples_per_frame>, 2>& samples) {
    std::array<s16, AudioCore::samples_per_frame * 2> input_samples;
    for (int i = 0; i < AudioCore::samples_per_frame; i++) {
        input_samples[i * 2 + 0] = samples[0][i];
        input_samples[i * 2 + 1] = samples[1][i];
    }
    sound_touch->putSamples(input_samples.data(), AudioCore::samples_per_frame);
}

void OutputSamples(std::function<void(const std::vector<s16>&)> fn) {
    std::vector<s16> output_samples;
    int num;

    while (true) {
        output_samples.resize(1024);
        num = sound_touch->receiveSamples(output_samples.data(), output_samples.size()/2);
        written_samples += num;
        if (num == 0) {
            break;
        }
        output_samples.resize(num*2);
        fn(output_samples);
    }
}

}
