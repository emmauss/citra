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
#include "common/logging/log.h"

namespace TimeStretch {

static soundtouch::SoundTouch* sound_touch;

double time_per_sample;
/// The amount of delay that this algorithm aims for. (Units: seconds)
constexpr double ideal_audio_delay = 0.5;
double dynamic_delay = 0.2;

/// Short-time FFT (STFT)
template<s32 sign, typename To, typename Ti, size_t size>
std::array<To, size> stft(const std::array<Ti, size>& input) {
    std::array<To, size> output;

    return output;
}

/// Sliding-window filter that drops extreme values.
struct {
private:
    std::array<double, 51> buffer;
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
        return std::accumulate(sorted.begin() + 2, sorted.end() - 2, 0.0) / (buffer.size() - 4);
    }
} averager;

struct {
private:
    std::array<double, 51> buffer;
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
        return std::accumulate(sorted.begin() + 2, sorted.end() - 2, 0.0) / (buffer.size() - 4);
    }
} averager2;

static int written_samples = 0;
static std::chrono::time_point<std::chrono::steady_clock> time = std::chrono::steady_clock::now();
static std::chrono::time_point<std::chrono::steady_clock> last_set_tempo_time = std::chrono::steady_clock::now();
static double audio_delay = ideal_audio_delay;
static double last_tempo = 1.0;
static double integral = ideal_audio_delay;
static double smooth = 1.0;
static unsigned frame_counter = 0;

static double force_tempo = 1.0;

static double current_tempo = 1.0;

void Tick(unsigned samples_in_queue) {
    auto endtime = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = endtime - time;
    written_samples -= duration.count() / time_per_sample + 10;
    if (written_samples < 0) written_samples = 0;
    time = endtime;

    // A traditional PID is too unresponsive.

    double current_delay = (written_samples + samples_in_queue) * time_per_sample;
    averager.AddSample(current_delay);
    current_delay = averager.GetAverage();
    double weight = duration.count() / 0.5;
    if (weight > 1.0) weight = 1.0;
    audio_delay += weight * (current_delay - audio_delay);

    if (audio_delay < 0.01 && integral < 2.0) {
        integral = 2.0;
    }

    double tempo = audio_delay / integral;

    integral += 0.001 * (ideal_audio_delay/tempo - integral);
    if (integral < ideal_audio_delay) integral = ideal_audio_delay;
    if (integral > ideal_audio_delay/0.01) integral = ideal_audio_delay / 0.01;

    if (endtime - last_set_tempo_time > std::chrono::duration<double>(1.0)) {
        LOG_INFO(Audio, "Emulation is at %.1f%% speed\n", 100.0 * ((double)AudioCore::samples_per_frame / (double)AudioCore::native_sample_rate) / (5.0 / frame_counter), frame_counter / 5.0);

        force_tempo = ((double)AudioCore::samples_per_frame / (double)AudioCore::native_sample_rate) / (5.0 / frame_counter);

        last_set_tempo_time = endtime;
        frame_counter = 0;
    }
    frame_counter++;

    smooth += (weight * 0.5) * (tempo - smooth);

    tempo = force_tempo;
    if (tempo < 0.1) tempo = 0.1;
    if (tempo > 10.) tempo = 10.;

    tempo *= 0.9;
    tempo += 0.1 * smooth;

    sound_touch->setTempo(tempo);
    current_tempo = tempo;
}

void Init() {
    time_per_sample = 1.0 / (double)AudioCore::sink->GetNativeSampleRate();
    averager.Reset();

    sound_touch = new soundtouch::SoundTouch();

    sound_touch->setSampleRate(AudioCore::native_sample_rate);
    sound_touch->setChannels(2); // Stereo

    sound_touch->setSetting(SETTING_USE_AA_FILTER, 0);
    sound_touch->setSetting(SETTING_USE_QUICKSEEK, 0);

    // These numbers are tweakable.
    sound_touch->setSetting(SETTING_SEQUENCE_MS, 100);
    sound_touch->setSetting(SETTING_SEEKWINDOW_MS, 50);
    sound_touch->setSetting(SETTING_OVERLAP_MS, 20);

    sound_touch->setTempo(1.0);
    current_tempo = 1.0;

    written_samples = 1;
}

void Shutdown() {
    sound_touch->clear();
}

void AddSamples(const std::array<std::array<s16, AudioCore::samples_per_frame>, 2>& samples) {
    if ((written_samples * time_per_sample) > 1.5) return;
    written_samples += AudioCore::samples_per_frame / current_tempo;
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
        output_samples.resize(4096);
        num = sound_touch->receiveSamples(output_samples.data(), output_samples.size()/2);
        if (num == 0) {
            break;
        }
        output_samples.resize(num*2);
        fn(output_samples);
    }
}

}
