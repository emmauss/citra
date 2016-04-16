// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <SDL.h>

#include "audio_core/audio_core.h"
#include "audio_core/sdl2_sink.h"

#include "common/assert.h"
#include "common/logging/log.h"

namespace AudioCore {

std::unique_ptr<Sink> sink(new SDL2Sink());

SDL2Sink::SDL2Sink() {
    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        LOG_CRITICAL(Audio_SDL2, "SDL_Init(SDL_INIT_AUDIO) failed");
        exit(-2);
    }

    SDL_AudioSpec desired_audiospec;
    SDL_zero(desired_audiospec);
    desired_audiospec.format = AUDIO_S16;
    desired_audiospec.channels = 2;
    desired_audiospec.freq = AudioCore::native_sample_rate;
    desired_audiospec.samples = 4096;
    desired_audiospec.userdata = this;
    desired_audiospec.callback = &SDL2Sink::Callback; // We're going to use SDL_QueueAudio

    SDL_AudioSpec obtained_audiospec;
    SDL_zero(obtained_audiospec);

    audio_device_id = SDL_OpenAudioDevice(nullptr, /*iscapture=*/false, &desired_audiospec, &obtained_audiospec, 0);
    if (audio_device_id < 0) {
        LOG_CRITICAL(Audio_SDL2, "SDL_OpenAudioDevice failed");
        exit(-2);
    }

    sample_rate = obtained_audiospec.freq;

    SDL_PauseAudioDevice(audio_device_id, 0);
}

SDL2Sink::~SDL2Sink() {
}

/// The native rate of this sink. The sink expects to be fed samples that respect this. (Units: samples/sec)
unsigned SDL2Sink::GetNativeSampleRate() const {
    return sample_rate;
}

/**
* Feed stereo samples to sink.
* @param samples Samples in interleaved stereo PCM16 format. Size of vector must be multiple of two.
*/
void SDL2Sink::EnqueueSamples(const std::vector<s16>& samples) {
    ASSERT(samples.size() % 2 == 0);
    SDL_LockAudioDevice(audio_device_id);
    queue.emplace_back(samples);
    SDL_UnlockAudioDevice(audio_device_id);
}

/// Samples enqueued that have not been played yet.
size_t SDL2Sink::SamplesInQueue() const {
    const size_t queue_size = RealQueueSize() + dequeue_consumed;

    if (dequeue_consumed == 0)
        return queue_size;

    const std::chrono::duration<double> duration = std::chrono::steady_clock::now() - dequeue_time;
    const size_t estimated_samples_consumed = sample_rate * duration.count();

    if (estimated_samples_consumed > queue_size)
        return 0;

    return queue_size - estimated_samples_consumed;
}

size_t SDL2Sink::RealQueueSize() const {
    size_t total_size = 0;
    SDL_LockAudioDevice(audio_device_id);
    for (const auto& buf : queue) {
        total_size += buf.size() / 2;
    }
    SDL_UnlockAudioDevice(audio_device_id);
    return total_size;
}

void SDL2Sink::Callback(void* sink_, u8* buffer, int buffer_size) {
    SDL2Sink* sink = reinterpret_cast<SDL2Sink*>(sink_);
    buffer_size /= sizeof(s16); // Convert to number of half-samples.

    sink->dequeue_time = std::chrono::steady_clock::now();
    sink->dequeue_consumed = buffer_size / 2;

    while (buffer_size > 0 && !sink->queue.empty()) {
        if (sink->queue.front().size() <= buffer_size) {
            memcpy(buffer, sink->queue.front().data(), sink->queue.front().size() * sizeof(s16));
            buffer += sink->queue.front().size() * sizeof(s16);
            buffer_size -= sink->queue.front().size();
            sink->queue.pop_front();
        } else {
            memcpy(buffer, sink->queue.front().data(), buffer_size * sizeof(s16));
            buffer += buffer_size * sizeof(s16);
            sink->queue.front().erase(sink->queue.front().begin(), sink->queue.front().begin() + buffer_size);
            buffer_size = 0;
        }
    }

    if (buffer_size > 0) {
        sink->dequeue_consumed -= buffer_size / 2;
        memset(buffer, 0, buffer_size * sizeof(s16));
    }
}

}
