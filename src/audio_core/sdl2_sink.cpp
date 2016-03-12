// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <SDL.h>

#include "audio_core/audio_core.h"
#include "audio_core/sdl2_sink.h"

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
    desired_audiospec.freq = AudioCore::native_sample_rate; // TODO: Maybe go for a smaller value
    desired_audiospec.samples = 2048;
    desired_audiospec.callback = nullptr; // We're going to use SDL_QueueAudio

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
    SDL_QueueAudio(audio_device_id, samples.data(), samples.size() * sizeof(s16));
}

/// Samples enqueued that have not been played yet.
std::size_t SDL2Sink::SamplesInQueue() const {
    return SDL_GetQueuedAudioSize(audio_device_id) / sizeof(s16);
}

}
