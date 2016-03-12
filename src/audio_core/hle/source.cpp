// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <vector>

#include "audio_core/codec.h"
#include "audio_core/hle/common.h"
#include "audio_core/hle/source.h"

#include "common/assert.h"
#include "common/logging/log.h"

#include "core/memory.h"

namespace DSP {
namespace HLE {

using MonoOrStereo = SourceConfiguration::Configuration::MonoOrStereo;
using Format = SourceConfiguration::Configuration::Format;
using DspBuffer = SourceConfiguration::Configuration::Buffer;

struct Buffer {
    PAddr physical_address;
    u32 length;
    u8 adpcm_ps;
    u16 adpcm_yn[2];
    bool adpcm_dirty;
    bool is_looping;
    u16 buffer_id;

    bool from_queue;

    bool operator < (const Buffer& other) const {
        // We want things with lower id to appear first, unless we have wraparound.
        // priority_queue puts a before b when b < a.
        if (this->buffer_id < 10 && other.buffer_id > 65520) return true;
        if (other.buffer_id < 10 && this->buffer_id > 65520) return false;
        return this->buffer_id > other.buffer_id;
    }
};

struct State {
    size_t source_id;

    bool enabled;
    float rate_multiplier;
    u16 sync;
    std::array<std::array<float, 4>, 3> gains;
    MonoOrStereo mono_or_stereo;
    Format format;

    std::array<s16, 16> adpcm_coeffs;
    Codec::AdpcmState adpcm_state;

    bool do_not_trigger_update = true;
    bool buffer_update = false;
    u32 current_buffer_id;
    u32 previous_buffer_id;

    std::priority_queue<Buffer> queue;
    u32 current_sample_number;
    u32 next_sample_number;
    Codec::StereoBuffer16 current_buffer;
    QuadFrame32 current_frame;
};

static void ParseConfig(State& s, SourceConfiguration::Configuration& config, const s16_le adpcm_coeffs[16]) {
    if (!config.dirty_raw) {
        return;
    }

    if (config.reset_flag) {
        size_t id = s.source_id;
        s = {};
        s.source_id = id;
        LOG_DEBUG(Audio_DSP, "source_id=%zu reset", s.source_id);
    }

    if (config.enable_dirty) {
        s.enabled = config.enable != 0;
        LOG_DEBUG(Audio_DSP, "source_id=%zu enable=%d", s.source_id, s.enabled);
    }

    if (config.sync_dirty) {
        s.sync = config.sync;
        LOG_DEBUG(Audio_DSP, "source_id=%zu sync=%u", s.source_id, s.sync);
    }

    if (config.rate_multiplier_dirty) {
        s.rate_multiplier = config.rate_multiplier;
        LOG_DEBUG(Audio_DSP, "source_id=%zu rate=%f", s.source_id, s.rate_multiplier);
    }

    if (config.adpcm_coefficients_dirty) {
        std::copy(adpcm_coeffs, adpcm_coeffs + 16, s.adpcm_coeffs.begin());
        LOG_TRACE(Audio_DSP, "source_id=%zu adpcm update", s.source_id);
    }

    if (config.gain_0_dirty) {
        for (int i = 0; i < 4; i++) {
            s.gains[0][i] = config.gain[0][i];
            LOG_TRACE(Audio_DSP, "source_id=%zu gains[0][%i] = %f", s.source_id, i, s.gains[0][i]);
        }
    }

    if (config.gain_1_dirty) {
        for (int i = 0; i < 4; i++) {
            s.gains[1][i] = config.gain[1][i];
            LOG_TRACE(Audio_DSP, "source_id=%zu gains[1][%i] = %f", s.source_id, i, s.gains[1][i]);
        }
    }

    if (config.gain_2_dirty) {
        for (int i = 0; i < 4; i++) {
            s.gains[2][i] = config.gain[2][i];
            LOG_TRACE(Audio_DSP, "source_id=%zu gains[2][%i] = %f", s.source_id, i, s.gains[2][i]);
        }
    }

    if (config.buffer_queue_dirty) {
        for (int i = 0; i < 4; i++) {
            if (config.buffers_dirty & (1 << i)) {
                const auto& b = config.buffers[i];
                s.queue.emplace(Buffer {
                    b.physical_address,
                    b.length,
                    (u8)b.adpcm_ps,
                    { b.adpcm_yn[0], b.adpcm_yn[1] },
                    b.adpcm_dirty != 0,
                    b.is_looping != 0,
                    b.buffer_id,
                    true
                });
            }
        }
        config.buffers_dirty = 0;
    }

    if (config.unknown_flag) {
        LOG_WARNING(Audio_DSP, "(STUB) unknown_flag is set!!!");
    }

    if (config.format_dirty || config.embedded_buffer_dirty) {
        s.format = config.format;
        LOG_TRACE(Audio_DSP, "source_id=%zu format=%u", s.source_id, s.format);
    }

    if (config.mono_or_stereo_dirty || config.embedded_buffer_dirty) {
        s.mono_or_stereo = config.mono_or_stereo;
        LOG_TRACE(Audio_DSP, "source_id=%zu mono_or_stereo=%u", s.source_id, s.mono_or_stereo);
    }

    if (config.embedded_buffer_dirty) {
        s.queue.emplace(Buffer {
            config.physical_address,
            config.length,
            (u8)config.adpcm_ps,
            { config.adpcm_yn[0], config.adpcm_yn[1] },
            config.adpcm_dirty.ToBool(),
            config.is_looping.ToBool(),
            config.buffer_id,
            false
        });
    }

    config.dirty_raw = 0;
}

static void DequeueBuffer(State& s) {
    ASSERT(!s.queue.empty());
    ASSERT(s.current_buffer[0].size() == s.current_buffer[1].size());
    ASSERT(s.current_buffer[0].empty());

    const Buffer buf = s.queue.top();
    s.queue.pop();

    const u8* const memory = Memory::GetPhysicalPointer(buf.physical_address);
    ASSERT(memory);
    const unsigned num_channels = s.mono_or_stereo == MonoOrStereo::Mono ? 1 : 2;

    if (buf.adpcm_dirty) {
        s.adpcm_state.yn1 = buf.adpcm_yn[0];
        s.adpcm_state.yn2 = buf.adpcm_yn[1];
    }

    if (buf.is_looping) {
        LOG_ERROR(Audio_DSP, "Looped buffers are unimplemented at the moment");
    }

    switch (s.format) {
    case Format::PCM8:
        s.current_buffer = Codec::DecodePCM8(num_channels, memory, buf.length);
        break;
    case Format::PCM16:
        s.current_buffer = Codec::DecodePCM16(num_channels, memory, buf.length);
        break;
    case Format::ADPCM:
        s.current_buffer = Codec::DecodeADPCM(memory, buf.length, s.adpcm_coeffs, s.adpcm_state);
        break;
    default:
        UNIMPLEMENTED();
        break;
    }

    s.current_sample_number = s.next_sample_number = 0;
    s.current_buffer_id = buf.buffer_id;
    s.buffer_update = buf.from_queue;

    LOG_TRACE(Audio_DSP, "source_id=%u buffer_id=%u from_queue=%d", s.source_id, buf.buffer_id, buf.from_queue);
}

static void ResampleBuffer(State& s) {
    ASSERT(s.current_buffer[0].size() == s.current_buffer[1].size());
    const size_t samples_to_consume = std::min<size_t>(AudioCore::samples_per_frame * s.rate_multiplier,
                                                       s.current_buffer[0].size());

    // TODO: Resample.
    // TODO: Put resampled buffer into s.current_frame.

    for (size_t i = 0; i < AudioCore::samples_per_frame; i++) {
        size_t bufferi = i * s.rate_multiplier;
        if (bufferi >= samples_to_consume) bufferi = samples_to_consume - 1;
        s.current_frame[0][i] = s.current_buffer[0][bufferi];
        s.current_frame[1][i] = s.current_buffer[1][bufferi];
    }

    s.current_buffer[0].erase(s.current_buffer[0].begin(), s.current_buffer[0].begin() + samples_to_consume);
    s.current_buffer[1].erase(s.current_buffer[1].begin(), s.current_buffer[1].begin() + samples_to_consume);

    s.current_sample_number = s.next_sample_number;
    s.next_sample_number += samples_to_consume;
}

static void AdvanceFrame(State& s) {
    if (s.current_buffer[0].empty()) {
        if (s.queue.empty()) {
            return;
        }

        DequeueBuffer(s);
    }

    ResampleBuffer(s);

    // TODO: Filters
}

static void UpdateStatus(State& s, SourceStatus::Status& status) {
    // Applications depend on the correct emulation of
    // previous_buffer_id_dirty and previous_buffer_id to synchronise
    // audio with video.
    status.is_enabled = s.enabled;
    status.current_buffer_id_dirty = s.buffer_update ? 1 : 0;
    s.buffer_update = false;
    status.current_buffer_id = s.current_buffer_id;
    status.buffer_position = s.current_sample_number;
    status.sync = s.sync;
}

static std::array<State, AudioCore::num_sources> state;

void SourceInit() {
    state = {};
    for (size_t i = 0; i < state.size(); i++)
        state[i].source_id = i;
}

void SourceUpdate(int source_id, SourceConfiguration::Configuration& config, const s16_le adpcm_coeffs[16], SourceStatus::Status& status) {
    ASSERT(source_id >= 0 && source_id < AudioCore::num_sources);
    ParseConfig(state[source_id], config, adpcm_coeffs);
    AdvanceFrame(state[source_id]);
    UpdateStatus(state[source_id], status);
}

const QuadFrame32& SourceFrame(int source_id) {
    ASSERT(source_id >= 0 && source_id < AudioCore::num_sources);
    return state[source_id].current_frame;
}

void SourceFrameMixInto(QuadFrame32& dest, int source_id, int intermediate_mix_id) {
    ASSERT(source_id >= 0 && source_id < AudioCore::num_sources);
    ASSERT(intermediate_mix_id >= 0 && intermediate_mix_id < 3);

    const State& s = state[source_id];

    for (int i = 0; i < dest[0].size(); i++) {
        for (int channel = 0; channel < 4; channel++) {
            dest[channel][i] += s.gains[intermediate_mix_id][channel] * s.current_frame[channel][i];
        }
    }
}

}
}
