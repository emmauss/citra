// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <queue>
#include <vector>

#include "audio_core/codec.h"
#include "audio_core/hle/common.h"
#include "audio_core/hle/source.h"

#include "common/assert.h"
#include "common/logging/log.h"

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
};

struct State {
    bool enabled;
    float rate_multiplier;
    u16 sync;
    std::array<std::array<float, 4>, 3> gains;
    MonoOrStereo mono_or_stereo;
    Format format;

    std::array<s16, 16> adpcm_coeffs;
    Codec::AdpcmState adpcm_state;

    std::priority_queue<Buffer> queue;
    std::vector<s16> current_buffer;
    std::array<Frame32, 4> current_frames;
};

static void ParseConfig(State& s, SourceConfiguration::Configuration& config, const s16_le adpcm_coeffs[16]) {
    if (!config.dirty_raw) {
        return;
    }

    if (config.reset_flag) {
        s = {};
    }

    if (config.enable_dirty) {
        s.enabled = config.enable != 0;
    }

    if (config.sync_dirty) {
        s.sync = config.sync;
    }

    if (config.rate_multiplier_dirty) {
        s.rate_multiplier = config.rate_multiplier;
    }

    if (config.adpcm_coefficients_dirty) {
        std::copy(adpcm_coeffs, adpcm_coeffs + 16, s.adpcm_coeffs.begin());
    }

    if (config.gain_0_dirty) {
        for (int i = 0; i < 4; i++) {
            s.gains[0][i] = config.gain[0][i];
        }
    }

    if (config.gain_1_dirty) {
        for (int i = 0; i < 4; i++) {
            s.gains[1][i] = config.gain[1][i];
        }
    }

    if (config.gain_2_dirty) {
        for (int i = 0; i < 4; i++) {
            s.gains[2][i] = config.gain[2][i];
        }
    }

    if (config.buffer_queue_dirty) {
        for (int i = 0; i < 4; i++) {
            if (config.buffer_queue_dirty & (1 << i)) {
                s.queue.emplace(config.buffers[i]);
            }
        }
    }

    if (config.unknown_flag) {
        LOG_WARNING(Audio_DSP, "(STUB) unknown_flag is set!!!");
    }

    if (config.format_dirty || config.embedded_buffer_dirty) {
        s.format = config.format;
    }

    if (config.mono_or_stereo_dirty || config.embedded_buffer_dirty) {
        s.mono_or_stereo = config.mono_or_stereo;
    }

    if (config.embedded_buffer_dirty) {
        ASSERT(s.queue.empty());

        s.queue.emplace(Buffer {
            config.physical_address,
            config.length,
            (u8)config.adpcm_ps,
            { config.adpcm_yn[0], config.adpcm_yn[1] },
            config.adpcm_dirty.ToBool(),
            config.is_looping.ToBool(),
            config.buffer_id
        });
    }

    config.dirty_raw = 0;
}

static void AdvanceFrame(State& s) {

}

static void UpdateStatus(State& s, SourceStatus::Status& status) {

}

static std::array<State, AudioCore::num_sources> state;

void SourceInit() {
    state = {};
}

void SourceUpdate(int source_id, SourceConfiguration::Configuration& config, const s16_le adpcm_coeffs[16], SourceStatus::Status& status) {
    ParseConfig(state[source_id], config, adpcm_coeffs);
    AdvanceFrame(state[source_id]);
    UpdateStatus(state[source_id], status);
}

const Frame32& SourceFrame(int source_id, int channel_id) {
    return state[source_id].current_frames[channel_id];
}

}
}
