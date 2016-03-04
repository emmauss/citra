// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include <vector>

#include "audio_core/hle/dsp.h"
#include "audio_core/hle/pipe.h"

#include "common/assert.h"
#include "common/common_types.h"
#include "common/logging/log.h"

namespace DSP {
namespace HLE {

static bool is_active = false;

static size_t audio_pipe_position = 0;

// DSP addresses address 16-bit words, hence the division by two.
// These DSP addresses are converted by the application into ARM11 virtual addresses using
// the dsp::DSP ConvertProcessAddressFromDspDram service call.

constexpr size_t num_structs = 15;
static const std::array<u16_le, num_structs + 1> audio_pipe_payload = {
    num_structs,
    0x8000 + offsetof(SharedMemory, frame_counter) / 2,
    0x8000 + offsetof(SharedMemory, source_configurations) / 2,
    0x8000 + offsetof(SharedMemory, source_statuses) / 2,
    0x8000 + offsetof(SharedMemory, adpcm_coefficients) / 2,
    0x8000 + offsetof(SharedMemory, dsp_configuration) / 2,
    0x8000 + offsetof(SharedMemory, dsp_status) / 2,
    0x8000 + offsetof(SharedMemory, final_samples) / 2,
    0x8000 + offsetof(SharedMemory, intermediate_mix_samples) / 2,
    0x8000 + offsetof(SharedMemory, compressor) / 2,
    0x8000 + offsetof(SharedMemory, dsp_debug) / 2,
    0x8000 + offsetof(SharedMemory, unknown10) / 2,
    0x8000 + offsetof(SharedMemory, unknown11) / 2,
    0x8000 + offsetof(SharedMemory, unknown12) / 2,
    0x8000 + offsetof(SharedMemory, unknown13) / 2,
    0x8000 + offsetof(SharedMemory, unknown14) / 2
};

void ResetPipes() {
    audio_pipe_position = 0;
    is_active = false;
}

std::vector<u8> PipeRead(DspPipe pipe_number, u32 length) {
    switch (pipe_number) {
    case DspPipe::Audio: {
        const static size_t max_end = sizeof(audio_pipe_payload); ///< Length of the entire response in bytes
        const static u8* entire_response_bytes = reinterpret_cast<const u8*>(audio_pipe_payload.data());

        // Start and end positions of this read request.
        size_t start = audio_pipe_position;
        size_t end = audio_pipe_position + length;
        if (end > max_end) end = max_end;

        audio_pipe_position = end;
        ASSERT(audio_pipe_position <= max_end);

        return std::vector<u8>(entire_response_bytes + start, entire_response_bytes + end);
    }
    default:
        LOG_CRITICAL(Audio_DSP, "pipe_number = %u unimplemented", pipe_number);
        UNIMPLEMENTED();
        return {};
    }
}

size_t GetPipeReadableSize(DspPipe pipe_number) {
    switch (pipe_number) {
    case DspPipe::Audio:
        return sizeof(audio_pipe_payload) - audio_pipe_position;
    default:
        LOG_CRITICAL(Audio_DSP, "pipe_number = %u unimplemented", pipe_number);
        UNIMPLEMENTED();
        return 0;
    }
}

void PipeWrite(DspPipe pipe_number, const std::vector<u8>& buffer) {
    switch (pipe_number) {
    case DspPipe::Audio: {
        if (buffer.size() != 4) {
            LOG_ERROR(Audio_DSP, "DspPipe::Audio: Unexpected buffer length %zu was written", buffer.size());
            return;
        }

        enum class StateChange {
            Initalize = 0,
            Shutdown = 1,
            // The below two state changes have only been observed when going into / coming out of sleep.
            Wakeup = 2,
            Sleep = 3
        };

        switch (static_cast<StateChange>(buffer[0])) {
        case StateChange::Initalize:
            LOG_INFO(Audio_DSP, "Application has requested initialization of DSP hardware");
            ResetPipes();
            is_active = true;
            break;
        case StateChange::Shutdown:
            LOG_INFO(Audio_DSP, "Application has requested shutdown of DSP hardware");
            is_active = false;
            break;
        case StateChange::Wakeup:
            LOG_CRITICAL(Audio_DSP, "(Unimplemented) Wakeup state transition");
            UNIMPLEMENTED();
            break;
        case StateChange::Sleep:
            LOG_CRITICAL(Audio_DSP, "(Unimplemented) Sleep state transition");
            UNIMPLEMENTED();
            break;
        default:
            LOG_ERROR(Audio_DSP, "Application has requested unknown state transition of DSP hardware %hhu", buffer[0]);
            is_active = false;
            break;
        }

        return;
    }
    default:
        LOG_CRITICAL(Audio_DSP, "pipe_number = %u unimplemented", pipe_number);
        UNIMPLEMENTED();
        return;
    }
}

bool IsActivated() {
    return is_active;
}

} // namespace HLE
} // namespace DSP
