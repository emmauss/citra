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

static std::array<std::vector<u8>, static_cast<size_t>(DspPipe::DspPipe_MAX)> pipe_data;

void ResetPipes() {
    for (auto& data : pipe_data) {
        data.clear();
    }
    is_active = false;
}

std::vector<u8> PipeRead(DspPipe pipe_number, u32 length) {
    if (pipe_number >= DspPipe::DspPipe_MAX) {
        LOG_ERROR(Audio_DSP, "pipe_number = %u invalid", pipe_number);
        return {};
    }

    std::vector<u8>& data = pipe_data[static_cast<size_t>(pipe_number)];

    if (length > data.size()) {
        LOG_WARNING(Audio_DSP, "pipe_number = %u is out of data, application requested read of %u but %zu remain",
                    pipe_number, length, data.size());
        length = data.size();
    }

    if (length == 0)
        return {};

    std::vector<u8> ret(data.begin(), data.begin() + length);
    data.erase(data.begin(), data.begin() + length);
    return ret;
}

size_t GetPipeReadableSize(DspPipe pipe_number) {
    if (pipe_number >= DspPipe::DspPipe_MAX) {
        LOG_ERROR(Audio_DSP, "pipe_number = %u invalid", pipe_number);
        return 0;
    }

    return pipe_data[static_cast<size_t>(pipe_number)].size();
}

static void WriteU16(DspPipe pipe_number, u16 value) {
    std::vector<u8>& data = pipe_data[static_cast<size_t>(pipe_number)];
    // Little endian
    data.emplace_back(value & 0xFF);
    data.emplace_back(value >> 8);
}

static void AudioPipeWriteStructAddresses() {
    // These struct addresses are DSP dram addresses.
    // See also: DSP_DSP::ConvertProcessAddressFromDspDram
    static const std::array<u16, 15> struct_addresses = {
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

    // Begin with a u16 denoting the number of structs.
    WriteU16(DspPipe::Audio, struct_addresses.size());
    // Then write the struct addresses.
    for (u16 addr : struct_addresses) {
        WriteU16(DspPipe::Audio, addr);
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
            AudioPipeWriteStructAddresses();
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
