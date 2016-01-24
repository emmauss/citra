#pragma once

#include "AL/al.h"
#include "AL/alc.h"
#include "AL/alext.h"

#include "common/common_types.h"

#include <tuple>

namespace Audio {
    void Init();
    void Play(void* buf, size_t size);
    void Shutdown();

    enum Format : u16 {
        FORMAT_PCM8 = 0,
        FORMAT_PCM16 = 1,
        FORMAT_ADPCM = 2
    };

    void UpdateFormat(int chanid, int mono_or_stereo, Format format);

    void EnqueueBuffer(int chanid, u16 buffer_id,
        void* data, int sample_count,
        bool has_adpcm, u16 adpcm_ps, s16 adpcm_yn[2],
        bool is_looping);

    void Tick(int chanid);

    std::tuple<bool, u16, u32> GetStatus(int chanid);
};