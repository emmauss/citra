// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "audio_core/audio_core.h"
#include "audio_core/hle/common.h"
#include "audio_core/hle/dsp.h"

namespace DSP {
namespace HLE {

/// Initialise this DSP module
void SourceInit();

/**
 * Perform processing for this DSP module.
 * This module performs:
 * - Buffer management
 * - Decoding of buffers
 * - Buffer resampling and interpolation
 * - Per-source filtering (SimpleFilter, BiquadFilter)
 * - Per-source gain
 */
void SourceUpdate(int source_id, SourceConfiguration::Configuration& config, const s16_le adpcm_coeffs[16], SourceStatus::Status& status);

/// Output of this DSP module.
const QuadFrame32& SourceFrame(int source_id);

/// Mix current frame from source_id into buffer based on gain coefficients for intermediate_mix_id.
void SourceFrameMixInto(QuadFrame32& dest, int source_id, int intermediate_mix_id);

}
}
