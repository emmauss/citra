// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "video_core/pica.h"
#include "video_core/shader/shader.h"

namespace Pica {

namespace Shader {

template<bool Debug>
void RunInterpreter(const Pica::Regs::ShaderConfig& config, const ShaderSetup& setup, UnitState<Debug>& state);

} // namespace

} // namespace
