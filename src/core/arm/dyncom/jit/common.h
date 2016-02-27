// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "common/common_types.h"

#include "core/arm/dyncom/jit/jit.h"

namespace Jit {

constexpr Gen::X64Reg JitStateReg = Gen::RSP; // R15 will *always* contain the state register.

const static std::array<Gen::X64Reg, NUM_REG_GPR> IntToArmGPR {{
    Gen::RAX, //R0
    Gen::RBX, //R1
    Gen::RCX, //R2
    Gen::RDX, //R3
    Gen::RSI, //R4
    Gen::RDI, //R5
    Gen::RBP, //R6
    Gen::R15, //R7
    Gen::R8,  //R8
    Gen::R9,  //R9
    Gen::R10, //R10
    Gen::R11, //R11
    Gen::R12, //R12
    Gen::R13, //R13 == Stack Pointer
    Gen::R14, //R14 == Link Register
    // R15 is virtual and is calculated at compile time when necessary == Program Counter
}};

}