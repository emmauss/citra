// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>

#include "common/common_types.h"

#include "common/common_types.h"
#include "common/x64/abi.h"
#include "common/x64/emitter.h"

#include "core/arm/dyncom/jit/jit.h"

namespace Gen {

struct RunJit final : private XCodeBlock {
private:
    const u8* run_jit;
    const u8* return_from_run_jit;

public:
    RunJit() {
        // TODO: Optimize this

        AllocCodeSpace(1024);

        ////////////////////////////////////////////////////////////////////////////////
        // Run Jitted Code

        run_jit = this->GetCodePtr();

        ABI_PushRegistersAndAdjustStack(ABI_ALL_CALLEE_SAVED, 8);
        MOV(64, R(JitStateReg), R(ABI_PARAM1));
        MOV(64, MDisp(JitStateReg, offsetof(JitState, save_host_RSP)), R(RSP));

        for (int i = 0; i < NUM_REG_GPR; i++) {
            MOV(32, R(IntToArmGPR[i]), MDisp(JitStateReg, offsetof(JitState, spill) + i * sizeof(u32)));
        }

        JMPptr(MDisp(JitStateReg, offsetof(JitState, bb)));
        return_from_run_jit = this->GetCodePtr();

        for (int i = 1; i < NUM_REG_GPR; i++) {
            MOV(32, MDisp(JitStateReg, offsetof(JitState, spill) + i * sizeof(u32)), R(IntToArmGPR[i]));
        }

        MOV(64, R(RSP), MDisp(JitStateReg, offsetof(JitState, save_host_RSP)));
        ABI_PopRegistersAndAdjustStack(ABI_ALL_CALLEE_SAVED, 8);
        RET();
    }

    unsigned CallCode(JitState* jit_state, void* bb, s32 cycles_to_run) {
        jit_state->bb = bb;
        jit_state->cycles_remaining = cycles_to_run;
        jit_state->return_RIP = (u64)CallCodeReturnAddress();

        auto fn = (void(*)(u64))region;
        fn((u64)jit_state);

        return cycles_to_run - jit_state->cycles_remaining;
    }

    const u8* CallCodeReturnAddress() {
        return return_from_run_jit;
    }
};

extern RunJit run_jit;

}