// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/assert.h"
#include "common/x64/abi.h"
#include "common/x64/emitter.h"

#include "core/arm/dyncom/jit/basicblock.h"
#include "core/arm/dyncom/jit/common.h"
#include "core/arm/dyncom/jit/jit.h"
#include "core/core.h"
#include "core/core_timing.h"

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
        MOV(64, R(Jit::JitStateReg), R(ABI_PARAM1));
        MOV(64, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, save_host_RSP)), R(RSP));

        for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
            MOV(32, R(Jit::IntToArmGPR[i]), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + i * sizeof(u32)));
        }

        JMPptr(MDisp(Jit::JitStateReg, offsetof(Jit::JitState, bb)));
        return_from_run_jit = this->GetCodePtr();

        for (int i = 1; i < Jit::NUM_REG_GPR; i++) {
            MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + i * sizeof(u32)), R(Jit::IntToArmGPR[i]));
        }

        MOV(64, R(RSP), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, save_host_RSP)));
        ABI_PopRegistersAndAdjustStack(ABI_ALL_CALLEE_SAVED, 8);
        RET();
    }

    unsigned CallCode(Jit::JitState* jit_state, void* bb, s32 cycles_to_run, u32& new_pc) {

        auto cpu = jit_state->interp_state;

        jit_state->interp_state->NFlag = (cpu->Cpsr >> 31);     \
                       cpu->ZFlag = (cpu->Cpsr >> 30) & 1; \
                       cpu->CFlag = (cpu->Cpsr >> 29) & 1; \
                       cpu->VFlag = (cpu->Cpsr >> 28) & 1; \
                       cpu->TFlag = (cpu->Cpsr >> 5) & 1;

        for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
            jit_state->spill[i] = jit_state->interp_state->Reg[i];
        }

        jit_state->N = jit_state->interp_state->NFlag ? 1 : 0;
        jit_state->Z = jit_state->interp_state->ZFlag ? 1 : 0;
        jit_state->C = jit_state->interp_state->CFlag ? 1 : 0;
        jit_state->V = jit_state->interp_state->VFlag ? 1 : 0;
        jit_state->T = jit_state->interp_state->TFlag ? 1 : 0;

        jit_state->bb = bb;
        jit_state->cycles_remaining = cycles_to_run;
        jit_state->return_RIP = (u64)CallCodeReturnAddress();

        auto fn = (void(*)(u64))region;
        fn((u64)jit_state);

        cpu->NFlag = jit_state->N;
        cpu->ZFlag = jit_state->Z;
        cpu->CFlag = jit_state->C;
        cpu->VFlag = jit_state->V;
        cpu->TFlag = jit_state->T;

        for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
            cpu->Reg[i] = jit_state->spill[i];
        }

        new_pc = jit_state->final_PC;

        cpu->Cpsr = (cpu->Cpsr & 0x0fffffdf) | \
            (cpu->NFlag << 31) | \
            (cpu->ZFlag << 30) | \
            (cpu->CFlag << 29) | \
            (cpu->VFlag << 28) | \
            (cpu->TFlag << 5);

        return cycles_to_run - jit_state->cycles_remaining;
    }

    const u8* CallCodeReturnAddress() {
        return return_from_run_jit;
    }
};

}

static Gen::RunJit run_jit;
static Gen::JitCompiler compiler;

namespace Jit {

void JitState::Reset() {
    // Set stack pointer to the top of the stack
    spill[13] = 0x10000000;
}

ARM_Jit::ARM_Jit(PrivilegeMode initial_mode) {
    ASSERT_MSG(initial_mode == PrivilegeMode::USER32MODE, "Unimplemented");
    state = std::make_unique<JitState>();
    interp_state = std::make_unique<ARMul_State>(initial_mode);
    state->interp_state = interp_state.get();

//    pc = 0;

    state->N = (interp_state->Cpsr >> 31);
    state->Z = (interp_state->Cpsr >> 30) & 1;
    state->C = (interp_state->Cpsr >> 29) & 1;
    state->V = (interp_state->Cpsr >> 28) & 1;
    state->T = (interp_state->Cpsr >> 5) & 1;
}

ARM_Jit::~ARM_Jit() {
}

void ARM_Jit::SetPC(u32 pc) {
//    this->pc = pc;
    interp_state->Reg[15] = pc;
}

u32 ARM_Jit::GetPC() const {
//    return this->pc;
    return interp_state->Reg[15];
}

u32 ARM_Jit::GetReg(int index) const {
    if (index == 15) return GetPC();
    return interp_state->Reg[index];
}

void ARM_Jit::SetReg(int index, u32 value) {
    if (index == 15) return SetPC(value);
    interp_state->Reg[index] = value;
}

u32 ARM_Jit::GetVFPReg(int index) const {
    return interp_state->ExtReg[index];
}

void ARM_Jit::SetVFPReg(int index, u32 value) {
    interp_state->ExtReg[index] = value;
}

u32 ARM_Jit::GetVFPSystemReg(VFPSystemRegister reg) const {
    return interp_state->VFP[reg];
}

void ARM_Jit::SetVFPSystemReg(VFPSystemRegister reg, u32 value) {
    interp_state->VFP[reg] = value;
}

u32 ARM_Jit::GetCPSR() const {
    /*interp_state->Cpsr = (interp_state->Cpsr & 0x0fffffdf) |
        (state->N << 31) |
        (state->Z << 30) |
        (state->C << 29) |
        (state->V << 28) |
        (state->T << 5);*/
    return interp_state->Cpsr;
}

void ARM_Jit::SetCPSR(u32 cpsr) {
    interp_state->Cpsr = cpsr;
    /*
    state->N = (cpsr >> 31) & 1;
    state->Z = (cpsr >> 30) & 1;
    state->C = (cpsr >> 29) & 1;
    state->V = (cpsr >> 28) & 1;
    state->T = (cpsr >> 5) & 1;
    */
}

u32 ARM_Jit::GetCP15Register(CP15Register reg) {
    return interp_state->CP15[reg];
}

void ARM_Jit::SetCP15Register(CP15Register reg, u32 value) {
    interp_state->CP15[reg] = value;
}

void ARM_Jit::AddTicks(u64 ticks) {
    down_count -= ticks;
    if (down_count < 0)
        CoreTiming::Advance();
}

void ARM_Jit::ExecuteInstructions(int num_instructions) {
    state->reschedule = 0;

    do {
        interp_state->NFlag = (interp_state->Cpsr >> 31);
        interp_state->ZFlag = (interp_state->Cpsr >> 30) & 1;
        interp_state->CFlag = (interp_state->Cpsr >> 29) & 1;
        interp_state->VFlag = (interp_state->Cpsr >> 28) & 1;
        interp_state->TFlag = (interp_state->Cpsr >> 5) & 1;

        if (!interp_state->NirqSig) {
            ASSERT_MSG(0, "Unimplemented");
        }

        if (interp_state->TFlag)
            interp_state->Reg[15] &= 0xfffffffe;
        else
            interp_state->Reg[15] &= 0xfffffffc;

        interp_state->Cpsr = (interp_state->Cpsr & 0x0fffffdf) |
            (interp_state->NFlag << 31) |
            (interp_state->ZFlag << 30) |
            (interp_state->CFlag << 29) |
            (interp_state->VFlag << 28) |
            (interp_state->TFlag << 5);

        void *ptr;

        auto itr = basic_blocks.find(interp_state->Reg[15]);
        if (itr != basic_blocks.end()) {
            ptr = itr->second;
        } else {
            compiler.Compile(ptr, interp_state->Reg[15], interp_state->TFlag);
            basic_blocks[interp_state->Reg[15]] = (u8*)ptr;
        }

        unsigned ticks_executed = run_jit.CallCode(state.get(), ptr, num_instructions, interp_state->Reg[15]);
        num_instructions -= ticks_executed;
        AddTicks(ticks_executed);
    } while (!state->reschedule && num_instructions > 0);
}

void ARM_Jit::ResetContext(Core::ThreadContext& context, u32 stack_top, u32 entry_point, u32 arg) {
    memset(&context, 0, sizeof(Core::ThreadContext));

    context.cpu_registers[0] = arg;
    context.pc = entry_point;
    context.sp = stack_top;
    context.cpsr = 0x1F; // Usermode
}

void ARM_Jit::SaveContext(Core::ThreadContext& ctx) {
    //memcpy(ctx.cpu_registers, state->spill, sizeof(ctx.cpu_registers));
    memcpy(ctx.cpu_registers, interp_state->Reg.data(), sizeof(ctx.cpu_registers));
    memcpy(ctx.fpu_registers, interp_state->ExtReg.data(), sizeof(ctx.fpu_registers));

    //ctx.sp = state->spill[13];
    //ctx.lr = state->spill[14];
    //ctx.pc = this->pc;
    ctx.sp = interp_state->Reg[13];
    ctx.lr = interp_state->Reg[14];
    ctx.pc = interp_state->Reg[15];

    ctx.cpsr = GetCPSR();

    ctx.fpscr = interp_state->VFP[1];
    ctx.fpexc = interp_state->VFP[2];
}

void ARM_Jit::LoadContext(const Core::ThreadContext& ctx) {
    //memcpy(state->spill, ctx.cpu_registers, sizeof(ctx.cpu_registers));
    memcpy(interp_state->Reg.data(), ctx.cpu_registers, sizeof(ctx.cpu_registers));
    memcpy(interp_state->ExtReg.data(), ctx.fpu_registers, sizeof(ctx.fpu_registers));

    //state->spill[13] = ctx.sp;
    //state->spill[14] = ctx.lr;
    //this->pc = ctx.pc;
    interp_state->Reg[13] = ctx.sp;
    interp_state->Reg[14] = ctx.lr;
    interp_state->Reg[15] = ctx.pc;
    SetCPSR(ctx.cpsr);

    interp_state->VFP[1] = ctx.fpscr;
    interp_state->VFP[2] = ctx.fpexc;
}

void ARM_Jit::PrepareReschedule() {
    state->reschedule = 1;
    interp_state->NumInstrsToExecute = 0;
}


}
