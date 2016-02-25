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

#define MJitStateCpu(name) MDisp(Jit::JitStateReg, offsetof(Jit::JitState, cpu_state) + offsetof(ARMul_State, name))
#define MJitStateCpuReg(reg_num) MDisp(Jit::JitStateReg, offsetof(Jit::JitState, cpu_state) + offsetof(ARMul_State, Reg) + (reg_num) * sizeof(u32))
#define MJitStateOther(name) MDisp(Jit::JitStateReg, offsetof(Jit::JitState, name))

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
            MOV(32, R(Jit::IntToArmGPR[i]), MJitStateCpuReg(i));
        }

        JMPptr(MDisp(Jit::JitStateReg, offsetof(Jit::JitState, bb)));
        return_from_run_jit = this->GetCodePtr();

        for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
            MOV(32, MJitStateCpuReg(i), R(Jit::IntToArmGPR[i]));
        }

        MOV(64, R(RSP), MJitStateOther(save_host_RSP));
        ABI_PopRegistersAndAdjustStack(ABI_ALL_CALLEE_SAVED, 8);
        RET();
    }

    unsigned CallCode(Jit::JitState* jit_state, void* bb, s32 cycles_to_run, u32& new_pc) {

        auto cpu = &jit_state->cpu_state;

        cpu->NFlag = (cpu->Cpsr >> 31);
        cpu->ZFlag = (cpu->Cpsr >> 30) & 1;
        cpu->CFlag = (cpu->Cpsr >> 29) & 1;
        cpu->VFlag = (cpu->Cpsr >> 28) & 1;
        cpu->TFlag = (cpu->Cpsr >> 5) & 1;

        jit_state->bb = bb;
        jit_state->cycles_remaining = cycles_to_run;
        jit_state->return_RIP = (u64)CallCodeReturnAddress();

        auto fn = (void(*)(u64))region;
        fn((u64)jit_state);

        new_pc = cpu->Reg[15];

        cpu->Cpsr = (cpu->Cpsr & 0x0fffffdf) |
                    (cpu->NFlag << 31) |
                    (cpu->ZFlag << 30) |
                    (cpu->CFlag << 29) |
                    (cpu->VFlag << 28) |
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

namespace Memory {
    struct PageTable;
    extern PageTable* current_page_table;
}

namespace Jit {

void JitState::Reset() {
    // Set stack pointer to the top of the stack
    cpu_state.Reset();
}

ARM_Jit::ARM_Jit(PrivilegeMode initial_mode) {
    ASSERT_MSG(initial_mode == PrivilegeMode::USER32MODE, "Unimplemented");
    state = std::make_unique<JitState>();
}

ARM_Jit::~ARM_Jit() {
}

void ARM_Jit::SetPC(u32 pc) {
    state->cpu_state.Reg[15] = pc;
}

u32 ARM_Jit::GetPC() const {
    return state->cpu_state.Reg[15];
}

u32 ARM_Jit::GetReg(int index) const {
    if (index == 15) return GetPC();
    return state->cpu_state.Reg[index];
}

void ARM_Jit::SetReg(int index, u32 value) {
    if (index == 15) return SetPC(value);
    state->cpu_state.Reg[index] = value;
}

u32 ARM_Jit::GetVFPReg(int index) const {
    return state->cpu_state.ExtReg[index];
}

void ARM_Jit::SetVFPReg(int index, u32 value) {
    state->cpu_state.ExtReg[index] = value;
}

u32 ARM_Jit::GetVFPSystemReg(VFPSystemRegister reg) const {
    return state->cpu_state.VFP[reg];
}

void ARM_Jit::SetVFPSystemReg(VFPSystemRegister reg, u32 value) {
    state->cpu_state.VFP[reg] = value;
}

u32 ARM_Jit::GetCPSR() const {
    return state->cpu_state.Cpsr;
}

void ARM_Jit::SetCPSR(u32 cpsr) {
    state->cpu_state.Cpsr = cpsr;
}

u32 ARM_Jit::GetCP15Register(CP15Register reg) {
    return state->cpu_state.CP15[reg];
}

void ARM_Jit::SetCP15Register(CP15Register reg, u32 value) {
    state->cpu_state.CP15[reg] = value;
}

void ARM_Jit::AddTicks(u64 ticks) {
    down_count -= ticks;
    if (down_count < 0)
        CoreTiming::Advance();
}

void ARM_Jit::ExecuteInstructions(int num_instructions) {
    state->reschedule = 0;

    do {
        state->page_table = reinterpret_cast<void*>(Memory::current_page_table);

        state->cpu_state.TFlag = (state->cpu_state.Cpsr >> 5) & 1;

        if (!state->cpu_state.NirqSig) {
            ASSERT_MSG(0, "Unimplemented");
        }

        if (state->cpu_state.TFlag)
            state->cpu_state.Reg[15] &= 0xfffffffe;
        else
            state->cpu_state.Reg[15] &= 0xfffffffc;

        void *ptr;

        auto itr = compiler.basic_blocks.find(state->cpu_state.Reg[15]);
        if (itr != compiler.basic_blocks.end()) {
            ptr = itr->second;
        } else {
            compiler.Compile(ptr, state->cpu_state.Reg[15], state->cpu_state.TFlag);
            compiler.basic_blocks[state->cpu_state.Reg[15]] = (u8*)ptr;
        }

        unsigned ticks_executed = run_jit.CallCode(state.get(), ptr, num_instructions, state->cpu_state.Reg[15]);
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
    memcpy(ctx.cpu_registers, state->cpu_state.Reg.data(), sizeof(ctx.cpu_registers));
    memcpy(ctx.fpu_registers, state->cpu_state.ExtReg.data(), sizeof(ctx.fpu_registers));

    ctx.sp = state->cpu_state.Reg[13];
    ctx.lr = state->cpu_state.Reg[14];
    ctx.pc = state->cpu_state.Reg[15];

    ctx.cpsr = GetCPSR();

    ctx.fpscr = state->cpu_state.VFP[1];
    ctx.fpexc = state->cpu_state.VFP[2];
}

void ARM_Jit::LoadContext(const Core::ThreadContext& ctx) {
    memcpy(state->cpu_state.Reg.data(), ctx.cpu_registers, sizeof(ctx.cpu_registers));
    memcpy(state->cpu_state.ExtReg.data(), ctx.fpu_registers, sizeof(ctx.fpu_registers));

    state->cpu_state.Reg[13] = ctx.sp;
    state->cpu_state.Reg[14] = ctx.lr;
    state->cpu_state.Reg[15] = ctx.pc;
    SetCPSR(ctx.cpsr);

    state->cpu_state.VFP[1] = ctx.fpscr;
    state->cpu_state.VFP[2] = ctx.fpexc;
}

void ARM_Jit::PrepareReschedule() {
    state->reschedule = 1;
    state->cpu_state.NumInstrsToExecute = 0;
}

void ARM_Jit::ClearCache() {
    compiler.basic_blocks.clear();
    this->state->cpu_state.instruction_cache.clear();
    compiler.ClearCache();
}

void ARM_Jit::DebugRun(u32 pc, int num_inst) {
    SetPC(pc);
    compiler.debug = true;
    ClearCache();
    ExecuteInstructions(num_inst);
    compiler.debug = false;
    ClearCache();
}

}
