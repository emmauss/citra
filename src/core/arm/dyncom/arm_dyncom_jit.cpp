// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <cstdio>

#include "common/common_types.h"
#include "common/logging/log.h"
#include "common/microprofile.h"
#include "common/profiler.h"
#include "common/x64/abi.h"
#include "common/x64/emitter.h"

#include "core/memory.h"
#include "core/hle/svc.h"
#include "core/arm/disassembler/arm_disasm.h"
#include "core/arm/dyncom/arm_dyncom_dec.h"
#include "core/arm/dyncom/arm_dyncom_jit.h"
#include "core/arm/dyncom/arm_dyncom_thumb.h"
#include "core/arm/dyncom/arm_dyncom_translate.h"
#include "core/arm/dyncom/arm_dyncom_run.h"
#include "core/arm/skyeye_common/armstate.h"
#include "core/arm/skyeye_common/armsupp.h"
#include "core/arm/skyeye_common/vfp/vfp.h"

constexpr std::size_t NUM_REG_GPR = 15;

struct alignas(64) JitState {
    // Emulated CPU state
    u32 spill[NUM_REG_GPR];
    u8 N, Z, C, V;
    // Try to keep everything above here in one cache line.

    u8 T;

    void* bb;
    u64 save_host_RSP;
    s32 cycles_remaining;
    u32 final_PC;
    u64 return_RIP;

    ARMul_State* interp_state;
};
static_assert(std::is_pod<JitState>::value, "Jit stack layout needs to be POD");

const static std::array<Gen::X64Reg, NUM_REG_GPR> IntToArmGPR {{
    Gen::RAX, //R0
    Gen::RBX, //R1
    Gen::RCX, //R2
    Gen::RDX, //R3
    Gen::RSI, //R4
    Gen::RDI, //R5
    Gen::RBP, //R6
    Gen::R13, //R7
    Gen::R8,  //R8
    Gen::R9,  //R9
    Gen::R10, //R10
    Gen::R11, //R11
    Gen::R12, //R12
    Gen::RSP, //R13 == Stack Pointer
    Gen::R14, //R14 == Link Register
    // R15 is virtual and is calculated at compile time when necessary == Program Counter
}};

constexpr Gen::X64Reg JitStateReg = Gen::R15; // R15 will *always* contain the state register.

typedef unsigned int(*shtop_fp_t)(ARMul_State* cpu, unsigned int sht_oper);

struct RegisterAllocation {
    RegisterAllocation() {
        /// Default state

        for (int i = 0; i < NUM_REG_GPR; i++) {
            is_spilled[i] = false;
            is_in_use[i] = false;
            arm_reg_last_used[i] = 2;
        }

        // Heurestic: When fresh, encourage spilling less commonly used registers first.
        arm_reg_last_used[0] = 3;
        arm_reg_last_used[1] = 3;
        arm_reg_last_used[7] = 1;
        arm_reg_last_used[8] = 0;
        arm_reg_last_used[9] = 0;
    }

    void assert_no_temporaries() {
        for (int i = 0; i < NUM_REG_GPR; i++) {
            ASSERT(!is_in_use[i]);
        }
    }

    /// Last ARM11 PC for which this ARM register was referenced.
    /// Used to decide what to spill for temporaries.
    std::array<int, NUM_REG_GPR> arm_reg_last_used;

    std::array<bool, NUM_REG_GPR> is_spilled;
    std::array<bool, NUM_REG_GPR> is_in_use;
};

namespace Gen {
struct JitBasicBlock : private XCodeBlock {
public:
    void* GetRunPtr() { return region; }

    int Compile(ARMul_State* cpu, void*& bb_start, u32 addr);

private:
    ARMul_State* cpu;

    RegisterAllocation current_register_allocation;
    X64Reg AcquireArmRegister(int reg);
    X64Reg AcquireTemporaryRegister();
    void ReleaseAllRegisters();
    void SpillAllRegisters();
    void ResetAllocation();
    void RestoreRSP();

    void CallHostFunction(void* fn, u64, u64, u64);

    FixupBranch current_cond_fixup = {};
    ConditionCode current_cond = ConditionCode::AL;
    void CompileCond(ConditionCode new_cond);

private:
    int pc;
    int TFlag;
    bool CompileSingleInstruction();
};

#define SAVE_NZCVT cpu->Cpsr = (cpu->Cpsr & 0x0fffffdf) | \
                      (cpu->NFlag << 31) | \
                      (cpu->ZFlag << 30) | \
                      (cpu->CFlag << 29) | \
                      (cpu->VFlag << 28) | \
                      (cpu->TFlag << 5)
#define LOAD_NZCVT cpu->NFlag = (cpu->Cpsr >> 31);     \
                       cpu->ZFlag = (cpu->Cpsr >> 30) & 1; \
                       cpu->CFlag = (cpu->Cpsr >> 29) & 1; \
                       cpu->VFlag = (cpu->Cpsr >> 28) & 1; \
                       cpu->TFlag = (cpu->Cpsr >> 5) & 1;

struct RunJit : private XCodeBlock {
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

    unsigned CallCode(ARMul_State* cpu, void* bb, s32 cycles_to_run) {
        JitState* jit_state = new JitState;
        jit_state->interp_state = cpu;

        LOAD_NZCVT;

        for (int i = 0; i < NUM_REG_GPR; i++) {
            jit_state->spill[i] = cpu->Reg[i];
        }

        jit_state->N = cpu->NFlag ? 1 : 0;
        jit_state->Z = cpu->ZFlag ? 1 : 0;
        jit_state->C = cpu->CFlag ? 1 : 0;
        jit_state->V = cpu->VFlag ? 1 : 0;
        jit_state->T = cpu->TFlag ? 1 : 0;

        jit_state->bb = bb;
        jit_state->cycles_remaining = cycles_to_run;
        jit_state->return_RIP = (u64)CallCodeReturnAddress();

        auto fn = (void(*)(u64))region;
        fn((u64)jit_state);

        cycles_to_run = jit_state->cycles_remaining;
        cpu->Reg[15] = jit_state->final_PC;

        cpu->NFlag = jit_state->N;
        cpu->ZFlag = jit_state->Z;
        cpu->CFlag = jit_state->C;
        cpu->VFlag = jit_state->V;
        cpu->TFlag = jit_state->T;

        for (int i = 0; i < NUM_REG_GPR; i++) {
            cpu->Reg[i] = jit_state->spill[i];
        }

        SAVE_NZCVT;

        unsigned cycles_remaining = jit_state->cycles_remaining;

        delete jit_state;

        return cycles_to_run - cycles_remaining;
    }

    const u8* CallCodeReturnAddress() {
        return return_from_run_jit;
    }
};
};

Gen::RunJit run_jit;

JitState* __cdecl InterpretSingleInstruction(JitState* jit_state, u64 pc, u64 TFlag) {
    ARMul_State* cpu = jit_state->interp_state;

    cpu->Reg[15] = pc;

    cpu->NFlag = jit_state->N;
    cpu->ZFlag = jit_state->Z;
    cpu->CFlag = jit_state->C;
    cpu->VFlag = jit_state->V;
    cpu->TFlag = TFlag ? 1 : 0;

    for (int i = 0; i < NUM_REG_GPR; i++) {
        cpu->Reg[i] = jit_state->spill[i];
    }

    SAVE_NZCVT;

    cpu->NumInstrsToExecute = 1;
    extern unsigned InterpreterMainLoop(ARMul_State* cpu);
    InterpreterMainLoop(cpu);

    LOAD_NZCVT;

    for (int i = 0; i < NUM_REG_GPR; i++) {
        jit_state->spill[i] = cpu->Reg[i];
    }

    jit_state->N = (u8)cpu->NFlag;
    jit_state->Z = (u8)cpu->ZFlag;
    jit_state->C = (u8)cpu->CFlag;
    jit_state->V = (u8)cpu->VFlag;
    jit_state->T = (u8)cpu->TFlag;

    jit_state->final_PC = cpu->Reg[15];

    return jit_state;
}

int Gen::JitBasicBlock::Compile(ARMul_State* cpu, void*& bb_start, u32 addr) {
    this->cpu = cpu;
    this->pc = addr;
    this->TFlag = cpu->TFlag;

    AllocCodeSpace(4096);
    bb_start = this->region;

    int cycles = 0;
    bool cont = true;

    while (cont && ((GetCodePtr() - region) < (region_size - 100))) {
        cont = CompileSingleInstruction();
        cycles++;
    }

    ResetAllocation();
    //MOV(32, MDisp(JitStateReg, offsetof(JitState, final_PC)), Imm32(this->pc));
    SUB(32, MDisp(JitStateReg, offsetof(JitState, cycles_remaining)), Imm32(cycles));
    JMPptr(MDisp(JitStateReg, offsetof(JitState, return_RIP)));

    return 0;
}

/// Returns false if you should not continue
bool Gen::JitBasicBlock::CompileSingleInstruction() {
    int dummy = 0;
    unsigned inst_size = 4;
    arm_inst *inst = InterpreterTranslateSingle(TFlag, dummy, pc, &inst_size);

    switch (inst->idx) {
    default:
        CallHostFunction(InterpretSingleInstruction, pc, this->TFlag, 0);
        ReleaseAllRegisters();
        return false;
    }
    return true;
}

unsigned JitMainLoop(ARMul_State* cpu) {
    int cycles_required = cpu->NumInstrsToExecute;
    unsigned cycles_done = 0;

    while (cycles_required > 0) {
        LOAD_NZCVT;

        if (!cpu->NirqSig) {
            ASSERT_MSG(0, "Unimplemented");
        }

        if (cpu->TFlag)
            cpu->Reg[15] &= 0xfffffffe;
        else
            cpu->Reg[15] &= 0xfffffffc;

        SAVE_NZCVT;

        void *ptr;

        auto itr = cpu->jit_cache.find(cpu->Reg[15]);
        if (itr != cpu->jit_cache.end()) {
            ptr = itr->second->GetRunPtr();
        } else {
            auto bb = new Gen::JitBasicBlock();
            bb->Compile(cpu, ptr, cpu->Reg[15]);
            cpu->jit_cache[cpu->Reg[15]] = bb;
        }

        unsigned cycles = run_jit.CallCode(cpu, ptr, cycles_required);
        cycles_required -= cycles;
        cycles_done += cycles;
    }

    return cycles_done;
}

void Gen::JitBasicBlock::CompileCond(ConditionCode new_cond) {
    if (new_cond == current_cond)
        return;

    if (current_cond != ConditionCode::AL) {
        ResetAllocation();
        SetJumpTarget(current_cond_fixup);
    }

    auto Z_flag = MDisp(JitStateReg, offsetof(JitState, Z));
    auto C_flag = MDisp(JitStateReg, offsetof(JitState, C));
    auto N_flag = MDisp(JitStateReg, offsetof(JitState, N));
    auto V_flag = MDisp(JitStateReg, offsetof(JitState, V));

    if (new_cond != ConditionCode::AL) {
        CCFlags cc;

        switch (new_cond) {
        case ConditionCode::EQ: //z
            TEST(32, Z_flag, Imm8(1));
            cc = CC_Z;
            break;
        case ConditionCode::NE: //!z
            TEST(32, Z_flag, Imm8(1));
            cc = CC_NZ;
            break;
        case ConditionCode::CS: //c
            TEST(32, C_flag, Imm8(1));
            cc = CC_Z;
            break;
        case ConditionCode::CC: //!c
            TEST(32, C_flag, Imm8(1));
            cc = CC_NZ;
            break;
        case ConditionCode::MI: //n
            TEST(32, N_flag, Imm8(1));
            cc = CC_Z;
            break;
        case ConditionCode::PL: //!n
            TEST(32, N_flag, Imm8(1));
            cc = CC_NZ;
            break;
        case ConditionCode::VS: //v
            TEST(32, V_flag, Imm8(1));
            cc = CC_Z;
            break;
        case ConditionCode::VC: //!v
            TEST(32, V_flag, Imm8(1));
            cc = CC_NZ;
            break;
        case ConditionCode::HI: { //c & !z
            X64Reg tmp = AcquireTemporaryRegister();
            MOV(32, R(tmp), Z_flag);
            CMP(8, C_flag, R(tmp));
            cc = CC_BE;
            break;
        }
        case ConditionCode::LS: { //!c | z
            X64Reg tmp = AcquireTemporaryRegister();
            MOV(32, R(tmp), Z_flag);
            CMP(8, C_flag, R(tmp));
            cc = CC_A;
            break;
        }
        case ConditionCode::GE: { // n == v
            X64Reg tmp = AcquireTemporaryRegister();
            MOV(32, R(tmp), V_flag);
            CMP(8, N_flag, R(tmp));
            cc = CC_NE;
            break;
        }
        case ConditionCode::LT: { // n != v
            X64Reg tmp = AcquireTemporaryRegister();
            MOV(32, R(tmp), V_flag);
            CMP(8, N_flag, R(tmp));
            cc = CC_E;
            break;
        }
        case ConditionCode::GT: { // !z & (n == v)
            X64Reg tmp = AcquireTemporaryRegister();
            MOV(32, R(tmp), V_flag);
            CMP(8, N_flag, R(tmp));
            SETcc(CC_E, R(tmp));
            CMP(8, R(tmp), Z_flag);
            cc = CC_BE;
            break;
        }
        case ConditionCode::LE: { // z | (n != v)
            X64Reg tmp = AcquireTemporaryRegister();
            MOV(32, R(tmp), N_flag);
            XOR(8, R(tmp), V_flag);
            OR(8, R(tmp), Z_flag);
            TEST(8, R(tmp), R(tmp));
            cc = CC_Z;
            break;
        }
        default:
            ASSERT_MSG(false, "This should never happen.");
            break;
        }

        ReleaseAllRegisters();
        ResetAllocation();
        this->current_cond_fixup = J_CC(cc);
    }

    current_cond = new_cond;
}

Gen::X64Reg Gen::JitBasicBlock::AcquireArmRegister(int arm_reg) {
    if (!current_register_allocation.is_spilled[arm_reg]) {
        current_register_allocation.is_in_use[arm_reg] = true;
        return IntToArmGPR[arm_reg];
    }

    ASSERT(!current_register_allocation.is_in_use[arm_reg]);
    current_register_allocation.is_in_use[arm_reg] = true;

    MOV(32, R(IntToArmGPR[arm_reg]), MDisp(JitStateReg, offsetof(JitState, spill) + arm_reg * sizeof(u32)));
    current_register_allocation.is_spilled[arm_reg] = false;
}

Gen::X64Reg Gen::JitBasicBlock::AcquireTemporaryRegister() {
    // First try to allocate something which was spilled previously and is now free.
    for (int i = 0; i < NUM_REG_GPR; i++) {
        if (current_register_allocation.is_spilled[i] && !current_register_allocation.is_in_use[i]) {
            current_register_allocation.is_in_use[i] = true;
            return IntToArmGPR[i];
        }
    }

    // Otherwise, spill something.
    int bestreg = -1;
    int bestreg_lastuse = INT_MAX;
    for (int i = 0; i < NUM_REG_GPR; i++) {
        if (!current_register_allocation.is_in_use[i]) {
            if (current_register_allocation.arm_reg_last_used[i] < bestreg_lastuse) {
                bestreg_lastuse = current_register_allocation.arm_reg_last_used[i];
                bestreg = i;
            }
        }
    }
    ASSERT(bestreg != -1);

    MOV(32, MDisp(JitStateReg, offsetof(JitState, spill) + bestreg * sizeof(u32)), R(IntToArmGPR[bestreg]));

    current_register_allocation.is_spilled[bestreg] = true;
    current_register_allocation.is_in_use[bestreg] = true;

    return IntToArmGPR[bestreg];
}

void Gen::JitBasicBlock::ReleaseAllRegisters() {
    for (int i = 0; i < NUM_REG_GPR; i++) {
        current_register_allocation.is_in_use[i] = false;
    }

    current_register_allocation.assert_no_temporaries();
}

void Gen::JitBasicBlock::SpillAllRegisters() {
    for (int i = 0; i < NUM_REG_GPR; i++) {
        if (!current_register_allocation.is_spilled[i]) {
            MOV(32, MDisp(JitStateReg, offsetof(JitState, spill) + i * sizeof(u32)), R(IntToArmGPR[i]));
            current_register_allocation.is_spilled[i] = true;
        }
    }
}

// Reset allocation MUST NOT TOUCH any conditional flags
void Gen::JitBasicBlock::ResetAllocation() {
    ReleaseAllRegisters();

    for (int i = 0; i < NUM_REG_GPR; i++) {
        if (current_register_allocation.is_spilled[i]) {
            MOV(32, R(IntToArmGPR[i]), MDisp(JitStateReg, offsetof(JitState, spill) + i * sizeof(u32)));
        }
    }

    current_register_allocation.assert_no_temporaries();
}

void Gen::JitBasicBlock::RestoreRSP() {
    ASSERT(IntToArmGPR[13] == RSP);
    ASSERT(!current_register_allocation.is_in_use[13]);
    current_register_allocation.is_in_use[13] = true;
    if (!current_register_allocation.is_spilled[13]) {
        MOV(32, MDisp(JitStateReg, offsetof(JitState, spill) + 13 * sizeof(u32)), R(RSP));
        current_register_allocation.is_spilled[13] = true;
    }
    MOV(64, R(RSP), MDisp(JitStateReg, offsetof(JitState, save_host_RSP)));
}

void Gen::JitBasicBlock::CallHostFunction(void *fn, u64 a, u64 b, u64 c) {
    SpillAllRegisters();
    RestoreRSP();
    MOV(64, R(ABI_PARAM1), R(JitStateReg));
    MOV(64, R(ABI_PARAM2), Imm64(a));
    MOV(64, R(ABI_PARAM3), Imm64(b));
    MOV(64, R(ABI_PARAM4), Imm64(c));
    u64 distance = u64(fn) - (u64(GetCodePtr()) + 5);
    if (distance >= 0x0000000080000000ULL
        && distance <  0xFFFFFFFF80000000ULL) {
        // Far call
        MOV(64, R(RAX), ImmPtr(fn));
        CALLptr(R(RAX));
    } else {
        CALL(fn);
    }
    MOV(64, R(JitStateReg), R(ABI_RETURN));
}

#define BEFORE_COMPILE_INSTRUCTION                      \
    if (BITS(inst, 28, 31) == ConditionCode::NV)        \
        return;                                         \
    CompileCond((ConditionCode)BITS(inst, 28, 31));

#define OPARG_SET_Z(oparg) MOV(8, MDisp(JitStateReg, offsetof(JitState, Z)), (oparg))
#define OPARG_SET_C(oparg) MOV(8, MDisp(JitStateReg, offsetof(JitState, C)), (oparg))
#define OPARG_SET_V(oparg) MOV(8, MDisp(JitStateReg, offsetof(JitState, V)), (oparg))
#define OPARG_SET_N(oparg) MOV(8, MDisp(JitStateReg, offsetof(JitState, N)), (oparg))
#define FLAG_SET_Z() SETcc(CC_Z, MDisp(JitStateReg, offsetof(JitState, Z)))
#define FLAG_SET_C() SETcc(CC_C, MDisp(JitStateReg, offsetof(JitState, C)))
#define FLAG_SET_V() SETcc(CC_O, MDisp(JitStateReg, offsetof(JitState, V)))
#define FLAG_SET_N() SETcc(CC_S, MDisp(JitStateReg, offsetof(JitState, N)))
