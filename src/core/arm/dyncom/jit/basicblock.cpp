#include "core/arm/dyncom/arm_dyncom_translate.h"
#include "core/arm/dyncom/jit/basicblock.h"

extern unsigned InterpreterMainLoop(ARMul_State* cpu);

namespace Jit {
Jit::JitState* __cdecl InterpretSingleInstruction(Jit::JitState* jit_state, u64 pc, u64 TFlag, u64) {
    ARMul_State* cpu = jit_state->interp_state;

    cpu->Reg[15] = pc;

    cpu->NFlag = jit_state->N ? 1 : 0;
    cpu->ZFlag = jit_state->Z ? 1 : 0;
    cpu->CFlag = jit_state->C ? 1 : 0;
    cpu->VFlag = jit_state->V ? 1 : 0;
    cpu->TFlag = TFlag ? 1 : 0;

    for (int i = 0; i < NUM_REG_GPR; i++) {
        cpu->Reg[i] = jit_state->spill[i];
    }

    cpu->Cpsr = (cpu->Cpsr & 0x0fffffdf) |
        (cpu->NFlag << 31) |
        (cpu->ZFlag << 30) |
        (cpu->CFlag << 29) |
        (cpu->VFlag << 28) |
        (cpu->TFlag << 5);

    cpu->NumInstrsToExecute = 1;
    InterpreterMainLoop(cpu);

    cpu->NFlag = (cpu->Cpsr >> 31);
    cpu->ZFlag = (cpu->Cpsr >> 30) & 1;
    cpu->CFlag = (cpu->Cpsr >> 29) & 1;
    cpu->VFlag = (cpu->Cpsr >> 28) & 1;
    cpu->TFlag = (cpu->Cpsr >> 5) & 1;

    for (int i = 0; i < NUM_REG_GPR; i++) {
        jit_state->spill[i] = cpu->Reg[i];
    }

    jit_state->N = cpu->NFlag ? 1 : 0;
    jit_state->Z = cpu->ZFlag ? 1 : 0;
    jit_state->C = cpu->CFlag ? 1 : 0;
    jit_state->V = cpu->VFlag ? 1 : 0;
    jit_state->T = cpu->TFlag ? 1 : 0;

    jit_state->final_PC = cpu->Reg[15];

    return jit_state;
}
}

int Gen::JitCompiler::Compile(void*& bb_start, u32 addr, bool TFlag) {
    this->pc = addr;
    this->TFlag = TFlag;

    bb_start = (void*)GetWritableCodePtr();

    int cycles = 0;
    bool cont = true;

    do {
        cont = CompileSingleInstruction();
        cycles++;
    } while (cont);

    ResetAllocation();
    //MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, final_PC)), Imm32(this->pc));
    SUB(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, cycles_remaining)), Imm32(cycles));
    JMPptr(MDisp(Jit::JitStateReg, offsetof(Jit::JitState, return_RIP)));

    return 0;
}

/// Returns false if you should not continue
bool Gen::JitCompiler::CompileSingleInstruction() {
    int dummy = 0;
    unsigned inst_size = 4;
    arm_inst *inst = InterpreterTranslateSingle(TFlag, dummy, pc, &inst_size);

    switch (inst->idx) {
    default:
        CallHostFunction(Jit::InterpretSingleInstruction, pc, this->TFlag, 0);
        ReleaseAllRegisters();
        return false;
    }
    return true;
}

void Gen::JitCompiler::CompileCond(ConditionCode new_cond) {
    if (new_cond == current_cond)
        return;

    if (current_cond != ConditionCode::AL) {
        ResetAllocation();
        SetJumpTarget(current_cond_fixup);
    }

    auto Z_flag = MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z));
    auto C_flag = MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C));
    auto N_flag = MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N));
    auto V_flag = MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V));

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

Gen::X64Reg Gen::JitCompiler::AcquireArmRegister(int arm_reg) {
    if (!current_register_allocation.is_spilled[arm_reg]) {
        current_register_allocation.is_in_use[arm_reg] = true;
        return Jit::IntToArmGPR[arm_reg];
    }

    ASSERT(!current_register_allocation.is_in_use[arm_reg]);
    current_register_allocation.is_in_use[arm_reg] = true;

    MOV(32, R(Jit::IntToArmGPR[arm_reg]), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + arm_reg * sizeof(u32)));
    current_register_allocation.is_spilled[arm_reg] = false;
}

Gen::X64Reg Gen::JitCompiler::AcquireTemporaryRegister() {
    // First try to allocate something which was spilled previously and is now free.
    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (current_register_allocation.is_spilled[i] && !current_register_allocation.is_in_use[i]) {
            current_register_allocation.is_in_use[i] = true;
            return Jit::IntToArmGPR[i];
        }
    }

    // Otherwise, spill something.
    int bestreg = -1;
    int bestreg_lastuse = INT_MAX;
    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (!current_register_allocation.is_in_use[i]) {
            if (current_register_allocation.arm_reg_last_used[i] < bestreg_lastuse) {
                bestreg_lastuse = current_register_allocation.arm_reg_last_used[i];
                bestreg = i;
            }
        }
    }
    ASSERT(bestreg != -1);

    MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + bestreg * sizeof(u32)), R(Jit::IntToArmGPR[bestreg]));

    current_register_allocation.is_spilled[bestreg] = true;
    current_register_allocation.is_in_use[bestreg] = true;

    return Jit::IntToArmGPR[bestreg];
}

void Gen::JitCompiler::ReleaseAllRegisters() {
    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        current_register_allocation.is_in_use[i] = false;
    }

    current_register_allocation.assert_no_temporaries();
}

void Gen::JitCompiler::SpillAllRegisters() {
    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (!current_register_allocation.is_spilled[i]) {
            MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + i * sizeof(u32)), R(Jit::IntToArmGPR[i]));
            current_register_allocation.is_spilled[i] = true;
        }
    }
}

// Reset allocation MUST NOT TOUCH any conditional flags
void Gen::JitCompiler::ResetAllocation() {
    ReleaseAllRegisters();

    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (current_register_allocation.is_spilled[i]) {
            MOV(32, R(Jit::IntToArmGPR[i]), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + i * sizeof(u32)));
        }
    }

    current_register_allocation.assert_no_temporaries();
}

void Gen::JitCompiler::RestoreRSP() {
    ASSERT(Jit::IntToArmGPR[13] == RSP);
    ASSERT(!current_register_allocation.is_in_use[13]);
    current_register_allocation.is_in_use[13] = true;
    if (!current_register_allocation.is_spilled[13]) {
        MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + 13 * sizeof(u32)), R(RSP));
        current_register_allocation.is_spilled[13] = true;
    }
    MOV(64, R(RSP), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, save_host_RSP)));
}

void Gen::JitCompiler::CallHostFunction(Jit::JitState*(*fn)(Jit::JitState*,u64,u64,u64), u64 a, u64 b, u64 c) {
    SpillAllRegisters();
    RestoreRSP();
    MOV(64, R(ABI_PARAM1), R(Jit::JitStateReg));
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
    MOV(64, R(Jit::JitStateReg), R(ABI_RETURN));
}

#define BEFORE_COMPILE_INSTRUCTION                      \
    if (BITS(inst, 28, 31) == ConditionCode::NV)        \
        return;                                         \
    CompileCond((ConditionCode)BITS(inst, 28, 31));

#define OPARG_SET_Z(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)), (oparg))
#define OPARG_SET_C(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), (oparg))
#define OPARG_SET_V(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)), (oparg))
#define OPARG_SET_N(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)), (oparg))
#define FLAG_SET_Z() SETcc(CC_Z, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)))
#define FLAG_SET_C() SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)))
#define FLAG_SET_V() SETcc(CC_O, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)))
#define FLAG_SET_N() SETcc(CC_S, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)))
