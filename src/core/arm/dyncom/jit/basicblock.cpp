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
    NOP(); INT3(); NOP();

    current_cond = ConditionCode::AL;
    current_register_allocation.Reset();
    current_cond_fixup.ptr = nullptr;
    cycles = 0;

    this->pc = addr;
    this->TFlag = TFlag;

    bb_start = (void*)GetWritableCodePtr();

    bool cont = true;
    do {
        cont = CompileSingleInstruction();
        cycles++;
    } while (cont);

    ResetAllocation();
    CompileCond(ConditionCode::AL);

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
    case 144: return CompileInstruction_and(inst, inst_size);
    case 148: return CompileInstruction_add(inst, inst_size);
    case 152: return CompileInstruction_adc(inst, inst_size);
    default: return CompileInstruction_Interpret();
    }

    ASSERT_MSG(0, "Unreachable code");
}

bool Gen::JitCompiler::CompileInstruction_Interpret() {
    CompileCond(ConditionCode::AL);
    CallHostFunction(Jit::InterpretSingleInstruction, this->pc, this->TFlag, 0);
    ReleaseAllRegisters();
    return false;
}

void Gen::JitCompiler::CompileCond(const ConditionCode new_cond) {
    if (new_cond == current_cond)
        return;

    if (current_cond != ConditionCode::AL && current_cond != ConditionCode::NV) {
        SUB(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, cycles_remaining)), Imm32(cycles));
        cycles = 0;

        ResetAllocation();
        ASSERT(current_cond_fixup.ptr);
        SetJumpTarget(current_cond_fixup);
    }

    if (new_cond != ConditionCode::AL && current_cond != ConditionCode::NV) {
        SUB(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, cycles_remaining)), Imm32(cycles));
        cycles = 0;

        CCFlags cc;

        switch (new_cond) {
        case ConditionCode::EQ: //z
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::NE: //!z
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::CS: //c
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::CC: //!c
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::MI: //n
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::PL: //!n
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::VS: //v
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::VC: //!v
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::HI: { //c & !z
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)));
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), R(tmp));
            cc = CC_BE;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::LS: { //!c | z
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)));
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), R(tmp));
            cc = CC_A;
            ReleaseTemporaryRegister(tmp);
break;
        }
        case ConditionCode::GE: { // n == v
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)));
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)), R(tmp));
            cc = CC_NE;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::LT: { // n != v
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)));
            CMP(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)), R(tmp));
            cc = CC_E;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::GT: { // !z & (n == v)
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)));
            XOR(8, R(tmp), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)));
            OR(8, R(tmp), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)));
            TEST(8, R(tmp), R(tmp));
            cc = CC_NZ;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::LE: { // z | (n != v)
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)));
            XOR(8, R(tmp), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)));
            OR(8, R(tmp), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)));
            TEST(8, R(tmp), R(tmp));
            cc = CC_Z;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        default:
            ASSERT_MSG(false, "This should never happen.");
            break;
        }

        ResetAllocation();
        this->current_cond_fixup = J_CC(cc, true);
    }

    current_cond = new_cond;
}

Gen::X64Reg Gen::JitCompiler::AcquireArmRegister(int arm_reg) {
    ASSERT(arm_reg >= 0 && arm_reg <= 14);
    ASSERT(!cl_active);

    current_register_allocation.arm_reg_last_used[arm_reg] = pc;

    if (!current_register_allocation.is_spilled[arm_reg]) {
        current_register_allocation.is_in_use[arm_reg] = true;
        return Jit::IntToArmGPR[arm_reg];
    }

    ASSERT(!current_register_allocation.is_in_use[arm_reg]);
    current_register_allocation.is_in_use[arm_reg] = true;

    MOV(32, R(Jit::IntToArmGPR[arm_reg]), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + arm_reg * sizeof(u32)));
    current_register_allocation.is_spilled[arm_reg] = false;

    return Jit::IntToArmGPR[arm_reg];
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
    ASSERT(!current_register_allocation.is_spilled[bestreg]);
    ASSERT(!current_register_allocation.is_in_use[bestreg]);

    MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + bestreg * sizeof(u32)), R(Jit::IntToArmGPR[bestreg]));

    current_register_allocation.is_spilled[bestreg] = true;
    current_register_allocation.is_in_use[bestreg] = true;

    return Jit::IntToArmGPR[bestreg];
}

Gen::X64Reg Gen::JitCompiler::AcquireCopyOfArmRegister(int arm_reg) {
    ASSERT(arm_reg >= 0 && arm_reg <= 14);

    if (!current_register_allocation.is_in_use[arm_reg]) {
        if (current_register_allocation.is_spilled[arm_reg]) {
            MOV(32, R(Jit::IntToArmGPR[arm_reg]), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + arm_reg * sizeof(u32)));
        } else {
            MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + arm_reg * sizeof(u32)), R(Jit::IntToArmGPR[arm_reg]));
        }

        current_register_allocation.is_spilled[arm_reg] = true;
        current_register_allocation.is_in_use[arm_reg] = true;

        return Jit::IntToArmGPR[arm_reg];
    } else {
        Gen::X64Reg tmp = AcquireTemporaryRegister();

        if (current_register_allocation.is_spilled[arm_reg]) {
            MOV(32, R(tmp), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + arm_reg * sizeof(u32)));
        } else {
            MOV(32, R(tmp), R(Jit::IntToArmGPR[arm_reg]));
        }

        return tmp;
    }
}

static int WhichArmRegInNativeReg(Gen::X64Reg native) {
    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (native == Jit::IntToArmGPR[i]) {
            return i;
        }
    }
    ASSERT_MSG(0, "Internal error");
}

void Gen::JitCompiler::AcquireCLRegister(int arm_reg_to_copy) {
    ASSERT(arm_reg_to_copy <= 14);
    cl_active = true;

    int arm_reg = WhichArmRegInNativeReg(RCX);
    if (!current_register_allocation.is_in_use[arm_reg]) {
        current_register_allocation.is_in_use[arm_reg] = true;

        cl_active_tmp = INVALID_REG;

        if (current_register_allocation.is_spilled[arm_reg]) {
            // HURRAH
        } else {
            MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + arm_reg * sizeof(u32)), R(RCX));
            current_register_allocation.is_spilled[arm_reg] = true;
        }
    } else {
        cl_active_tmp = AcquireTemporaryRegister();
        MOV(32, R(cl_active_tmp), R(RCX));
    }

    if (arm_reg_to_copy >= 0) {
        if (current_register_allocation.is_spilled[arm_reg_to_copy]) {
            MOV(32, R(RCX), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + arm_reg_to_copy * sizeof(u32)));
        } else if (arm_reg_to_copy != arm_reg) {
            MOV(32, R(RCX), R(Jit::IntToArmGPR[arm_reg_to_copy]));
        }
    }
}

void Gen::JitCompiler::ReleaseCLRegister() {
    if (cl_active_tmp == INVALID_REG) {
        current_register_allocation.is_in_use[WhichArmRegInNativeReg(RCX)] = false;
    } else {
        MOV(32, R(RCX), R(cl_active_tmp));
    }
    cl_active = false;
}

void Gen::JitCompiler::ReleaseAllRegisters() {
    ASSERT(!cl_active);

    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        current_register_allocation.is_in_use[i] = false;
    }

    current_register_allocation.assert_no_temporaries();
}

void Gen::JitCompiler::ReleaseTemporaryRegister(Gen::X64Reg reg) {
    ASSERT(!cl_active);

    int i = WhichArmRegInNativeReg(reg);
    ASSERT(current_register_allocation.is_spilled[i]);
    ASSERT(current_register_allocation.is_in_use[i]);

    current_register_allocation.is_in_use[i] = false;
}

void Gen::JitCompiler::SpillAllRegisters() {
    ASSERT(!cl_active);

    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (!current_register_allocation.is_spilled[i]) {
            MOV(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + i * sizeof(u32)), R(Jit::IntToArmGPR[i]));
            current_register_allocation.is_spilled[i] = true;
        }
    }
}

// Reset allocation MUST NOT TOUCH any conditional flags
void Gen::JitCompiler::ResetAllocation() {
    ASSERT(!cl_active);

    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (current_register_allocation.is_spilled[i]) {
            MOV(32, R(Jit::IntToArmGPR[i]), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, spill) + i * sizeof(u32)));
            current_register_allocation.is_spilled[i] = false;
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
        && distance < 0xFFFFFFFF80000000ULL) {
        // Far call
        MOV(64, R(RAX), ImmPtr(fn));
        CALLptr(R(RAX));
    } else {
        CALL(fn);
    }
    MOV(64, R(Jit::JitStateReg), R(ABI_RETURN));
}

#define BITS(s, a, b) ((s << ((sizeof(s) * 8 - 1) - b)) >> (sizeof(s) * 8 - b + a - 1))
#define BIT(s, n) ((s >> (n)) & 1)

#define ROTATE_RIGHT(n, i, l) ((n << (l - i)) | (n >> i))
#define ROTATE_LEFT(n, i, l)  ((n >> (l - i)) | (n << i))
#define ROTATE_RIGHT_32(n, i) ROTATE_RIGHT(n, i, 32)
#define ROTATE_LEFT_32(n, i)  ROTATE_LEFT(n, i, 32)

Gen::X64Reg Gen::JitCompiler::CompileShifterOperand(shtop_fp_t shtop_func, unsigned sht_oper, bool SCO, unsigned inst_size) {
    if (shtop_func == DPO(Immediate)) {
        Gen::X64Reg ret = AcquireTemporaryRegister();

        unsigned int immed_8 = BITS(sht_oper, 0, 7);
        unsigned int rotate_imm = BITS(sht_oper, 8, 11);
        unsigned int shifter_operand = ROTATE_RIGHT_32(immed_8, rotate_imm * 2);

        if (SCO) {
            if (rotate_imm == 0) {
                MOVZX(64, 8, ret, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)));
                MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), R(ret));
            } else {
                MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), Imm8(BIT(shifter_operand, 31)));
            }
        }

        MOV(32, R(ret), Imm32(shifter_operand));

        return ret;
    }
    if (shtop_func == DPO(Register)) {
        unsigned int rm = BITS(sht_oper, 0, 3);

        if (SCO) {
            Gen::X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)));
            MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), R(tmp));
            ReleaseTemporaryRegister(tmp);
        }

        if (rm != 15) {
            return AcquireArmRegister(rm);
        } else {
            Gen::X64Reg ret = AcquireTemporaryRegister();
            MOV(32, R(ret), Imm32(GetReg15(inst_size)));
            return ret;
        }
    }
    if (shtop_func == DPO(LogicalShiftLeftByImmediate)) {
        int shift_imm = BITS(sht_oper, 7, 11);
        unsigned int rm = BITS(sht_oper, 0, 3);
        Gen::X64Reg Rm;

        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }

        if (shift_imm == 0) {
            if (SCO) {
                BT(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), Imm8(0));
                SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            }
            return Rm;
        } else {
            SHL(32, R(Rm), Imm8(shift_imm));
            if (SCO) {
                SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            }
            return Rm;
        }
    }
    if (shtop_func == DPO(LogicalShiftLeftByRegister)) {
        unsigned int rm = BITS(sht_oper, 0, 3);
        unsigned int rs = BITS(sht_oper, 8, 11);
        Gen::X64Reg Rm;
        if (rs != 15) {
            AcquireCLRegister(rs);
            AND(32, R(RCX), Imm32(0xFF));
        } else {
            AcquireCLRegister();
            MOV(32, R(RCX), Imm32(GetReg15(inst_size) & 0xFF));
        }
        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }

        if (!SCO) {
            SHL(32, R(Rm), R(CL));
            ReleaseCLRegister();
            return Rm;
        }

        TEST(32, R(RCX), R(RCX));
        auto Rs_not_zero = J_CC(CC_NZ);

        // if (Rs & 0xFF == 0) {
        MOVZX(64, 8, RCX, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)));
        MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), R(RCX));
        auto jmp_to_end_1 = J();
        // }
        SetJumpTarget(Rs_not_zero);
        CMP(32, R(RCX), Imm8(32));
        auto Rs_gt32 = J_CC(CC_A);
        auto Rs_eq32 = J_CC(CC_E);
        // else if (Rs & 0xFF < 32) {
        SHL(32, R(Rm), R(CL));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        auto jmp_to_end_2 = J();
        // }
        SetJumpTarget(Rs_gt32);
        // else if (Rs & 0xFF > 32) {
        MOV(32, R(Rm), Imm32(0));
        MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), Imm8(0));
        auto jmp_to_end_3 = J();
        // }
        SetJumpTarget(Rs_eq32);
        // else if (Rs & 0xFF == 32) {
        BT(32, R(Rm), Imm8(0));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        MOV(32, R(Rm), Imm32(0));
        // }
        SetJumpTarget(jmp_to_end_1);
        SetJumpTarget(jmp_to_end_2);
        SetJumpTarget(jmp_to_end_3);

        ReleaseCLRegister();
        return Rm;
    }
    if (shtop_func == DPO(LogicalShiftRightByImmediate)) {
        int shift_imm = BITS(sht_oper, 7, 11);
        unsigned int rm = BITS(sht_oper, 0, 3);
        Gen::X64Reg Rm;

        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }

        if (shift_imm == 0) {
            if (SCO) {
                BT(32, R(Rm), Imm8(31));
                SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            }
            MOV(64, R(Rm), Imm32(0));
            return Rm;
        }
        else {
            SHR(32, R(Rm), Imm8(shift_imm));
            if (SCO) {
                SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            }
            return Rm;
        }
    }
    if (shtop_func == DPO(LogicalShiftRightByRegister)) {
        unsigned int rm = BITS(sht_oper, 0, 3);
        unsigned int rs = BITS(sht_oper, 8, 11);
        Gen::X64Reg Rm;
        if (rs != 15) {
            AcquireCLRegister(rs);
            AND(32, R(RCX), Imm32(0xFF));
        } else {
            AcquireCLRegister();
            MOV(32, R(RCX), Imm32(GetReg15(inst_size) & 0xFF));
        }
        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }

        if (!SCO) {
            SHR(32, R(Rm), R(CL));
            ReleaseCLRegister();
            return Rm;
        }

        TEST(32, R(RCX), R(RCX));
        auto Rs_not_zero = J_CC(CC_NZ);

        // if (Rs & 0xFF == 0) {
        MOVZX(64, 8, RCX, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)));
        MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), R(RCX));
        auto jmp_to_end_1 = J();
        // }
        SetJumpTarget(Rs_not_zero);
        CMP(32, R(RCX), Imm8(32));
        auto Rs_gt32 = J_CC(CC_A);
        auto Rs_eq32 = J_CC(CC_E);
        // else if (Rs & 0xFF < 32) {
        SHR(32, R(Rm), R(RCX));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        auto jmp_to_end_2 = J();
        // }
        SetJumpTarget(Rs_gt32);
        // else if (Rs & 0xFF > 32) {
        MOV(32, R(Rm), Imm32(0));
        MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), Imm8(0));
        auto jmp_to_end_3 = J();
        // }
        SetJumpTarget(Rs_eq32);
        // else if (Rs & 0xFF == 32) {
        BT(32, R(Rm), Imm8(31));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        MOV(32, R(Rm), Imm32(0));
        // }
        SetJumpTarget(jmp_to_end_1);
        SetJumpTarget(jmp_to_end_2);
        SetJumpTarget(jmp_to_end_3);

        ReleaseCLRegister();
        return Rm;
    }
    if (shtop_func == DPO(ArithmeticShiftRightByImmediate)) {
        int shift_imm = BITS(sht_oper, 7, 11);
        unsigned int rm = BITS(sht_oper, 0, 3);
        Gen::X64Reg Rm;

        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }

        if (shift_imm == 0) {
            if (SCO) {
                BT(32, R(Rm), Imm8(31));
                SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            }
            SAR(32, R(Rm), Imm8(31));
            return Rm;
        } else {
            SAR(32, R(Rm), Imm8(shift_imm));
            if (SCO) {
                SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            }
            return Rm;
        }
    }
    if (shtop_func == DPO(ArithmeticShiftRightByRegister)) {
        unsigned int rm = BITS(sht_oper, 0, 3);
        unsigned int rs = BITS(sht_oper, 8, 11);
        Gen::X64Reg Rm;
        if (rs != 15) {
            AcquireCLRegister(rs);
            AND(32, R(RCX), Imm32(0xFF));
        } else {
            AcquireCLRegister();
            MOV(32, R(RCX), Imm32(GetReg15(inst_size) & 0xFF));
        }
        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }

        if (!SCO) {
            SAR(32, R(Rm), R(CL));
            ReleaseCLRegister();
            return Rm;
        }

        TEST(32, R(RCX), R(RCX));
        auto Rs_not_zero = J_CC(CC_NZ);

        // if (Rs & 0xFF == 0) {
        MOVZX(64, 8, RCX, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)));
        MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)), R(RCX));
        auto jmp_to_end_1 = J();
        // }
        SetJumpTarget(Rs_not_zero);
        CMP(32, R(RCX), Imm8(31));
        auto Rs_gt31 = J_CC(CC_A);
        // else if (Rs & 0xFF < 32) {
        SAR(32, R(Rm), R(CL));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        auto jmp_to_end_2 = J();
        // }
        SetJumpTarget(Rs_gt31);
        // else if (Rs & 0xFF > 31) {
        SAR(32, R(Rm), Imm8(31)); // Verified to have no incorrect counterexamples.
        BT(32, R(Rm), Imm8(31));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        // }
        SetJumpTarget(jmp_to_end_1);
        SetJumpTarget(jmp_to_end_2);

        ReleaseCLRegister();
        return Rm;
    }
    if (shtop_func == DPO(RotateRightByImmediate)) {
        int shift_imm = BITS(sht_oper, 7, 11);
        unsigned int rm = BITS(sht_oper, 0, 3);
        Gen::X64Reg Rm = AcquireCopyOfArmRegister(rm);
        if (shift_imm == 0) { //RRX
            BT(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), Imm8(0));
            RCR(32, R(Rm), Imm8(1));
            if (SCO) SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            return Rm;
        } else {
            ROR(32, R(Rm), Imm8(shift_imm));
            if (SCO) SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
            return Rm;
        }
    }
    if (shtop_func == DPO(RotateRightByRegister)) {
        unsigned int rm = BITS(sht_oper, 0, 3);
        unsigned int rs = BITS(sht_oper, 8, 11);
        Gen::X64Reg Rm;
        if (rs != 15) {
            AcquireCLRegister(rs);
        } else {
            AcquireCLRegister();
            MOV(32, R(RCX), Imm32(GetReg15(inst_size) & 0xFF));
        }
        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }

        if (!SCO) {
            ROR(32, R(Rm), R(RCX));
            ReleaseCLRegister();
            return Rm;
        }

        AND(32, R(RCX), Imm32(0xFF));
        auto zero_FF = J_CC(CC_Z);
        AND(32, R(RCX), Imm32(0x1F));
        auto zero_1F = J_CC(CC_Z);
        ROR(32, R(Rm), R(CL));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        auto done_1 = J();

        SetJumpTarget(zero_FF);
        BT(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), Imm8(0));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        auto done_2 = J();

        SetJumpTarget(zero_1F);
        BT(32, R(Rm), Imm8(31));
        SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));

        SetJumpTarget(done_1);
        SetJumpTarget(done_2);

        ReleaseCLRegister();
        return Rm;
    }

    ASSERT_MSG(0, "Unreachable");
    return Gen::INVALID_REG;
}

#define BEFORE_COMPILE_INSTRUCTION CompileCond((ConditionCode)inst->cond);

#define OPARG_SET_Z(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)), (oparg))
#define OPARG_SET_C(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), (oparg))
#define OPARG_SET_V(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)), (oparg))
#define OPARG_SET_N(oparg) MOV(8, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)), (oparg))
#define FLAG_SET_Z() SETcc(CC_Z, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, Z)))
#define FLAG_SET_C() SETcc(CC_C, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)))
#define FLAG_SET_V() SETcc(CC_O, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, V)))
#define FLAG_SET_N() SETcc(CC_S, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, N)))

bool Gen::JitCompiler::CompileInstruction_adc(arm_inst* inst, unsigned inst_size) {
    adc_inst* const inst_cream = (adc_inst*)inst->component;

    if (inst_cream->Rd == 15) {
        return CompileInstruction_Interpret();
    }

    BEFORE_COMPILE_INSTRUCTION;

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, false, inst_size);

    BT(32, MDisp(Jit::JitStateReg, offsetof(Jit::JitState, C)), Imm8(0));
    if (operand != Rd) {
        if (inst_cream->Rn == 15) {
            MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else if (Rd != Rn) {
            MOV(64, R(Rd), R(Rn));
        }
        ADC(32, R(Rd), R(operand));
    } else {
        if (inst_cream->Rn == 15) {
            ADC(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else {
            ADC(32, R(Rd), R(Rn));
        }
    }

    if (inst_cream->S && (inst_cream->Rd == 15)) {
        ASSERT_MSG(0, "Unimplemented");
    } else if (inst_cream->S) {
        FLAG_SET_Z();
        FLAG_SET_C();
        FLAG_SET_V();
        FLAG_SET_N();
    }

    if (inst_cream->Rd == 15) {
        ASSERT_MSG(0, "Unimplemented");
        return false;
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_and(arm_inst* inst, unsigned inst_size) {
    and_inst* const inst_cream = (and_inst*)inst->component;

    if (inst_cream->Rd == 15) {
        return CompileInstruction_Interpret();
    }

    BEFORE_COMPILE_INSTRUCTION;

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, true, inst_size);

    if (operand != Rd) {
        if (inst_cream->Rn == 15) {
            MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else if (Rd != Rn) {
            MOV(64, R(Rd), R(Rn));
        }
        AND(32, R(Rd), R(operand));
    }
    else {
        if (inst_cream->Rn == 15) {
            AND(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else {
            AND(32, R(Rd), R(Rn));
        }
    }

    if (inst_cream->S && (inst_cream->Rd == 15)) {
        ASSERT_MSG(0, "Unimplemented");
    } else if (inst_cream->S) {
        FLAG_SET_Z();
        Gen::X64Reg tmp = AcquireTemporaryRegister();
        MOV(8, R(tmp), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, shifter_carry_out)));
        OPARG_SET_C(R(tmp));
        ReleaseTemporaryRegister(tmp);
        FLAG_SET_N();
        // V is unaffected
    }

    if (inst_cream->Rd == 15) {
        ASSERT_MSG(0, "Unimplemented");
        return false;
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_add(arm_inst* inst, unsigned inst_size) {
    add_inst* const inst_cream = (add_inst*)inst->component;

    if (inst_cream->Rd == 15) {
        return CompileInstruction_Interpret();
    }

    BEFORE_COMPILE_INSTRUCTION;

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, false, inst_size);

    if (operand != Rd) {
        if (inst_cream->Rn == 15) {
            MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else if (Rd != Rn) {
            MOV(64, R(Rd), R(Rn));
        }
        ADD(32, R(Rd), R(operand));
    } else {
        if (inst_cream->Rn == 15) {
            ADD(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else {
            ADD(32, R(Rd), R(Rn));
        }
    }

    if (inst_cream->S && (inst_cream->Rd == 15)) {
        ASSERT_MSG(0, "Unimplemented");
    } else if (inst_cream->S) {
        FLAG_SET_Z();
        FLAG_SET_C();
        FLAG_SET_V();
        FLAG_SET_N();
    }

    if (inst_cream->Rd == 15) {
        ASSERT_MSG(0, "Unimplemented");
        return false;
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}
