#include "core/arm/dyncom/arm_dyncom_translate.h"
#include "core/arm/dyncom/jit/basicblock.h"
#include "core/memory.h"

#include "core/arm/disassembler/arm_disasm.h"

extern unsigned InterpreterMainLoop(ARMul_State* cpu);

#define MJitStateCpu(name) MDisp(Jit::JitStateReg, offsetof(Jit::JitState, cpu_state) + offsetof(ARMul_State, name))
#define MJitStateCpuReg(reg_num) MDisp(Jit::JitStateReg, offsetof(Jit::JitState, cpu_state) + offsetof(ARMul_State, Reg) + (reg_num) * sizeof(u32))
#define MJitStateOther(name) MDisp(Jit::JitStateReg, offsetof(Jit::JitState, name))

namespace Jit {
Jit::JitState* InterpretSingleInstruction(Jit::JitState* jit_state, u64 pc, u64 TFlag, u64) {
    ARMul_State* cpu = &jit_state->cpu_state;

    cpu->Reg[15] = pc;

    cpu->Cpsr = (cpu->Cpsr & 0x0fffffdf) |
        (cpu->NFlag << 31) |
        (cpu->ZFlag << 30) |
        (cpu->CFlag << 29) |
        (cpu->VFlag << 28) |
        (cpu->TFlag << 5);

    cpu->NumInstrsToExecute = jit_state->cycles_remaining > 0 ? jit_state->cycles_remaining : 1;
    if (cpu->NumInstrsToExecute > 100) cpu->NumInstrsToExecute = 100;
    jit_state->cycles_remaining -= InterpreterMainLoop(cpu) - 1;

    return jit_state;
}
}

int Gen::JitCompiler::Compile(void*& bb_start, u32 addr, bool TFlag) {
    NOP(); INT3(); NOP(); // These are here to mark basic blocks so that they're easy to spot in a memory dump.

    this->pc = addr;
    this->TFlag = TFlag;

    bb_start = (void*)GetWritableCodePtr();
    if (debug) INT3();

    for (u8* ptr : update_jmps[addr]) {
        SetCodePtr(ptr);
        J_CC(CC_G, (u8*)bb_start, true);
    }
    update_jmps.erase(addr);
    SetCodePtr((u8*)bb_start);

    current_cond = ConditionCode::AL;
    current_register_allocation.Reset();
    current_cond_fixup.ptr = nullptr;
    cycles = 0;
    status_flag_update = false;

    ResetAllocation();

    bool cont = true;
    do {
        cycles++;
        cont = CompileSingleInstruction();
    } while (cont && (this->pc & 0xFFF));

    if (cont) {
        CompileCond(ConditionCode::AL);
        if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
        CompileMaybeJumpToBB(this->pc);
        MOV(32, MJitStateCpuReg(15), Imm32(this->pc));
        JMPptr(MJitStateOther(return_RIP));

        cycles = 0;
    }

    return 0;
}

/// Returns false if you should not continue
bool Gen::JitCompiler::CompileSingleInstruction() {
    int dummy = 0;
    unsigned inst_size = 4;
    arm_inst *inst = InterpreterTranslateSingle(TFlag, dummy, pc, &inst_size);

    CompileCond((ConditionCode)inst->cond);
    switch (inst->idx) {
    case 37: return CompileInstruction_Skip(inst_size); // PLD is a hint, we don't implement it.
    case 48: return CompileInstruction_cpy(inst, inst_size);
    case 95: return CompileInstruction_bx(inst, inst_size); // When BXJ fails, it behaves like BX.
    case 98: return CompileInstruction_bx(inst, inst_size);
    case 99: return CompileInstruction_rev(inst, inst_size);
    case 100: return CompileInstruction_blx(inst, inst_size);
    case 102: return CompileInstruction_q32(inst, inst_size); // QADD
    case 105: return CompileInstruction_ldrex(inst, inst_size);
    case 106: return CompileInstruction_q32(inst, inst_size); // QDADD
    case 107: return CompileInstruction_q32(inst, inst_size); // QDSUB
    case 108: return CompileInstruction_q32(inst, inst_size); // QSUB
    case 109: return CompileInstruction_ldrexb(inst, inst_size);
    case 123: return CompileInstruction_pkhtb(inst, inst_size);
    case 124: return CompileInstruction_pkhbt(inst, inst_size);
    case 130: return CompileInstruction_cmp(inst, inst_size);
    case 131: return CompileInstruction_tst(inst, inst_size);
    case 132: return CompileInstruction_teq(inst, inst_size);
    case 133: return CompileInstruction_cmn(inst, inst_size);
    case 138: return CompileInstruction_mul(inst, inst_size);
    case 139: return CompileInstruction_mla(inst, inst_size);
    case 144: return CompileInstruction_and(inst, inst_size);
    case 145: return CompileInstruction_bic(inst, inst_size);
    case 146: return CompileInstruction_ldm(inst, inst_size);
    case 147: return CompileInstruction_eor(inst, inst_size);
    case 148: return CompileInstruction_add(inst, inst_size);
    case 149: return CompileInstruction_rsb(inst, inst_size);
    case 150: return CompileInstruction_rsc(inst, inst_size);
    case 151: return CompileInstruction_sbc(inst, inst_size);
    case 152: return CompileInstruction_adc(inst, inst_size);
    case 153: return CompileInstruction_sub(inst, inst_size);
    case 154: return CompileInstruction_orr(inst, inst_size);
    case 155: return CompileInstruction_mvn(inst, inst_size);
    case 156: return CompileInstruction_mov(inst, inst_size);
    case 157: return CompileInstruction_stm(inst, inst_size);
    case 158: return CompileInstruction_ldm(inst, inst_size);
    case 159: return CompileInstruction_ldrsh(inst, inst_size);
    case 160: return CompileInstruction_stm(inst, inst_size);
    case 161: return CompileInstruction_ldm(inst, inst_size);
    case 162: return CompileInstruction_ldrsb(inst, inst_size);
    case 163: return CompileInstruction_strd(inst, inst_size);
    case 164: return CompileInstruction_ldrh(inst, inst_size);
    case 165: return CompileInstruction_strh(inst, inst_size);
    case 166: return CompileInstruction_ldrd(inst, inst_size);
    case 167: return CompileInstruction_str(inst, inst_size); // In usermode, STRT has some behaviour as STR.
    case 168: return CompileInstruction_strb(inst, inst_size); // In usermode, STRBT has some behaviour as STRB.
    case 169: return CompileInstruction_ldrb(inst, inst_size); // In usermode, LDRBT has some behaviour as LDRB.
    case 170: return CompileInstruction_ldr(inst, inst_size); // In usermode, LDRT has some behaviour as LDR.
    case 178: return CompileInstruction_ldrb(inst, inst_size);
    case 179: return CompileInstruction_strb(inst, inst_size);
    case 180: return CompileInstruction_ldr(inst, inst_size);
    case 181: return CompileInstruction_ldr(inst, inst_size);
    case 182: return CompileInstruction_str(inst, inst_size);
    case 183: ASSERT_MSG(0, "Undefined instruction in this context."); INT3(); return false; // CDP
    case 185: ASSERT_MSG(0, "Undefined instruction in this context."); INT3(); return false; // LDC
    case 186: return CompileInstruction_ldrexd(inst, inst_size);
    case 188: return CompileInstruction_ldrexh(inst, inst_size);
    case 196: return CompileInstruction_bl(inst, inst_size);
    case 197: return CompileInstruction_b_2_thumb(inst, inst_size);
    case 198: return CompileInstruction_b_cond_thumb(inst, inst_size);
    case 199: return CompileInstruction_bl_1_thumb(inst, inst_size);
    case 200: return CompileInstruction_bl_2_thumb(inst, inst_size);
    case 201: return CompileInstruction_blx_1_thumb(inst, inst_size);
    default:
        //printf("%i\n", inst->idx);
        return CompileInstruction_Interpret(inst_size);
    }

    ASSERT_MSG(0, "Unreachable code");
}

bool Gen::JitCompiler::CompileInstruction_Interpret(unsigned inst_size) {
    CompileCond(ConditionCode::AL);
    CallHostFunction(Jit::InterpretSingleInstruction, this->pc, this->TFlag, 0);
    ReleaseAllRegisters();
    this->pc += inst_size;

    ResetAllocation();
    CompileCond(ConditionCode::AL);
    ResetAllocation();
    if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
    cycles = 0;
    JMPptr(MJitStateOther(return_RIP));
    return false;
}

bool Gen::JitCompiler::CompileInstruction_Skip(unsigned inst_size) {
    this->pc += inst_size;
    return true;
}

void Gen::JitCompiler::CompileCond(const ConditionCode new_cond) {
    if (current_cond == new_cond && !status_flag_update)
        return;

    if (current_cond != ConditionCode::AL && current_cond != ConditionCode::NV) {
        ResetAllocation();
        ASSERT(current_cond_fixup.ptr);
        SetJumpTarget(current_cond_fixup);
        current_cond_fixup.ptr = nullptr;
    }

    if (new_cond != ConditionCode::AL && new_cond != ConditionCode::NV) {
        CCFlags cc;

        switch (new_cond) {
        case ConditionCode::EQ: //z
            CMP(8, MJitStateCpu(ZFlag), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::NE: //!z
            CMP(8, MJitStateCpu(ZFlag), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::CS: //c
            CMP(8, MJitStateCpu(CFlag), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::CC: //!c
            CMP(8, MJitStateCpu(CFlag), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::MI: //n
            CMP(8, MJitStateCpu(NFlag), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::PL: //!n
            CMP(8, MJitStateCpu(NFlag), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::VS: //v
            CMP(8, MJitStateCpu(VFlag), Imm8(0));
            cc = CC_E;
            break;
        case ConditionCode::VC: //!v
            CMP(8, MJitStateCpu(VFlag), Imm8(0));
            cc = CC_NE;
            break;
        case ConditionCode::HI: { //c & !z
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MJitStateCpu(ZFlag));
            CMP(8, MJitStateCpu(CFlag), R(tmp));
            cc = CC_BE;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::LS: { //!c | z
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MJitStateCpu(ZFlag));
            CMP(8, MJitStateCpu(CFlag), R(tmp));
            cc = CC_A;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::GE: { // n == v
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MJitStateCpu(VFlag));
            CMP(8, MJitStateCpu(NFlag), R(tmp));
            cc = CC_NE;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::LT: { // n != v
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MJitStateCpu(VFlag));
            CMP(8, MJitStateCpu(NFlag), R(tmp));
            cc = CC_E;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::GT: { // !z & (n == v)
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MJitStateCpu(NFlag));
            XOR(8, R(tmp), MJitStateCpu(VFlag));
            OR(8, R(tmp), MJitStateCpu(ZFlag));
            TEST(8, R(tmp), R(tmp));
            cc = CC_NZ;
            ReleaseTemporaryRegister(tmp);
            break;
        }
        case ConditionCode::LE: { // z | (n != v)
            X64Reg tmp = AcquireTemporaryRegister();
            MOVZX(64, 8, tmp, MJitStateCpu(NFlag));
            XOR(8, R(tmp), MJitStateCpu(VFlag));
            OR(8, R(tmp), MJitStateCpu(ZFlag));
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
    status_flag_update = false;
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

    MOV(32, R(Jit::IntToArmGPR[arm_reg]), MJitStateCpuReg(arm_reg));
    current_register_allocation.is_spilled[arm_reg] = false;

    return Jit::IntToArmGPR[arm_reg];
}

Gen::X64Reg Gen::JitCompiler::AcquireCopyOfArmRegister_with15WA(int arm_reg, unsigned inst_size) {
    if (arm_reg == 15) {
        Gen::X64Reg ret = AcquireTemporaryRegister();
        MOV(32, R(ret), Imm32(GetReg15_WordAligned(inst_size)));
        return ret;
    }

    return AcquireCopyOfArmRegister(arm_reg);
}

Gen::OpArg Gen::JitCompiler::GetArmRegisterValue(int arm_reg) {
    ASSERT(arm_reg >= 0 && arm_reg <= 14);
    if (current_register_allocation.is_spilled[arm_reg]) {
        return MJitStateCpuReg(arm_reg);
    } else {
        return R(Jit::IntToArmGPR[arm_reg]);
    }
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

    MOV(32, MJitStateCpuReg(bestreg), R(Jit::IntToArmGPR[bestreg]));

    current_register_allocation.is_spilled[bestreg] = true;
    current_register_allocation.is_in_use[bestreg] = true;

    return Jit::IntToArmGPR[bestreg];
}

Gen::X64Reg Gen::JitCompiler::AcquireCopyOfArmRegister(int arm_reg) {
    ASSERT(arm_reg >= 0 && arm_reg <= 14);

    if (!current_register_allocation.is_in_use[arm_reg]) {
        if (current_register_allocation.is_spilled[arm_reg]) {
            MOV(32, R(Jit::IntToArmGPR[arm_reg]), MJitStateCpuReg(arm_reg));
        } else {
            MOV(32, MJitStateCpuReg(arm_reg), R(Jit::IntToArmGPR[arm_reg]));
        }

        current_register_allocation.is_spilled[arm_reg] = true;
        current_register_allocation.is_in_use[arm_reg] = true;

        return Jit::IntToArmGPR[arm_reg];
    } else {
        Gen::X64Reg tmp = AcquireTemporaryRegister();

        if (current_register_allocation.is_spilled[arm_reg]) {
            MOV(32, R(tmp), MJitStateCpuReg(arm_reg));
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

Gen::X64Reg Gen::JitCompiler::EnsureTemp(Gen::X64Reg reg) {
    ASSERT(!cl_active);

    int arm_reg = WhichArmRegInNativeReg(reg);
    ASSERT(current_register_allocation.is_in_use[arm_reg]);

    if (current_register_allocation.is_spilled[arm_reg]) {
        return reg;
    }

    Gen::X64Reg ret = AcquireTemporaryRegister();
    MOV(32, R(ret), R(reg));
    return ret;
}

void Gen::JitCompiler::AcquireCLRegister(int arm_reg_to_copy) {
    ASSERT(!cl_active);
    ASSERT(cl_active_tmp == INVALID_REG);
    ASSERT(arm_reg_to_copy <= 14);
    cl_active = true;

    int arm_reg = WhichArmRegInNativeReg(RCX);
    if (!current_register_allocation.is_in_use[arm_reg]) {
        current_register_allocation.is_in_use[arm_reg] = true;

        cl_active_tmp = INVALID_REG;

        if (current_register_allocation.is_spilled[arm_reg]) {
            // HURRAH
        } else {
            MOV(32, MJitStateCpuReg(arm_reg), R(RCX));
            current_register_allocation.is_spilled[arm_reg] = true;
        }
    } else {
        if (current_register_allocation.is_spilled[arm_reg]) {
            cl_active_tmp = AcquireTemporaryRegister();
            ASSERT(cl_active_tmp != RCX);
            ASSERT(cl_active_tmp != INVALID_REG);
            MOV(32, R(cl_active_tmp), R(RCX));
        } else {
            MOV(32, MJitStateCpuReg(arm_reg), R(RCX));
            current_register_allocation.is_spilled[arm_reg] = true;
            cl_active_tmp = RCX;
        }
    }

    if (arm_reg_to_copy >= 0) {
        if (current_register_allocation.is_spilled[arm_reg_to_copy]) {
            MOV(32, R(RCX), MJitStateCpuReg(arm_reg_to_copy));
        } else if (arm_reg_to_copy != arm_reg) {
            MOV(32, R(RCX), R(Jit::IntToArmGPR[arm_reg_to_copy]));
        }
    }
}

void Gen::JitCompiler::ReleaseCLRegister() {
    ASSERT(cl_active);

    int arm_reg = WhichArmRegInNativeReg(RCX);

    if (cl_active_tmp == INVALID_REG) {
        current_register_allocation.is_in_use[arm_reg] = false;
    } else if (cl_active_tmp == RCX) {
        MOV(32, R(RCX), MJitStateCpuReg(arm_reg));
        current_register_allocation.is_spilled[arm_reg] = false;
    } else {
        MOV(32, R(RCX), R(cl_active_tmp));
    }
    cl_active = false;
    cl_active_tmp = INVALID_REG;
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
            MOV(32, MJitStateCpuReg(i), R(Jit::IntToArmGPR[i]));
            current_register_allocation.is_spilled[i] = true;
        }
    }
}

// Reset allocation MUST NOT TOUCH any conditional flags
void Gen::JitCompiler::ResetAllocation() {
    ASSERT(!cl_active);

    for (int i = 0; i < Jit::NUM_REG_GPR; i++) {
        if (current_register_allocation.is_spilled[i]) {
            MOV(32, R(Jit::IntToArmGPR[i]), MJitStateCpuReg(i));
            current_register_allocation.is_spilled[i] = false;
        }
    }

    current_register_allocation.assert_no_temporaries();
}

void Gen::JitCompiler::RestoreRSP() {
    MOV(64, R(RSP), MJitStateOther(save_host_RSP));
}

void Gen::JitCompiler::CallHostFunction(Jit::JitState*(*fn)(Jit::JitState*,u64,u64,u64), u64 a, u64 b, u64 c) {
    SpillAllRegisters();
    MOV(64, R(ABI_PARAM1), R(Jit::JitStateReg));
    MOV(64, R(ABI_PARAM2), Imm64(a));
    MOV(64, R(ABI_PARAM3), Imm64(b));
    MOV(64, R(ABI_PARAM4), Imm64(c));
    RestoreRSP();
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
                MOVZX(64, 8, ret, MJitStateCpu(CFlag));
                MOV(8, MJitStateCpu(shifter_carry_out), R(ret));
            } else {
                MOV(8, MJitStateCpu(shifter_carry_out), Imm8(BIT(shifter_operand, 31)));
            }
        }

        MOV(32, R(ret), Imm32(shifter_operand));

        return ret;
    } else if (shtop_func == DPO(Register)) {
        unsigned int rm = BITS(sht_oper, 0, 3);

        if (SCO) {
            BT(8, MJitStateCpu(CFlag), Imm8(0));
            SETcc(CC_C, MJitStateCpu(shifter_carry_out));
        }

        if (rm != 15) {
            return AcquireArmRegister(rm);
        } else {
            Gen::X64Reg ret = AcquireTemporaryRegister();
            MOV(32, R(ret), Imm32(GetReg15(inst_size)));
            return ret;
        }
    } else if (shtop_func == DPO(LogicalShiftLeftByImmediate)) {
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
                BT(8, MJitStateCpu(CFlag), Imm8(0));
                SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            }
            return Rm;
        } else {
            SHL(32, R(Rm), Imm8(shift_imm));
            if (SCO) {
                SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            }
            return Rm;
        }
    } else if (shtop_func == DPO(LogicalShiftLeftByRegister)) {
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

        if (SCO) {
            TEST(32, R(RCX), R(RCX));
            auto Rs_not_zero = J_CC(CC_NZ);

            // if (Rs & 0xFF == 0) {
            MOVZX(64, 8, RCX, MJitStateCpu(CFlag));
            MOV(8, MJitStateCpu(shifter_carry_out), R(RCX));
            auto jmp_to_end_1 = J();
            // }
            SetJumpTarget(Rs_not_zero);
            CMP(32, R(RCX), Imm8(32));
            auto Rs_gt32 = J_CC(CC_A);
            auto Rs_eq32 = J_CC(CC_E);
            // else if (Rs & 0xFF < 32) {
            SHL(32, R(Rm), R(CL));
            SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            auto jmp_to_end_2 = J();
            // }
            SetJumpTarget(Rs_gt32);
            // else if (Rs & 0xFF > 32) {
            MOV(32, R(Rm), Imm32(0));
            MOV(8, MJitStateCpu(shifter_carry_out), Imm8(0));
            auto jmp_to_end_3 = J();
            // }
            SetJumpTarget(Rs_eq32);
            // else if (Rs & 0xFF == 32) {
            BT(32, R(Rm), Imm8(0));
            SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            MOV(32, R(Rm), Imm32(0));
            // }
            SetJumpTarget(jmp_to_end_1);
            SetJumpTarget(jmp_to_end_2);
            SetJumpTarget(jmp_to_end_3);
        } else {
            CMP(32, R(RCX), Imm8(32));
            auto Rs_lt32 = J_CC(CC_B);
            // if (Rs >= 32) {
            MOV(32, R(Rm), Imm32(0));
            auto jmp_to_end = J();
            // } else {
            SetJumpTarget(Rs_lt32);
            SHL(32, R(Rm), R(CL));
            // }
            SetJumpTarget(jmp_to_end);
        }

        ReleaseCLRegister();
        return Rm;
    } else if (shtop_func == DPO(LogicalShiftRightByImmediate)) {
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
                SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            }
            MOV(64, R(Rm), Imm32(0));
            return Rm;
        } else {
            SHR(32, R(Rm), Imm8(shift_imm));
            if (SCO) {
                SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            }
            return Rm;
        }
    } else if (shtop_func == DPO(LogicalShiftRightByRegister)) {
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

        if (SCO) {
            TEST(32, R(RCX), R(RCX));
            auto Rs_not_zero = J_CC(CC_NZ);
            // if (Rs & 0xFF == 0) {
            MOVZX(64, 8, RCX, MJitStateCpu(CFlag));
            MOV(8, MJitStateCpu(shifter_carry_out), R(RCX));
            auto jmp_to_end_1 = J();
            // }
            SetJumpTarget(Rs_not_zero);
            CMP(32, R(RCX), Imm8(32));
            auto Rs_gt32 = J_CC(CC_A);
            auto Rs_eq32 = J_CC(CC_E);
            // else if (Rs & 0xFF < 32) {
            SHR(32, R(Rm), R(RCX));
            SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            auto jmp_to_end_2 = J();
            // }
            SetJumpTarget(Rs_gt32);
            // else if (Rs & 0xFF > 32) {
            MOV(32, R(Rm), Imm32(0));
            MOV(8, MJitStateCpu(shifter_carry_out), Imm8(0));
            auto jmp_to_end_3 = J();
            // }
            SetJumpTarget(Rs_eq32);
            // else if (Rs & 0xFF == 32) {
            BT(32, R(Rm), Imm8(31));
            SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            MOV(32, R(Rm), Imm32(0));
            // }
            SetJumpTarget(jmp_to_end_1);
            SetJumpTarget(jmp_to_end_2);
            SetJumpTarget(jmp_to_end_3);
        } else {
            CMP(32, R(RCX), Imm8(32));
            auto Rs_lt32 = J_CC(CC_B);
            // if (Rs & 0xFF >= 32) {
            MOV(32, R(Rm), Imm32(0));
            auto jmp_to_end = J();
            // } else {
            SetJumpTarget(Rs_lt32);
            SHR(32, R(Rm), R(RCX));
            // }
            SetJumpTarget(jmp_to_end);
        }

        ReleaseCLRegister();
        return Rm;
    } else if (shtop_func == DPO(ArithmeticShiftRightByImmediate)) {
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
                SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            }
            SAR(32, R(Rm), Imm8(31));
            return Rm;
        } else {
            SAR(32, R(Rm), Imm8(shift_imm));
            if (SCO) {
                SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            }
            return Rm;
        }
    } else if (shtop_func == DPO(ArithmeticShiftRightByRegister)) {
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

        TEST(32, R(RCX), R(RCX));
        auto Rs_not_zero = J_CC(CC_NZ);

        if (SCO) {
            // if (Rs & 0xFF == 0) {
            MOVZX(64, 8, RCX, MJitStateCpu(CFlag));
            MOV(8, MJitStateCpu(shifter_carry_out), R(RCX));
            auto jmp_to_end_1 = J();
            // }
            SetJumpTarget(Rs_not_zero);
            CMP(32, R(RCX), Imm8(31));
            auto Rs_gt31 = J_CC(CC_A);
            // else if (Rs & 0xFF < 32) {
            SAR(32, R(Rm), R(CL));
            SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            auto jmp_to_end_2 = J();
            // }
            SetJumpTarget(Rs_gt31);
            // else if (Rs & 0xFF > 31) {
            SAR(32, R(Rm), Imm8(31)); // Verified to have no incorrect counterexamples.
            BT(32, R(Rm), Imm8(31));
            SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            // }
            SetJumpTarget(jmp_to_end_1);
            SetJumpTarget(jmp_to_end_2);
        } else {
            CMP(32, R(RCX), Imm8(31));
            auto Rs_gt31 = J_CC(CC_A);
            // if (Rs & 0xFF <= 31) {
            SAR(32, R(Rm), R(CL));
            auto jmp_to_end = J();
            // } else {
            SetJumpTarget(Rs_gt31);
            SAR(32, R(Rm), Imm8(31)); // Verified to have no incorrect counterexamples.
            // }
            SetJumpTarget(jmp_to_end);
        }

        ReleaseCLRegister();
        return Rm;
    } else if (shtop_func == DPO(RotateRightByImmediate)) {
        int shift_imm = BITS(sht_oper, 7, 11);
        unsigned int rm = BITS(sht_oper, 0, 3);
        Gen::X64Reg Rm;
        if (rm != 15) Rm = AcquireCopyOfArmRegister(rm);
        else {
            Rm = AcquireTemporaryRegister();
            MOV(32, R(Rm), Imm32(GetReg15(inst_size)));
        }
        if (shift_imm == 0) { //RRX
            BT(8, MJitStateCpu(CFlag), Imm8(0));
            RCR(32, R(Rm), Imm8(1));
            if (SCO) SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            return Rm;
        } else {
            ROR(32, R(Rm), Imm8(shift_imm));
            if (SCO) SETcc(CC_C, MJitStateCpu(shifter_carry_out));
            return Rm;
        }
    } else if (shtop_func == DPO(RotateRightByRegister)) {
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

        AND(32, R(RCX), Imm32(0xFF));
        auto zero_FF = J_CC(CC_Z);
        AND(32, R(RCX), Imm32(0x1F));
        auto zero_1F = J_CC(CC_Z);
        ROR(32, R(Rm), R(CL));
        SETcc(CC_C, MJitStateCpu(shifter_carry_out));
        auto done_1 = J();

        SetJumpTarget(zero_FF);
        BT(32, MJitStateCpu(CFlag), Imm8(0));
        SETcc(CC_C, MJitStateCpu(shifter_carry_out));
        auto done_2 = J();

        SetJumpTarget(zero_1F);
        BT(32, R(Rm), Imm8(31));
        SETcc(CC_C, MJitStateCpu(shifter_carry_out));

        SetJumpTarget(done_1);
        SetJumpTarget(done_2);

        ReleaseCLRegister();
        return Rm;
    }

    ASSERT_MSG(0, "Unreachable");
    return Gen::INVALID_REG;
}

void Gen::JitCompiler::CompileMemoryRead(unsigned bits, Gen::X64Reg dest, Gen::X64Reg addr_reg, bool sign_extend) {
    // This code is very fragile. It relies on the structure of PageTable.
    // This code assumes offsetof the pointers array in the PageTable struct is 0.

    addr_reg = EnsureTemp(addr_reg);

    Gen::X64Reg page_table_reg = AcquireTemporaryRegister();
    Gen::X64Reg within_page = AcquireTemporaryRegister();

    MOV(64, R(page_table_reg), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, page_table)));

    MOV(32, R(within_page), R(addr_reg));
    AND(32, R(within_page), Imm32(Memory::PAGE_MASK));
    SHR(32, R(addr_reg), Imm8(Memory::PAGE_BITS));

    MOV(64, R(page_table_reg), MComplex(page_table_reg, addr_reg, 8, 0));

    if (bits == 64) {
        MOV(64, R(dest), MComplex(page_table_reg, within_page, 1, 0));
    } else {
        if (!sign_extend) {
            MOVZX(64, bits, dest, MComplex(page_table_reg, within_page, 1, 0));
        } else {
            MOVSX(32, bits, dest, MComplex(page_table_reg, within_page, 1, 0));
        }
    }

    ReleaseTemporaryRegister(page_table_reg);
    ReleaseTemporaryRegister(within_page);
    ReleaseTemporaryRegister(addr_reg);
}

void Gen::JitCompiler::CompileMemoryWrite(unsigned bits, Gen::X64Reg addr_reg, Gen::X64Reg src) {
    // This code is very fragile. It relies on the structure of PageTable.
    // This code assumes offsetof the pointers array in the PageTable struct is 0.

    addr_reg = EnsureTemp(addr_reg);

    Gen::X64Reg page_table_reg = AcquireTemporaryRegister();

    Gen::X64Reg within_page = AcquireTemporaryRegister();

    MOV(64, R(page_table_reg), MDisp(Jit::JitStateReg, offsetof(Jit::JitState, page_table)));

    MOV(32, R(within_page), R(addr_reg));
    AND(32, R(within_page), Imm32(Memory::PAGE_MASK));
    SHR(32, R(addr_reg), Imm8(Memory::PAGE_BITS));

    MOV(64, R(addr_reg), MComplex(page_table_reg, addr_reg, 8, 0));
    MOV(bits, MComplex(addr_reg, within_page, 1, 0), R(src));

    ReleaseTemporaryRegister(page_table_reg);
    ReleaseTemporaryRegister(within_page);
    ReleaseTemporaryRegister(addr_reg);
}

#define OPARG_SET_Z(oparg) ASSERT(status_flag_update); MOV(8, MJitStateCpu(ZFlag), (oparg))
#define OPARG_SET_C(oparg) ASSERT(status_flag_update); MOV(8, MJitStateCpu(CFlag), (oparg))
#define OPARG_SET_V(oparg) ASSERT(status_flag_update); MOV(8, MJitStateCpu(VFlag), (oparg))
#define OPARG_SET_N(oparg) ASSERT(status_flag_update); MOV(8, MJitStateCpu(NFlag), (oparg))
#define FLAG_SET_Z() ASSERT(status_flag_update); SETcc(CC_Z, MJitStateCpu(ZFlag))
#define FLAG_SET_C() ASSERT(status_flag_update); SETcc(CC_C, MJitStateCpu(CFlag))
#define FLAG_SET_V() ASSERT(status_flag_update); SETcc(CC_O, MJitStateCpu(VFlag))
#define FLAG_SET_N() ASSERT(status_flag_update); SETcc(CC_S, MJitStateCpu(NFlag))
#define FLAG_SET_C_COMPLEMENT() ASSERT(status_flag_update); SETcc(CC_NC, MJitStateCpu(CFlag))

template<typename T>
bool Gen::JitCompiler::CompileInstruction_Logical(arm_inst* inst, unsigned inst_size, void (Gen::XEmitter::*fn)(int bits, const OpArg& a1, const OpArg& a2), bool invert_operand) {
    T* const inst_cream = (T*)inst->component;

    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);
    Gen::X64Reg Rd = INVALID_REG;
    if (inst_cream->Rd != 15) Rd = AcquireArmRegister(inst_cream->Rd);
    else Rd = AcquireTemporaryRegister();

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, inst_cream->S, inst_size);
    if (invert_operand) {
        operand = EnsureTemp(operand);
        NOT(32, R(operand));
    }

    if (operand != Rd) {
        if (inst_cream->Rn == 15) {
            MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else if (Rd != Rn) {
            MOV(64, R(Rd), R(Rn));
        }
        (this->*fn)(32, R(Rd), R(operand));
    }
    else {
        if (inst_cream->Rn == 15) {
            (this->*fn)(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else {
            (this->*fn)(32, R(Rd), R(Rn));
        }
    }

    if (inst_cream->S && (inst_cream->Rd == 15)) {
        ASSERT_MSG(0, "Unimplemented");
    } else if (inst_cream->S) {
        status_flag_update = true;
        FLAG_SET_Z();
        FLAG_SET_N();
        BT(8, MJitStateCpu(shifter_carry_out), Imm8(0));
        FLAG_SET_C();
        // V is unaffected
    }

    if (inst_cream->Rd != 15) {
        ReleaseAllRegisters();
        this->pc += inst_size;
        return true;
    } else {
        MOV(32, MJitStateCpuReg(15), R(Rd));
        BT(8, R(Rd), Imm8(0));
        SETcc(CC_C, MJitStateCpu(TFlag));
        ReleaseAllRegisters();
        this->pc += inst_size;
        return CompileReturnToDispatch();
    }
}

bool Gen::JitCompiler::CompileInstruction_and(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Logical<and_inst>(inst, inst_size, &Gen::XEmitter::AND, false);
}

bool Gen::JitCompiler::CompileInstruction_eor(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Logical<eor_inst>(inst, inst_size, &Gen::XEmitter::XOR, false);
}

bool Gen::JitCompiler::CompileInstruction_orr(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Logical<orr_inst>(inst, inst_size, &Gen::XEmitter::OR, false);
}

bool Gen::JitCompiler::CompileInstruction_bic(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Logical<bic_inst>(inst, inst_size, &Gen::XEmitter::AND, true);
}

bool Gen::JitCompiler::CompileInstruction_mla(arm_inst* inst, unsigned inst_size) {
    mla_inst* const inst_cream = (mla_inst*)inst->component;

    //Spec Note: Using R15 is UNPREDICTABLE

    if (inst_cream->Rm == 15 || inst_cream->Rs == 15 || inst_cream->Rd == 15 || inst_cream->Rn == 15) {
        this->pc += inst_size;
        return true;
    }

    Gen::X64Reg Rn = AcquireArmRegister(inst_cream->Rn);
    Gen::X64Reg Rs = AcquireArmRegister(inst_cream->Rs);
    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rm = AcquireCopyOfArmRegister(inst_cream->Rm);

    IMUL(32, Rm, R(Rs));
    ADD(32, R(Rm), R(Rn));
    MOV(32, R(Rd), R(Rm));

    if (inst_cream->S) {
        status_flag_update = true;
        FLAG_SET_Z();
        FLAG_SET_N();
        // C, V are unaffected
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_mul(arm_inst* inst, unsigned inst_size) {
    mul_inst* const inst_cream = (mul_inst*)inst->component;

    //Spec Note: Using R15 is UNPREDICTABLE

    if (inst_cream->Rm == 15 || inst_cream->Rs == 15 || inst_cream->Rd == 15) {
        this->pc += inst_size;
        return true;
    }

    Gen::X64Reg Rs = AcquireArmRegister(inst_cream->Rs);
    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rm = AcquireCopyOfArmRegister(inst_cream->Rm);

    IMUL(32, Rm, R(Rs));
    MOV(32, R(Rd), R(Rm));

    if (inst_cream->S) {
        status_flag_update = true;
        TEST(32, R(Rd), R(Rd));
        FLAG_SET_Z();
        FLAG_SET_N();
        // C, V are unaffected
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_q32(arm_inst* inst, unsigned inst_size) {
    generic_arm_inst* const inst_cream = (generic_arm_inst*)inst->component;
    const u8 op1 = inst_cream->op1;
    const Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    const Gen::X64Reg Rn = AcquireCopyOfArmRegister(inst_cream->Rn);
    const Gen::X64Reg Rm = AcquireCopyOfArmRegister(inst_cream->Rm);

    ASSERT(0);

    switch (op1) {
    case 0x00: { //QADD
        MOV(32, R(Rd), R(Rm));
        SHR(32, R(Rm), Imm8(31));
        ADD(32, R(Rm), Imm32(0x7FFFFFFF));
        ADD(32, R(Rd), R(Rn));
        CMOVcc(32, Rd, R(Rm), CC_O);
        SETcc(CC_O, R(Rn));
        SHL(32, R(Rn), Imm8(27)); // Q flag
        OR(32, MJitStateCpu(Cpsr), R(Rn));
        break;
    }
    case 0x01: { //QSUB
        MOV(32, R(Rd), R(Rm));
        SHR(32, R(Rm), Imm8(31));
        ADD(32, R(Rm), Imm32(0x7FFFFFFF));
        SUB(32, R(Rd), R(Rn));
        CMOVcc(32, Rd, R(Rm), CC_O);
        SETcc(CC_O, R(Rn));
        SHL(32, R(Rn), Imm8(27)); // Q flag
        OR(32, MJitStateCpu(Cpsr), R(Rn));
        break;
    }
    case 0x02: { //QDADD
        MOV(32, R(Rd), R(Rn));
        SHR(32, R(Rn), Imm8(31));
        ADD(32, R(Rn), Imm32(0x7FFFFFFF));
        SHL(32, R(Rd), Imm8(2));
        CMOVcc(32, Rd, R(Rn), CC_C);
        SETcc(CC_C, R(Rn));
        SHL(32, R(Rn), Imm8(27)); // Q flag
        OR(32, MJitStateCpu(Cpsr), R(Rn));
        MOV(32, R(Rn), R(Rd));
        SHR(32, R(Rn), Imm8(31));
        ADD(32, R(Rn), Imm32(0x7FFFFFFF));
        ADD(32, R(Rd), R(Rm));
        CMOVcc(32, Rd, R(Rn), CC_O);
        SETcc(CC_O, R(Rn));
        SHL(32, R(Rn), Imm8(27)); // Q flag
        OR(32, MJitStateCpu(Cpsr), R(Rn));
        break;
    }
    case 0x03: { //QDSUB
        MOV(32, R(Rd), R(Rn));
        SHR(32, R(Rn), Imm8(31));
        ADD(32, R(Rn), Imm32(0x7FFFFFFF));
        SHL(32, R(Rd), Imm8(2));
        CMOVcc(32, Rd, R(Rn), CC_C);
        SETcc(CC_C, R(Rn));
        SHL(32, R(Rn), Imm8(27)); // Q flag
        OR(32, MJitStateCpu(Cpsr), R(Rn));
        MOV(32, R(Rn), R(Rd));
        SHR(32, R(Rn), Imm8(31));
        ADD(32, R(Rn), Imm32(0x7FFFFFFF));
        SUB(32, R(Rd), R(Rm));
        CMOVcc(32, Rd, R(Rn), CC_O);
        SETcc(CC_O, R(Rn));
        SHL(32, R(Rn), Imm8(27)); // Q flag
        OR(32, MJitStateCpu(Cpsr), R(Rn));
        break;
    }
    default:
        ASSERT_MSG(0, "Unreachable");
        break;
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_pkhbt(arm_inst* inst, unsigned inst_size) {
    pkh_inst* const inst_cream = (pkh_inst*)inst->component;

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rm = AcquireCopyOfArmRegister(inst_cream->Rm);
    Gen::X64Reg Rn = AcquireCopyOfArmRegister(inst_cream->Rn);

    AND(32, R(Rn), Imm32(0x0000FFFF));
    SHL(32, R(Rm), Imm8(inst_cream->imm));
    MOV(32, R(Rd), R(Rn));
    AND(32, R(Rm), Imm32(0xFFFF0000));
    OR(32, R(Rd), R(Rm));

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_pkhtb(arm_inst* inst, unsigned inst_size) {
    pkh_inst* const inst_cream = (pkh_inst*)inst->component;

    if (inst_cream->imm == 0) inst_cream->imm = 31;

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rm = AcquireCopyOfArmRegister(inst_cream->Rm);
    Gen::X64Reg Rn = AcquireCopyOfArmRegister(inst_cream->Rn);

    AND(32, R(Rn), Imm32(0xFFFF0000));
    SAR(32, R(Rm), Imm8(inst_cream->imm));
    MOV(32, R(Rd), R(Rn));
    AND(32, R(Rm), Imm32(0x0000FFFF));
    OR(32, R(Rd), R(Rm));

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

template<typename T>
bool Gen::JitCompiler::CompileInstruction_Arithmetic(arm_inst* inst, unsigned inst_size, void (Gen::XEmitter::*fn)(int bits, const OpArg& a1, const OpArg& a2), int carry, bool commutative) {
    T* const inst_cream = (T*)inst->component;

    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);
    Gen::X64Reg Rd = INVALID_REG;
    if (inst_cream->Rd != 15) Rd = AcquireArmRegister(inst_cream->Rd);

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, false, inst_size);

    if (inst_cream->Rd == 15) Rd = AcquireTemporaryRegister();

    switch (carry) {
    case 0:
    case 2:
        break;
    case 1:
        BT(32, MJitStateCpu(CFlag), Imm8(0));
        break;
    case 3:
        BT(32, MJitStateCpu(CFlag), Imm8(0));
        CMC();
        break;
    }

    if (operand != Rd) {
        if (inst_cream->Rn == 15) {
            MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else if (Rd != Rn) {
            MOV(64, R(Rd), R(Rn));
        }
        (this->*fn)(32, R(Rd), R(operand));
    } else if (commutative) {
        if (inst_cream->Rn == 15) {
            (this->*fn)(32, R(Rd), Imm32(GetReg15(inst_size)));
        } else {
            (this->*fn)(32, R(Rd), R(Rn));
        }
    } else {
        Gen::X64Reg tmp = AcquireTemporaryRegister();
        MOV(32, R(tmp), R(operand));

        if (inst_cream->Rn == 15) {
            MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
        }
        else if (Rd != Rn) {
            MOV(64, R(Rd), R(Rn));
        }
        (this->*fn)(32, R(Rd), R(tmp));

        ReleaseTemporaryRegister(tmp);
    }

    if (inst_cream->S && (inst_cream->Rd == 15)) {
        ASSERT_MSG(0, "Unimplemented");
    } else if (inst_cream->S) {
        status_flag_update = true;
        FLAG_SET_Z();
        FLAG_SET_V();
        FLAG_SET_N();

        switch (carry) {
        case 0:
        case 1:
            FLAG_SET_C();
            break;
        case 2:
        case 3:
            FLAG_SET_C_COMPLEMENT();
            break;
        }
    }

    if (inst_cream->Rd != 15) {
        ReleaseAllRegisters();
        this->pc += inst_size;
        return true;
    } else {
        MOV(32, MJitStateCpuReg(15), R(Rd));
        ReleaseAllRegisters();
        this->pc += inst_size;
        return CompileReturnToDispatch();
    }
}

bool Gen::JitCompiler::CompileInstruction_add(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Arithmetic<add_inst>(inst, inst_size, &Gen::XEmitter::ADD, 0, true);
}

bool Gen::JitCompiler::CompileInstruction_adc(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Arithmetic<adc_inst>(inst, inst_size, &Gen::XEmitter::ADC, 1, true);
}

bool Gen::JitCompiler::CompileInstruction_sub(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Arithmetic<sub_inst>(inst, inst_size, &Gen::XEmitter::SUB, 2, false);
}

bool Gen::JitCompiler::CompileInstruction_sbc(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_Arithmetic<sbc_inst>(inst, inst_size, &Gen::XEmitter::SBB, 3, false);
}

template <typename T>
bool Gen::JitCompiler::CompileInstruction_ReverseSubtraction(arm_inst* inst, unsigned inst_size, void (Gen::XEmitter::*fn)(int bits, const OpArg& a1, const OpArg& a2), bool carry) {
    T* const inst_cream = (T*)inst->component;

    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);
    Gen::X64Reg Rd = INVALID_REG;
    if (inst_cream->Rd != 15) Rd = AcquireArmRegister(inst_cream->Rd);
    else Rd = AcquireTemporaryRegister();

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, false, inst_size);

    if (carry) {
        BT(32, MJitStateCpu(CFlag), Imm8(0));
        CMC();
    }

    if (Rd != Rn) {
        if (operand != Rd) {
            MOV(32, R(Rd), R(operand));
        }
        if (inst_cream->Rn != 15) {
            (this->*fn)(32, R(Rd), R(Rn));
        } else {
            (this->*fn)(32, R(Rd), Imm32(GetReg15(inst_size)));
        }
    } else {
        Gen::X64Reg tmp = AcquireTemporaryRegister();
        MOV(32, R(tmp), R(Rn));

        if (operand != Rd) {
            MOV(32, R(Rd), R(operand));
        }

        (this->*fn)(32, R(Rd), R(tmp));

        ReleaseTemporaryRegister(tmp);
    }

    if (inst_cream->S && (inst_cream->Rd == 15)) {
        ASSERT_MSG(0, "Unimplemented");
    } else if (inst_cream->S) {
        status_flag_update = true;
        FLAG_SET_Z();
        FLAG_SET_V();
        FLAG_SET_N();
        FLAG_SET_C_COMPLEMENT();
    }

    if (inst_cream->Rd != 15) {
        ReleaseAllRegisters();
        this->pc += inst_size;
        return true;
    } else {
        MOV(32, MJitStateCpuReg(15), R(Rd));
        ReleaseAllRegisters();
        this->pc += inst_size;
        return CompileReturnToDispatch();
    }
}

bool Gen::JitCompiler::CompileInstruction_rsb(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_ReverseSubtraction<rsb_inst>(inst, inst_size, &Gen::XEmitter::SUB, false);
}

bool Gen::JitCompiler::CompileInstruction_rsc(arm_inst* inst, unsigned inst_size) {
    return CompileInstruction_ReverseSubtraction<rsc_inst>(inst, inst_size, &Gen::XEmitter::SBB, true);
}

bool Gen::JitCompiler::CompileInstruction_cmp(arm_inst* inst, unsigned inst_size) {
    cmp_inst* const inst_cream = (cmp_inst*)inst->component;

    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, false, inst_size);

    if (inst_cream->Rn != 15) {
        CMP(32, R(Rn), R(operand));
    } else {
        CMP(32, Imm32(GetReg15(inst_size)), R(operand));
    }

    status_flag_update = true;
    FLAG_SET_Z();
    FLAG_SET_C_COMPLEMENT();
    FLAG_SET_N();
    FLAG_SET_V();

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_cmn(arm_inst* inst, unsigned inst_size) {
    cmn_inst* const inst_cream = (cmn_inst*)inst->component;

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, false, inst_size);

    Gen::X64Reg Rn = INVALID_REG;
    if (inst_cream->Rn != 15) Rn = AcquireCopyOfArmRegister(inst_cream->Rn);
    else {
        Rn = AcquireTemporaryRegister();
        MOV(32, R(Rn), Imm32(GetReg15(inst_size)));
    }

    ADD(32, R(Rn), R(operand));

    status_flag_update = true;
    FLAG_SET_Z();
    FLAG_SET_C();
    FLAG_SET_N();
    FLAG_SET_V();

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_mov(arm_inst* inst, unsigned inst_size) {
    mov_inst* const inst_cream = (mov_inst*)inst->component;

    Gen::X64Reg Rd = INVALID_REG;
    if (inst_cream->Rd != 15) Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, inst_cream->S, inst_size);

    if (inst_cream->Rd != 15 && Rd != operand) MOV(32, R(Rd), R(operand));
    else if (inst_cream->Rd == 15) MOV(32, MJitStateCpuReg(15), R(operand));

    if (inst_cream->S && (inst_cream->Rd == 15)) {
        ASSERT_MSG(0, "Unimplemented");
    } else if (inst_cream->S) {
        status_flag_update = true;
        CMP(32, R(Rd), Imm32(0));
        FLAG_SET_Z();
        FLAG_SET_N();
        BT(8, MJitStateCpu(shifter_carry_out), Imm8(0));
        FLAG_SET_C();
        // V is unaffected
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    if (inst_cream->Rd == 15) {
        return CompileReturnToDispatch();
    } else {
        return true;
    }
}

bool Gen::JitCompiler::CompileInstruction_mvn(arm_inst* inst, unsigned inst_size) {
    mvn_inst* const inst_cream = (mvn_inst*)inst->component;
    if (inst_cream->Rd == 15) return CompileInstruction_Interpret(inst_size);

    Gen::X64Reg Rd = INVALID_REG;
    if (inst_cream->Rd != 15) Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, inst_cream->S, inst_size);

    if (inst_cream->Rd != 15) {
        if (Rd != operand) MOV(32, R(Rd), R(operand));
        NOT(32, R(Rd));

        if (inst_cream->S) {
            status_flag_update = true;
            CMP(32, R(Rd), Imm32(0));
            FLAG_SET_Z();
            FLAG_SET_N();
            BT(8, MJitStateCpu(shifter_carry_out), Imm8(0));
            FLAG_SET_C();
            // V is unaffected
        }

        ReleaseAllRegisters();
        this->pc += inst_size;
        return true;
    } else {
        operand = EnsureTemp(operand);
        NOT(32, R(operand));
        MOV(32, MJitStateCpuReg(15), R(operand));

        if (inst_cream->S) {
            ASSERT_MSG(0, "Unimplemented");
        }

        ReleaseAllRegisters();
        this->pc += inst_size;
        return CompileReturnToDispatch();
    }
}

bool Gen::JitCompiler::CompileInstruction_rev(arm_inst* inst, unsigned inst_size) {
    rev_inst* const inst_cream = (rev_inst*)inst->component;

    ASSERT(inst_cream->op1 == 0x03 && inst_cream->op2 == 0x01);

    Gen:X64Reg Rd = AcquireArmRegister(inst_cream->Rd);

    if (inst_cream->Rd != inst_cream->Rm) MOV(32, R(Rd), GetArmRegisterValue(inst_cream->Rm));
    BSWAP(32, Rd);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_cpy(arm_inst* inst, unsigned inst_size) {
    mov_inst* const inst_cream = (mov_inst*)inst->component;

    Gen::X64Reg Rd = INVALID_REG;
    if (inst_cream->Rd != 15) Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, inst_cream->S, inst_size);

    if (inst_cream->Rd != 15 && Rd != operand) MOV(32, R(Rd), R(operand));
    else if (inst_cream->Rd == 15) MOV(32, MJitStateCpuReg(15), R(operand));

    ReleaseAllRegisters();
    this->pc += inst_size;
    if (inst_cream->Rd == 15) {
        return CompileReturnToDispatch();
    } else {
        return true;
    }
}

bool Gen::JitCompiler::CompileInstruction_teq(arm_inst* inst, unsigned inst_size) {
    teq_inst* const inst_cream = (teq_inst*)inst->component;

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, true, inst_size);

    Gen::X64Reg Rn;
    if (inst_cream->Rn != 15) Rn = AcquireCopyOfArmRegister(inst_cream->Rn);
    else {
        Rn = AcquireTemporaryRegister();
        MOV(32, R(Rn), Imm32(GetReg15(inst_size)));
    }

    XOR(32, R(Rn), R(operand));

    status_flag_update = true;
    FLAG_SET_Z();
    FLAG_SET_N();
    BT(8, MJitStateCpu(shifter_carry_out), Imm8(0));
    FLAG_SET_C();

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_tst(arm_inst* inst, unsigned inst_size) {
    tst_inst* const inst_cream = (tst_inst*)inst->component;

    Gen::X64Reg Rn;
    if (inst_cream->Rn != 15) Rn = AcquireArmRegister(inst_cream->Rn);
    else {
        Rn = AcquireTemporaryRegister();
        MOV(32, R(Rn), Imm32(GetReg15(inst_size)));
    }

    Gen::X64Reg operand = CompileShifterOperand(inst_cream->shtop_func, inst_cream->shifter_operand, true, inst_size);

    TEST(32, R(Rn), R(operand));

    status_flag_update = true;
    FLAG_SET_Z();
    FLAG_SET_N();
    BT(8, MJitStateCpu(shifter_carry_out), Imm8(0));
    FLAG_SET_C();

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldm(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;

    Gen::X64Reg address = CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size);
    Gen::X64Reg addr_temp = INVALID_REG;
    Gen::X64Reg value = AcquireTemporaryRegister();

    ASSERT(!BIT(inst_cream->inst, 22)); // We don't support priviledged modes

    for (int i = 0; i < 16; i++) {
        if (BIT(inst_cream->inst, i)) {
            addr_temp = AcquireTemporaryRegister();
            MOV(32, R(addr_temp), R(address));
            CompileMemoryRead(32, value, addr_temp, false);

            // For armv5t, should enter thumb when bits[0] is non-zero.
            if (i == 15) {
                BT(32, R(value), Imm8(0));
                SETcc(CC_C, MJitStateCpu(TFlag));
                MOV(32, MJitStateCpuReg(15), R(value));
            } else {
                MOV(32, GetArmRegisterValue(i), R(value));
                ADD(32, R(address), Imm32(4));
            }
        }
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    if (BIT(inst_cream->inst, 15))
        return CompileReturnToDispatch();
    else
        return true;
}

bool Gen::JitCompiler::CompileInstruction_ldr(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;
    u32 Rd_num = BITS(inst_cream->inst, 12, 15);

    // ASSERT(CP15_reg1_Ubit == 0)

    if (Rd_num != 15) {
        Gen::X64Reg Rd = AcquireArmRegister(Rd_num);
        Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));
        CompileMemoryRead(32, Rd, addr, false);
        ReleaseAllRegisters();
        this->pc += inst_size;
        return true;
    } else {
        Gen::X64Reg Rd = AcquireTemporaryRegister();
        Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));
        CompileMemoryRead(32, Rd, addr, false);
        MOV(32, MJitStateCpuReg(15), R(Rd));
        ReleaseAllRegisters();
        this->pc += inst_size;
        return CompileReturnToDispatch();
    }
}

bool Gen::JitCompiler::CompileInstruction_ldrb(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;
    u32 Rd_num = BITS(inst_cream->inst, 12, 15);

    // ASSERT(CP15_reg1_Ubit == 0)
    ASSERT(Rd_num != 15); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd = AcquireArmRegister(Rd_num);
    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));
    CompileMemoryRead(8, Rd, addr, false);
    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrd(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;
    u32 Rd_num = BITS(inst_cream->inst, 12, 15);

    // Should maybe check CP15_reg1_Ubit too
    ASSERT(Rd_num < 14); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd1 = AcquireArmRegister(Rd_num);
    Gen::X64Reg Rd2 = AcquireArmRegister(Rd_num+1);
    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));
    CompileMemoryRead(64, Rd1, addr, false);
    MOV(64, R(Rd2), R(Rd1));
    SHR(64, R(Rd2), Imm8(32));
    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrexd(arm_inst* inst, unsigned inst_size) {
    generic_arm_inst* const inst_cream = (generic_arm_inst*)inst->component;

    // TODO: Exclusive mode
    ASSERT(inst_cream->Rd < 14); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd1 = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg Rd2 = AcquireArmRegister(inst_cream->Rd+1);
    Gen::X64Reg addr = AcquireCopyOfArmRegister(inst_cream->Rn);
    MOV(32, MJitStateCpu(exclusive_tag), R(addr));
    AND(32, MJitStateCpu(exclusive_tag), Imm32(ARMul_State::RESERVATION_GRANULE_MASK));
    MOV(8, MJitStateCpu(exclusive_state), Imm8(1));
    CompileMemoryRead(64, Rd1, addr, false);
    MOV(64, R(Rd2), R(Rd1));
    SHR(64, R(Rd2), Imm8(32));
    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrexb(arm_inst* inst, unsigned inst_size) {
    generic_arm_inst* const inst_cream = (generic_arm_inst*)inst->component;

    // TODO: Exclusive mode
    ASSERT(inst_cream->Rd != 15); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg addr = AcquireCopyOfArmRegister(inst_cream->Rn);
    MOV(32, MJitStateCpu(exclusive_tag), R(addr));
    AND(32, MJitStateCpu(exclusive_tag), Imm32(ARMul_State::RESERVATION_GRANULE_MASK));
    MOV(8, MJitStateCpu(exclusive_state), Imm8(1));
    CompileMemoryRead(8, Rd, addr, false);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrex(arm_inst* inst, unsigned inst_size) {
    generic_arm_inst* const inst_cream = (generic_arm_inst*)inst->component;

    // TODO: Exclusive mode
    // Should maybe check CP15_reg1_Ubit too
    ASSERT(inst_cream->Rd < 14); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg addr = AcquireCopyOfArmRegister(inst_cream->Rn);
    MOV(32, MJitStateCpu(exclusive_tag), R(addr));
    AND(32, MJitStateCpu(exclusive_tag), Imm32(ARMul_State::RESERVATION_GRANULE_MASK));
    MOV(8, MJitStateCpu(exclusive_state), Imm8(1));
    CompileMemoryRead(32, Rd, addr, false);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrexh(arm_inst* inst, unsigned inst_size) {
    generic_arm_inst* const inst_cream = (generic_arm_inst*)inst->component;

    // TODO: Exclusive mode
    ASSERT(inst_cream->Rd != 15); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd = AcquireArmRegister(inst_cream->Rd);
    Gen::X64Reg addr = AcquireCopyOfArmRegister(inst_cream->Rn);
    MOV(32, MJitStateCpu(exclusive_tag), R(addr));
    AND(32, MJitStateCpu(exclusive_tag), Imm32(ARMul_State::RESERVATION_GRANULE_MASK));
    MOV(8, MJitStateCpu(exclusive_state), Imm8(1));
    CompileMemoryRead(16, Rd, addr, false);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrh(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;
    u32 Rd_num = BITS(inst_cream->inst, 12, 15);

    // ASSERT(CP15_reg1_Ubit == 0)
    ASSERT(Rd_num != 15); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd = AcquireArmRegister(Rd_num);
    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));
    CompileMemoryRead(16, Rd, addr, false);
    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrsb(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;
    u32 Rd_num = BITS(inst_cream->inst, 12, 15);

    // ASSERT(CP15_reg1_Ubit == 0)
    ASSERT(Rd_num != 15); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd = AcquireArmRegister(Rd_num);
    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));
    CompileMemoryRead(8, Rd, addr, true);
    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_ldrsh(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;
    u32 Rd_num = BITS(inst_cream->inst, 12, 15);

    // ASSERT(CP15_reg1_Ubit == 0)
    ASSERT(Rd_num != 15); // Spec Note: UNPREDICTABLE behaviour

    Gen::X64Reg Rd = AcquireArmRegister(Rd_num);
    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));
    CompileMemoryRead(16, Rd, addr, true);
    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_stm(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;

    Gen::X64Reg address = CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size);
    Gen::X64Reg addr_temp = INVALID_REG;
    Gen::X64Reg value = AcquireTemporaryRegister();

    u32 Rn_num = BITS(inst_cream->inst, 16, 19);
    Gen::X64Reg Rn_old = AcquireCopyOfArmRegister(Rn_num);

    ASSERT(!BIT(inst_cream->inst, 22)); // We don't support priviledged modes

    // For armv5t, should enter thumb when bits[0] is non-zero.

    for (int i = 0; i < 16; i++) {
        if (BIT(inst_cream->inst, i)) {
            addr_temp = AcquireTemporaryRegister();
            MOV(32, R(addr_temp), R(address));

            if (i == 15) {
                MOV(32, R(value), Imm32(GetReg15(inst_size)));
            } else if (i == Rn_num) {
                MOV(32, R(value), R(Rn_old));
                ADD(32, R(address), Imm32(4));
            } else {
                MOV(32, R(value), GetArmRegisterValue(i));
                ADD(32, R(address), Imm32(4));
            }

            CompileMemoryWrite(32, addr_temp, value);
        }
    }

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_str(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;

    u32 Rd_num = BITS(inst_cream->inst, 12, 15);
    Gen::X64Reg Rd = INVALID_REG;
    if (Rd_num != 15) Rd = AcquireArmRegister(Rd_num);
    else {
        Rd = AcquireTemporaryRegister();
        MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
    }

    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));

    CompileMemoryWrite(32, addr, Rd);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_strb(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;

    u32 Rd_num = BITS(inst_cream->inst, 12, 15);
    Gen::X64Reg Rd = INVALID_REG;
    if (Rd_num != 15) Rd = AcquireArmRegister(Rd_num);
    else {
        Rd = AcquireTemporaryRegister();
        MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
    }

    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));

    CompileMemoryWrite(8, addr, Rd);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_strd(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;

    u32 Rd_num = BITS(inst_cream->inst, 12, 15);
    Gen::X64Reg Rd1 = INVALID_REG;
    if (Rd_num != 15) Rd1 = AcquireCopyOfArmRegister(Rd_num);

    Gen::X64Reg Rd2 = INVALID_REG;
    if ((Rd_num + 1) % 16 != 15) Rd2 = AcquireCopyOfArmRegister((Rd_num + 1) % 16);

    if (Rd_num == 15) {
        Rd1 = AcquireTemporaryRegister();
        MOV(32, R(Rd1), Imm32(GetReg15(inst_size)));
    }
    if ((Rd_num + 1) % 16 == 15) {
        Rd2 = AcquireTemporaryRegister();
        MOV(32, R(Rd2), Imm32(GetReg15(inst_size)));
    }

    SHL(64, R(Rd2), Imm8(32));
    OR(64, R(Rd1), R(Rd2));

    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));

    CompileMemoryWrite(64, addr, Rd1);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

bool Gen::JitCompiler::CompileInstruction_strh(arm_inst* inst, unsigned inst_size) {
    ldst_inst* const inst_cream = (ldst_inst*)inst->component;

    u32 Rd_num = BITS(inst_cream->inst, 12, 15);
    Gen::X64Reg Rd = INVALID_REG;
    if (Rd_num != 15) Rd = AcquireArmRegister(Rd_num);
    else {
        Rd = AcquireTemporaryRegister();
        MOV(32, R(Rd), Imm32(GetReg15(inst_size)));
    }

    Gen::X64Reg addr = EnsureTemp(CompileCalculateAddress(inst_cream->get_addr, inst_cream->inst, inst_size));

    CompileMemoryWrite(16, addr, Rd);

    ReleaseAllRegisters();
    this->pc += inst_size;
    return true;
}

void Gen::JitCompiler::CompileMaybeJumpToBB(u32 new_pc) {
    ResetAllocation();
    Gen::X64Reg tmp = AcquireTemporaryRegister();
    MOV(32, R(tmp), MJitStateOther(cycles_remaining));
    TEST(32, R(tmp), R(tmp));
    ReleaseAllRegisters();
    ResetAllocation();
    if (basic_blocks.find(new_pc) == basic_blocks.end()) {
        update_jmps[new_pc].push_back(GetWritableCodePtr());
        NOP(6); // Leave enough space for a jg instruction.
    } else {
        J_CC(CC_G, basic_blocks[new_pc], true);
    }
}

bool Gen::JitCompiler::CompileReturnToDispatch() {
    if (current_cond == ConditionCode::AL) {
        ResetAllocation();
        CompileCond(ConditionCode::AL);
        if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
        JMPptr(MJitStateOther(return_RIP));

        cycles = 0;

        return false;
    }
    else {
        ResetAllocation();
        if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
        JMPptr(MJitStateOther(return_RIP));

        CompileCond(ConditionCode::AL);
        if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
        CompileMaybeJumpToBB(this->pc);
        MOV(32, MJitStateCpuReg(15), Imm32(this->pc));
        JMPptr(MJitStateOther(return_RIP));

        cycles = 0;

        return false;
    }
}

bool Gen::JitCompiler::CompileInstruction_Branch(ConditionCode cond, u32 new_pc) {
    if (cond == ConditionCode::AL) {
        ResetAllocation();
        CompileCond(ConditionCode::AL);
        if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
        CompileMaybeJumpToBB(new_pc);
        MOV(32, MJitStateCpuReg(15), Imm32(new_pc));
        JMPptr(MJitStateOther(return_RIP));

        cycles = 0;

        return false;
    } else {
        ResetAllocation();
        if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
        CompileMaybeJumpToBB(new_pc);
        MOV(32, MJitStateCpuReg(15), Imm32(new_pc));
        JMPptr(MJitStateOther(return_RIP));

        CompileCond(ConditionCode::AL);
        if (cycles) SUB(32, MJitStateOther(cycles_remaining), Imm32(cycles));
        CompileMaybeJumpToBB(this->pc);
        MOV(32, MJitStateCpuReg(15), Imm32(this->pc));
        JMPptr(MJitStateOther(return_RIP));

        cycles = 0;

        return false;
    }
}

bool Gen::JitCompiler::CompileInstruction_bl(arm_inst* inst, unsigned inst_size) {
    bbl_inst* const inst_cream = (bbl_inst*)inst->component;

    ASSERT(!TFlag);
    u32 new_pc = pc + 8 + inst_cream->signed_immed_24;
    u32 link_pc = pc + 4;

    if (inst_cream->L) {
        Gen::X64Reg LR = AcquireArmRegister(14);
        MOV(32, R(LR), Imm32(link_pc));
    }

    ReleaseAllRegisters();
    ResetAllocation();
    this->pc += inst_size;

    return CompileInstruction_Branch((ConditionCode)inst->cond, new_pc);
}

bool Gen::JitCompiler::CompileInstruction_bx(arm_inst* inst, unsigned inst_size) {
    bx_inst* const inst_cream = (bx_inst*)inst->component;

    if (inst_cream->Rm == 15) {
        MOV(32, MJitStateCpuReg(15), Imm32(GetReg15(inst_size)));
        MOV(8, MJitStateCpu(TFlag), Imm8(0));
    } else {
        Gen::X64Reg Rm = AcquireArmRegister(inst_cream->Rm);
        MOV(32, MJitStateCpuReg(15), R(Rm));
        BT(8, R(Rm), Imm8(0));
        SETcc(CC_C, MJitStateCpu(TFlag));
    }

    this->pc += inst_size;

    ReleaseAllRegisters();

    return CompileReturnToDispatch();
}

bool Gen::JitCompiler::CompileInstruction_blx(arm_inst* inst, unsigned inst_size) {
    MOV(8, MJitStateCpu(TFlag), Imm8(1));

    blx_inst *inst_cream = (blx_inst *)inst->component;

    if (BITS(inst_cream->inst, 20, 27) == 0x12 && BITS(inst_cream->inst, 4, 7) == 0x3) {
        Gen::X64Reg Rm = AcquireArmRegister(inst_cream->val.Rm);
        Gen::X64Reg LR = AcquireArmRegister(14);

        MOV(32, R(LR), Imm32((pc + inst_size) | TFlag));
        MOV(32, MJitStateCpuReg(15), R(Rm));
        BT(32, R(Rm), Imm8(0));
        SETcc(CC_C, MJitStateCpu(TFlag));

        pc += inst_size;
        ReleaseAllRegisters();
        return CompileReturnToDispatch();
    } else {
        Gen::X64Reg LR = AcquireArmRegister(14);
        MOV(32, R(LR), Imm32(pc + inst_size));
        MOV(8, MJitStateCpu(TFlag), Imm8(1));

        int signed_int = inst_cream->val.signed_immed_24;
        signed_int = (signed_int & 0x800000) ? (0x3F000000 | signed_int) : signed_int;
        signed_int = signed_int << 2;

        u32 new_pc = pc + 8 + signed_int + (BIT(inst_cream->inst, 24) << 1);

        ReleaseAllRegisters();
        ResetAllocation();
        this->pc += inst_size;

        return CompileInstruction_Branch((ConditionCode)inst->cond, new_pc);
    }
}

bool Gen::JitCompiler::CompileInstruction_b_2_thumb(arm_inst* inst, unsigned inst_size) {
    b_2_thumb* inst_cream = (b_2_thumb*)inst->component;

    ASSERT(TFlag);
    u32 new_pc = pc + 4 + inst_cream->imm;

    ReleaseAllRegisters();
    ResetAllocation();
    this->pc += inst_size;

    return CompileInstruction_Branch((ConditionCode)inst->cond, new_pc);
}

bool Gen::JitCompiler::CompileInstruction_b_cond_thumb(arm_inst* inst, unsigned inst_size) {
    b_cond_thumb* inst_cream = (b_cond_thumb*)inst->component;

    ASSERT(TFlag);
    u32 new_pc = pc + 4 + inst_cream->imm;

    ReleaseAllRegisters();
    ResetAllocation();
    this->pc += inst_size;

    return CompileInstruction_Branch((ConditionCode)inst->cond, new_pc);
}

bool Gen::JitCompiler::CompileInstruction_bl_1_thumb(arm_inst* inst, unsigned inst_size) {
    bl_1_thumb* const inst_cream = (bl_1_thumb*)inst->component;

    ASSERT(TFlag);
    u32 link_pc = pc + 4 + inst_cream->imm;

    Gen::X64Reg LR = AcquireArmRegister(14);
    MOV(32, R(LR), Imm32(link_pc));

    ReleaseAllRegisters();
    ResetAllocation();
    this->pc += inst_size;

    return true;
}

bool Gen::JitCompiler::CompileInstruction_bl_2_thumb(arm_inst* inst, unsigned inst_size) {
    bl_2_thumb* const inst_cream = (bl_2_thumb*)inst->component;

    ASSERT(TFlag);
    u32 link_pc = (pc + 2) | 1;

    Gen::X64Reg LR = AcquireArmRegister(14);
    MOV(32, MJitStateCpuReg(15), R(LR));
    ADD(32, MJitStateCpuReg(15), Imm32(inst_cream->imm));
    MOV(32, R(LR), Imm32(link_pc));

    ReleaseAllRegisters();
    ResetAllocation();
    this->pc += inst_size;

    return CompileReturnToDispatch();
}

bool Gen::JitCompiler::CompileInstruction_blx_1_thumb(arm_inst* inst, unsigned inst_size) {
    blx_1_thumb* inst_cream = (blx_1_thumb*)inst->component;

    ASSERT(TFlag);
    u32 link_pc = (pc + 2) | 1;

    Gen::X64Reg LR = AcquireArmRegister(14);
    MOV(32, MJitStateCpuReg(15), R(LR));
    ADD(32, MJitStateCpuReg(15), Imm32(inst_cream->imm));
    MOV(32, R(LR), Imm32(link_pc));
    MOV(32, MJitStateCpu(TFlag), Imm8(0));

    ReleaseAllRegisters();
    ResetAllocation();
    this->pc += inst_size;

    return CompileReturnToDispatch();
}

Gen::X64Reg Gen::JitCompiler::CompileCalculateAddress(get_addr_fp_t addr_func, u32 inst, unsigned inst_size) {
    u32 u_bit = BIT(inst, 23);
    u32 Rn_num = BITS(inst, 16, 19);

    u32 Rm_num = BITS(inst, 0, 3);

    u32 immed12 = BITS(inst, 0, 11);

    u32 offset_8 = (BITS(inst, 8, 11) << 4) | BITS(inst, 0, 3);

    auto calc_scale = [this, Rm_num](u32 shift, u32 shift_imm) -> Gen::X64Reg {
        Gen::X64Reg index = INVALID_REG;
        switch (shift) {
        case 0: { //LSL
            index = AcquireCopyOfArmRegister(Rm_num);
            SHL(32, R(index), Imm8(shift_imm));
            break;
        }
        case 1: { //LSR
            if (shift_imm == 0) { // LSR #32
                index = AcquireTemporaryRegister();
                MOV(32, R(index), Imm32(0));
            } else {
                index = AcquireCopyOfArmRegister(Rm_num);
                SHR(32, R(index), Imm8(shift_imm));
            }
            break;
        }
        case 2: { //ASR
            index = AcquireCopyOfArmRegister(Rm_num);
            if (shift_imm == 0) shift_imm = 31; // ASR #32
            SAR(32, R(index), Imm8(shift_imm));
            break;
        }
        case 3: {
            index = AcquireCopyOfArmRegister(Rm_num);
            if (shift_imm == 0) { // RRX
                BT(8, MJitStateCpu(CFlag), Imm8(0));
                RCR(32, R(index), Imm8(1));
            } else { // ROR
                ROR(32, R(index), Imm8(shift_imm));
            }
            break;
        default:
            ASSERT_MSG(0, "Unreachable");
        }
        }

        return index;
    };

    if (addr_func == LnSWoUB(ImmediateOffset)) {
        Gen::X64Reg ret = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
        if (u_bit) {
            ADD(32, R(ret), Imm32(immed12));
        } else {
            SUB(32, R(ret), Imm32(immed12));
        }
        return ret;
    } else if (addr_func == LnSWoUB(RegisterOffset)) {
        // Spec Note: Specifying Rm as R15 has UNPREDICTABLE results.
        ASSERT(Rm_num != 15);
        u32 Rm_num = BITS(inst, 0, 3);
        Gen::X64Reg Rn_copy = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
        if (u_bit) {
            ADD(32, R(Rn_copy), GetArmRegisterValue(Rm_num));
        } else {
            SUB(32, R(Rn_copy), GetArmRegisterValue(Rm_num));
        }
        return Rn_copy;
    } else if (addr_func == LnSWoUB(ImmediatePostIndexed)) {
        // Spec Note: Specifying R15 as Rn has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(BITS(inst, 28, 31) == current_cond);
        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        Gen::X64Reg ret = AcquireCopyOfArmRegister(Rn_num);
        if (u_bit) {
            ADD(32, R(Rn), Imm32(immed12));
        } else {
            SUB(32, R(Rn), Imm32(immed12));
        }
        return ret;
    } else if (addr_func == LnSWoUB(ImmediatePreIndexed)) {
        // Spec Note: Specifying R15 as Rn has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(BITS(inst, 28, 31) == current_cond);
        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        if (u_bit) {
            ADD(32, R(Rn), Imm32(immed12));
        } else {
            SUB(32, R(Rn), Imm32(immed12));
        }
        return EnsureTemp(Rn);
    } else if (addr_func == LnSWoUB(RegisterPreIndexed)) {
        // Spec Note: Specifying R15 as Rn or Rm has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(Rm_num != 15);
        ASSERT(current_cond == BITS(inst, 28, 31));
        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        if (u_bit) {
            ADD(32, R(Rn), GetArmRegisterValue(Rm_num));
        } else {
            SUB(32, R(Rn), GetArmRegisterValue(Rm_num));
        }
        return EnsureTemp(Rn);
    } else if (addr_func == LnSWoUB(ScaledRegisterPreIndexed)) {
        // Spec Note: Specifying R15 as Rn or Rm has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(Rm_num != 15);
        ASSERT(current_cond == BITS(inst, 28, 31));

        u32 shift = BITS(inst, 5, 6);
        u32 shift_imm = BITS(inst, 7, 11);

        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        Gen::X64Reg index = calc_scale(shift, shift_imm);

        if (u_bit) {
            ADD(32, R(Rn), R(index));
        } else {
            SUB(32, R(Rn), R(index));
        }

        return EnsureTemp(Rn);
    } else if (addr_func == LnSWoUB(ScaledRegisterPostIndexed)) {
        // Spec Note: Specifying R15 as Rn or Rm has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(Rm_num != 15);
        ASSERT(current_cond == BITS(inst, 28, 31));

        unsigned int shift = BITS(inst, 5, 6);
        unsigned int shift_imm = BITS(inst, 7, 11);

        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        Gen::X64Reg ret = AcquireCopyOfArmRegister(Rn_num);
        Gen::X64Reg index = calc_scale(shift, shift_imm);

        if (u_bit) {
            ADD(32, R(Rn), R(index));
        } else {
            SUB(32, R(Rn), R(index));
        }

        return ret;
    } else if (addr_func == LnSWoUB(RegisterPostIndexed)) {
        // Spec Note: Specifying R15 as Rn or Rm has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(Rm_num != 15);
        ASSERT(current_cond == BITS(inst, 28, 31));

        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        Gen::X64Reg ret = AcquireCopyOfArmRegister(Rn_num);

        if (u_bit) {
            ADD(32, R(Rn), GetArmRegisterValue(Rm_num));
        } else {
            SUB(32, R(Rn), GetArmRegisterValue(Rm_num));
        }

        return ret;
    } else if (addr_func == LnSWoUB(ScaledRegisterOffset)) {
        // Spec Note: Specifying R15 as Rn or Rm has UNPREDICTABLE results.
        ASSERT(Rm_num != 15);
        ASSERT(current_cond == BITS(inst, 28, 31));

        unsigned int shift = BITS(inst, 5, 6);
        unsigned int shift_imm = BITS(inst, 7, 11);

        Gen::X64Reg ret = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
        Gen::X64Reg index = calc_scale(shift, shift_imm);

        if (u_bit) {
            ADD(32, R(ret), R(index));
        } else {
            SUB(32, R(ret), R(index));
        }

        return ret;
    } else if (addr_func == MLnS(ImmediateOffset)) {
        Gen::X64Reg ret = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
        if (u_bit) {
            ADD(32, R(ret), Imm32(offset_8));
        } else {
            SUB(32, R(ret), Imm32(offset_8));
        }
        return ret;
    } else if (addr_func == MLnS(RegisterOffset)) {
        // Spec Note: Specifying Rm as R15 has UNPREDICTABLE results.
        ASSERT(Rm_num != 15);
        u32 Rm_num = BITS(inst, 0, 3);
        Gen::X64Reg Rn_copy = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
        if (u_bit) {
            ADD(32, R(Rn_copy), GetArmRegisterValue(Rm_num));
        } else {
            SUB(32, R(Rn_copy), GetArmRegisterValue(Rm_num));
        }
        return Rn_copy;
    } else if (addr_func == MLnS(ImmediatePreIndexed)) {
        // Spec Note: Specifying R15 as Rn has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(BITS(inst, 28, 31) == current_cond);
        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        if (u_bit) {
            ADD(32, R(Rn), Imm32(offset_8));
        } else {
            SUB(32, R(Rn), Imm32(offset_8));
        }
        return EnsureTemp(Rn);
    } else if (addr_func == MLnS(ImmediatePostIndexed)) {
        // Spec Note: Specifying R15 as Rn has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(BITS(inst, 28, 31) == current_cond);
        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        Gen::X64Reg ret = AcquireCopyOfArmRegister(Rn_num);
        if (u_bit) {
            ADD(32, R(Rn), Imm32(offset_8));
        } else {
            SUB(32, R(Rn), Imm32(offset_8));
        }
        return ret;
    } else if (addr_func == MLnS(RegisterPreIndexed)) {
        // Spec Note: Specifying R15 as Rn or Rm has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(Rm_num != 15);
        ASSERT(current_cond == BITS(inst, 28, 31));
        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        if (u_bit) {
            ADD(32, R(Rn), GetArmRegisterValue(Rm_num));
        } else {
            SUB(32, R(Rn), GetArmRegisterValue(Rm_num));
        }
        return EnsureTemp(Rn);
    } else if (addr_func == MLnS(RegisterPostIndexed)) {
        // Spec Note: Specifying R15 as Rn or Rm has UNPREDICTABLE results.
        ASSERT(Rn_num != 15);
        ASSERT(Rm_num != 15);
        ASSERT(current_cond == BITS(inst, 28, 31));

        Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
        Gen::X64Reg ret = AcquireCopyOfArmRegister(Rn_num);

        if (u_bit) {
            ADD(32, R(Rn), GetArmRegisterValue(Rm_num));
        } else {
            SUB(32, R(Rn), GetArmRegisterValue(Rm_num));
        }

        return ret;
    } else if (addr_func == LdnStM(DecrementBefore)) {
        ASSERT(current_cond == BITS(inst, 28, 31));
        int count = 0;

        unsigned int i = BITS(inst, 0, 15);
        while (i) {
            if (i & 1) count++;
            i = i >> 1;
        }

        if (BIT(inst, 21) && Rn_num != 15) {
            Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
            SUB(32, R(Rn), Imm32(4 * count));
            return AcquireCopyOfArmRegister(Rn_num);
        } if (BIT(inst, 21) && Rn_num == 15) {
            ASSERT_MSG(0, "Is this ever actually used anywhere?");
            Gen::X64Reg ret = AcquireTemporaryRegister();
            MOV(32, R(ret), Imm32(GetReg15_WordAligned(inst_size) - 4 * count));
            MOV(32, MJitStateCpuReg(15), Imm32(GetReg15_WordAligned(inst_size) - 4 * count));
            return ret;
        } else {
            Gen::X64Reg Rn = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
            SUB(32, R(Rn), Imm32(4 * count));
            return Rn;
        }
    } else if (addr_func == LdnStM(IncrementBefore)) {
        ASSERT(current_cond == BITS(inst, 28, 31));
        int count = 0;

        unsigned int i = BITS(inst, 0, 15);
        while (i) {
            if (i & 1) count++;
            i = i >> 1;
        }

        if (BIT(inst, 21) && Rn_num != 15) {
            Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
            Gen::X64Reg ret = AcquireCopyOfArmRegister(Rn_num);
            ADD(32, R(ret), Imm32(4));
            ADD(32, R(Rn), Imm32(4 * count));
            return ret;
        } if (BIT(inst, 21) && Rn_num == 15) {
            ASSERT_MSG(0, "Is this ever actually used anywhere?");
            Gen::X64Reg ret = AcquireTemporaryRegister();
            MOV(32, R(ret), Imm32(GetReg15_WordAligned(inst_size) + 4));
            MOV(32, MJitStateCpuReg(15), Imm32(GetReg15_WordAligned(inst_size) + 4 * count));
            return ret;
        } else {
            Gen::X64Reg Rn = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
            ADD(32, R(Rn), Imm32(4));
            return Rn;
        }
    } else if (addr_func == LdnStM(IncrementAfter)) {
        ASSERT(current_cond == BITS(inst, 28, 31));
        int count = 0;

        unsigned int i = BITS(inst, 0, 15);
        while (i) {
            if (i & 1) count++;
            i = i >> 1;
        }

        if (BIT(inst, 21) && Rn_num != 15) {
            Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
            Gen::X64Reg ret = AcquireCopyOfArmRegister(Rn_num);
            ADD(32, R(Rn), Imm32(4 * count));
            return ret;
        } if (BIT(inst, 21) && Rn_num == 15) {
            ASSERT_MSG(0, "Is this ever actually used anywhere?");
            Gen::X64Reg ret = AcquireTemporaryRegister();
            MOV(32, R(ret), Imm32(GetReg15_WordAligned(inst_size)));
            MOV(32, MJitStateCpuReg(15), Imm32(GetReg15_WordAligned(inst_size) + 4 * count));
            return ret;
        } else {
            return AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
        }
    } else if (addr_func == LdnStM(DecrementAfter)) {
        ASSERT(current_cond == BITS(inst, 28, 31));
        int count = 0;

        unsigned int i = BITS(inst, 0, 15);
        while (i) {
            if (i & 1) count++;
            i = i >> 1;
        }

        if (BIT(inst, 21) && Rn_num != 15) {
            Gen::X64Reg Rn = AcquireArmRegister(Rn_num);
            SUB(32, R(Rn), Imm32(4 * count));
            Gen::X64Reg temp = AcquireCopyOfArmRegister(Rn_num);
            ADD(32, R(temp), Imm32(4));
            return temp;
        } if (BIT(inst, 21) && Rn_num == 15) {
            ASSERT_MSG(0, "Is this ever actually used anywhere?");
            Gen::X64Reg ret = AcquireTemporaryRegister();
            MOV(32, R(ret), Imm32(GetReg15_WordAligned(inst_size) - 4 * count + 4));
            MOV(32, MJitStateCpuReg(15), Imm32(GetReg15_WordAligned(inst_size) - 4 * count));
            return ret;
        } else {
            Gen::X64Reg ret = AcquireCopyOfArmRegister_with15WA(Rn_num, inst_size);
            SUB(32, R(ret), Imm32(4 * count - 4));
            return ret;
        }
    }

    ASSERT_MSG(0, "Unreachable");
    return INVALID_REG;
}
