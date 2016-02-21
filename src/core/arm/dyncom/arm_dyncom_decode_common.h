#define DPO(s)                glue(DataProcessingOperands, s)

typedef unsigned int (*shtop_fp_t)(ARMul_State* cpu, unsigned int sht_oper);

static unsigned int DPO(Immediate)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int immed_8 = BITS(sht_oper, 0, 7);
    unsigned int rotate_imm = BITS(sht_oper, 8, 11);
    unsigned int shifter_operand = ROTATE_RIGHT_32(immed_8, rotate_imm * 2);
    if (rotate_imm == 0)
        cpu->shifter_carry_out = cpu->CFlag;
    else
        cpu->shifter_carry_out = BIT(shifter_operand, 31);
    return shifter_operand;
}

static unsigned int DPO(Register)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int shifter_operand = rm;
    cpu->shifter_carry_out = cpu->CFlag;
    return shifter_operand;
}

static unsigned int DPO(LogicalShiftLeftByImmediate)(ARMul_State* cpu, unsigned int sht_oper) {
    int shift_imm = BITS(sht_oper, 7, 11);
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int shifter_operand;
    if (shift_imm == 0) {
        shifter_operand = rm;
        cpu->shifter_carry_out = cpu->CFlag;
    } else {
        shifter_operand = rm << shift_imm;
        cpu->shifter_carry_out = BIT(rm, 32 - shift_imm);
    }
    return shifter_operand;
}

static unsigned int DPO(LogicalShiftLeftByRegister)(ARMul_State* cpu, unsigned int sht_oper) {
    int shifter_operand;
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int rs = CHECK_READ_REG15(cpu, RS);
    if (BITS(rs, 0, 7) == 0) {
        shifter_operand = rm;
        cpu->shifter_carry_out = cpu->CFlag;
    } else if (BITS(rs, 0, 7) < 32) {
        shifter_operand = rm << BITS(rs, 0, 7);
        cpu->shifter_carry_out = BIT(rm, 32 - BITS(rs, 0, 7));
    } else if (BITS(rs, 0, 7) == 32) {
        shifter_operand = 0;
        cpu->shifter_carry_out = BIT(rm, 0);
    } else {
        shifter_operand = 0;
        cpu->shifter_carry_out = 0;
    }
    return shifter_operand;
}

static unsigned int DPO(LogicalShiftRightByImmediate)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int shifter_operand;
    int shift_imm = BITS(sht_oper, 7, 11);
    if (shift_imm == 0) {
        shifter_operand = 0;
        cpu->shifter_carry_out = BIT(rm, 31);
    } else {
        shifter_operand = rm >> shift_imm;
        cpu->shifter_carry_out = BIT(rm, shift_imm - 1);
    }
    return shifter_operand;
}

static unsigned int DPO(LogicalShiftRightByRegister)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int rs = CHECK_READ_REG15(cpu, RS);
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int shifter_operand;
    if (BITS(rs, 0, 7) == 0) {
        shifter_operand = rm;
        cpu->shifter_carry_out = cpu->CFlag;
    } else if (BITS(rs, 0, 7) < 32) {
        shifter_operand = rm >> BITS(rs, 0, 7);
        cpu->shifter_carry_out = BIT(rm, BITS(rs, 0, 7) - 1);
    } else if (BITS(rs, 0, 7) == 32) {
        shifter_operand = 0;
        cpu->shifter_carry_out = BIT(rm, 31);
    } else {
        shifter_operand = 0;
        cpu->shifter_carry_out = 0;
    }
    return shifter_operand;
}

static unsigned int DPO(ArithmeticShiftRightByImmediate)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int shifter_operand;
    int shift_imm = BITS(sht_oper, 7, 11);
    if (shift_imm == 0) {
        if (BIT(rm, 31) == 0)
            shifter_operand = 0;
        else
            shifter_operand = 0xFFFFFFFF;
        cpu->shifter_carry_out = BIT(rm, 31);
    } else {
        shifter_operand = static_cast<int>(rm) >> shift_imm;
        cpu->shifter_carry_out = BIT(rm, shift_imm - 1);
    }
    return shifter_operand;
}

static unsigned int DPO(ArithmeticShiftRightByRegister)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int rs = CHECK_READ_REG15(cpu, RS);
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int shifter_operand;
    if (BITS(rs, 0, 7) == 0) {
        shifter_operand = rm;
        cpu->shifter_carry_out = cpu->CFlag;
    } else if (BITS(rs, 0, 7) < 32) {
        shifter_operand = static_cast<int>(rm) >> BITS(rs, 0, 7);
        cpu->shifter_carry_out = BIT(rm, BITS(rs, 0, 7) - 1);
    } else {
        if (BIT(rm, 31) == 0)
            shifter_operand = 0;
        else
            shifter_operand = 0xffffffff;
        cpu->shifter_carry_out = BIT(rm, 31);
    }
    return shifter_operand;
}

static unsigned int DPO(RotateRightByImmediate)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int shifter_operand;
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    int shift_imm = BITS(sht_oper, 7, 11);
    if (shift_imm == 0) {
        shifter_operand = (cpu->CFlag << 31) | (rm >> 1);
        cpu->shifter_carry_out = BIT(rm, 0);
    } else {
        shifter_operand = ROTATE_RIGHT_32(rm, shift_imm);
        cpu->shifter_carry_out = BIT(rm, shift_imm - 1);
    }
    return shifter_operand;
}

static unsigned int DPO(RotateRightByRegister)(ARMul_State* cpu, unsigned int sht_oper) {
    unsigned int rm = CHECK_READ_REG15(cpu, RM);
    unsigned int rs = CHECK_READ_REG15(cpu, RS);
    unsigned int shifter_operand;
    if (BITS(rs, 0, 7) == 0) {
        shifter_operand = rm;
        cpu->shifter_carry_out = cpu->CFlag;
    } else if (BITS(rs, 0, 4) == 0) {
        shifter_operand = rm;
        cpu->shifter_carry_out = BIT(rm, 31);
    } else {
        shifter_operand = ROTATE_RIGHT_32(rm, BITS(rs, 0, 4));
        cpu->shifter_carry_out = BIT(rm, BITS(rs, 0, 4) - 1);
    }
    return shifter_operand;
}

typedef void (*get_addr_fp_t)(ARMul_State *cpu, unsigned int inst, unsigned int &virt_addr);

struct ldst_inst {
    unsigned int inst;
    get_addr_fp_t get_addr;
};
#define DEBUG_MSG LOG_DEBUG(Core_ARM11, "inst is %x", inst); CITRA_IGNORE_EXIT(0)

#define LnSWoUB(s)   glue(LnSWoUB, s)
#define MLnS(s)      glue(MLnS, s)
#define LdnStM(s)    glue(LdnStM, s)

#define W_BIT        BIT(inst, 21)
#define U_BIT        BIT(inst, 23)
#define I_BIT        BIT(inst, 25)
#define P_BIT        BIT(inst, 24)
#define OFFSET_12    BITS(inst, 0, 11)

static void LnSWoUB(ImmediateOffset)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int addr;

    if (U_BIT)
        addr = CHECK_READ_REG15_WA(cpu, Rn) + OFFSET_12;
    else
        addr = CHECK_READ_REG15_WA(cpu, Rn) - OFFSET_12;

    virt_addr = addr;
}

static void LnSWoUB(RegisterOffset)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst, 0, 3);
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);
    unsigned int addr;

    if (U_BIT)
        addr = rn + rm;
    else
        addr = rn - rm;

    virt_addr = addr;
}

static void LnSWoUB(ImmediatePostIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int addr = CHECK_READ_REG15_WA(cpu, Rn);

    if (U_BIT)
        cpu->Reg[Rn] += OFFSET_12;
    else
        cpu->Reg[Rn] -= OFFSET_12;

    virt_addr = addr;
}

static void LnSWoUB(ImmediatePreIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int addr;

    if (U_BIT)
        addr = CHECK_READ_REG15_WA(cpu, Rn) + OFFSET_12;
    else
        addr = CHECK_READ_REG15_WA(cpu, Rn) - OFFSET_12;

    virt_addr = addr;

    if (CondPassed(cpu, BITS(inst, 28, 31)))
        cpu->Reg[Rn] = addr;
}

static void MLnS(RegisterPreIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int addr;
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst,  0,  3);
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);

    if (U_BIT)
        addr = rn + rm;
    else
        addr = rn - rm;

    virt_addr = addr;

    if (CondPassed(cpu, BITS(inst, 28, 31)))
        cpu->Reg[Rn] = addr;
}

static void LnSWoUB(RegisterPreIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst, 0, 3);
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);
    unsigned int addr;

    if (U_BIT)
        addr = rn + rm;
    else
        addr = rn - rm;

    virt_addr = addr;

    if (CondPassed(cpu, BITS(inst, 28, 31))) {
        cpu->Reg[Rn] = addr;
    }
}

static void LnSWoUB(ScaledRegisterPreIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int shift = BITS(inst, 5, 6);
    unsigned int shift_imm = BITS(inst, 7, 11);
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst, 0, 3);
    unsigned int index = 0;
    unsigned int addr;
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);

    switch (shift) {
    case 0:
        index = rm << shift_imm;
        break;
    case 1:
        if (shift_imm == 0) {
            index = 0;
        } else {
            index = rm >> shift_imm;
        }
        break;
    case 2:
        if (shift_imm == 0) { // ASR #32
            if (BIT(rm, 31) == 1)
                index = 0xFFFFFFFF;
            else
                index = 0;
        } else {
            index = static_cast<int>(rm) >> shift_imm;
        }
        break;
    case 3:
        if (shift_imm == 0) {
            index = (cpu->CFlag << 31) | (rm >> 1);
        } else {
            index = ROTATE_RIGHT_32(rm, shift_imm);
        }
        break;
    }

    if (U_BIT)
        addr = rn + index;
    else
        addr = rn - index;

    virt_addr = addr;

    if (CondPassed(cpu, BITS(inst, 28, 31)))
        cpu->Reg[Rn] = addr;
}

static void LnSWoUB(ScaledRegisterPostIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int shift = BITS(inst, 5, 6);
    unsigned int shift_imm = BITS(inst, 7, 11);
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst, 0, 3);
    unsigned int index = 0;
    unsigned int addr = CHECK_READ_REG15_WA(cpu, Rn);
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);

    switch (shift) {
    case 0:
        index = rm << shift_imm;
        break;
    case 1:
        if (shift_imm == 0) {
            index = 0;
        } else {
            index = rm >> shift_imm;
        }
        break;
    case 2:
        if (shift_imm == 0) { // ASR #32
            if (BIT(rm, 31) == 1)
                index = 0xFFFFFFFF;
            else
                index = 0;
        } else {
            index = static_cast<int>(rm) >> shift_imm;
        }
        break;
    case 3:
        if (shift_imm == 0) {
            index = (cpu->CFlag << 31) | (rm >> 1);
        } else {
            index = ROTATE_RIGHT_32(rm, shift_imm);
        }
        break;
    }

    virt_addr = addr;

    if (CondPassed(cpu, BITS(inst, 28, 31))) {
        if (U_BIT)
            cpu->Reg[Rn] += index;
        else
            cpu->Reg[Rn] -= index;
    }
}

static void LnSWoUB(RegisterPostIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst,  0,  3);
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);

    virt_addr = CHECK_READ_REG15_WA(cpu, Rn);

    if (CondPassed(cpu, BITS(inst, 28, 31))) {
        if (U_BIT) {
            cpu->Reg[Rn] += rm;
        } else {
            cpu->Reg[Rn] -= rm;
        }
    }
}

static void MLnS(ImmediateOffset)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int immedL = BITS(inst, 0, 3);
    unsigned int immedH = BITS(inst, 8, 11);
    unsigned int Rn     = BITS(inst, 16, 19);
    unsigned int addr;

    unsigned int offset_8 = (immedH << 4) | immedL;

    if (U_BIT)
        addr = CHECK_READ_REG15_WA(cpu, Rn) + offset_8;
    else
        addr = CHECK_READ_REG15_WA(cpu, Rn) - offset_8;

    virt_addr = addr;
}

static void MLnS(RegisterOffset)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int addr;
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst,  0,  3);
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);

    if (U_BIT)
        addr = rn + rm;
    else
        addr = rn - rm;

    virt_addr = addr;
}

static void MLnS(ImmediatePreIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn     = BITS(inst, 16, 19);
    unsigned int immedH = BITS(inst,  8, 11);
    unsigned int immedL = BITS(inst,  0,  3);
    unsigned int addr;
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);
    unsigned int offset_8 = (immedH << 4) | immedL;

    if (U_BIT)
        addr = rn + offset_8;
    else
        addr = rn - offset_8;

    virt_addr = addr;

    if (CondPassed(cpu, BITS(inst, 28, 31)))
        cpu->Reg[Rn] = addr;
}

static void MLnS(ImmediatePostIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn     = BITS(inst, 16, 19);
    unsigned int immedH = BITS(inst,  8, 11);
    unsigned int immedL = BITS(inst,  0,  3);
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);

    virt_addr = rn;

    if (CondPassed(cpu, BITS(inst, 28, 31))) {
        unsigned int offset_8 = (immedH << 4) | immedL;
        if (U_BIT)
            rn += offset_8;
        else
            rn -= offset_8;

        cpu->Reg[Rn] = rn;
    }
}

static void MLnS(RegisterPostIndexed)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst,  0,  3);
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);

    virt_addr = CHECK_READ_REG15_WA(cpu, Rn);

    if (CondPassed(cpu, BITS(inst, 28, 31))) {
        if (U_BIT)
            cpu->Reg[Rn] += rm;
        else
            cpu->Reg[Rn] -= rm;
    }
}

static void LdnStM(DecrementBefore)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int i = BITS(inst, 0, 15);
    int count = 0;

    while (i) {
        if (i & 1) count++;
        i = i >> 1;
    }

    virt_addr = CHECK_READ_REG15_WA(cpu, Rn) - count * 4;

    if (CondPassed(cpu, BITS(inst, 28, 31)) && BIT(inst, 21))
        cpu->Reg[Rn] -= count * 4;
}

static void LdnStM(IncrementBefore)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int i = BITS(inst, 0, 15);
    int count = 0;

    while (i) {
        if (i & 1) count++;
        i = i >> 1;
    }

    virt_addr = CHECK_READ_REG15_WA(cpu, Rn) + 4;

    if (CondPassed(cpu, BITS(inst, 28, 31)) && BIT(inst, 21))
        cpu->Reg[Rn] += count * 4;
}

static void LdnStM(IncrementAfter)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int i = BITS(inst, 0, 15);
    int count = 0;

    while(i) {
        if (i & 1) count++;
        i = i >> 1;
    }

    virt_addr = CHECK_READ_REG15_WA(cpu, Rn);

    if (CondPassed(cpu, BITS(inst, 28, 31)) && BIT(inst, 21))
        cpu->Reg[Rn] += count * 4;
}

static void LdnStM(DecrementAfter)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int i = BITS(inst, 0, 15);
    int count = 0;
    while(i) {
        if(i & 1) count++;
        i = i >> 1;
    }
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);
    unsigned int start_addr = rn - count * 4 + 4;

    virt_addr = start_addr;

    if (CondPassed(cpu, BITS(inst, 28, 31)) && BIT(inst, 21)) {
        cpu->Reg[Rn] -= count * 4;
    }
}

static void LnSWoUB(ScaledRegisterOffset)(ARMul_State* cpu, unsigned int inst, unsigned int& virt_addr) {
    unsigned int shift = BITS(inst, 5, 6);
    unsigned int shift_imm = BITS(inst, 7, 11);
    unsigned int Rn = BITS(inst, 16, 19);
    unsigned int Rm = BITS(inst, 0, 3);
    unsigned int index = 0;
    unsigned int addr;
    unsigned int rm = CHECK_READ_REG15_WA(cpu, Rm);
    unsigned int rn = CHECK_READ_REG15_WA(cpu, Rn);

    switch (shift) {
    case 0:
        index = rm << shift_imm;
        break;
    case 1:
        if (shift_imm == 0) {
            index = 0;
        } else {
            index = rm >> shift_imm;
        }
        break;
    case 2:
        if (shift_imm == 0) { // ASR #32
            if (BIT(rm, 31) == 1)
                index = 0xFFFFFFFF;
            else
                index = 0;
        } else {
            index = static_cast<int>(rm) >> shift_imm;
        }
        break;
    case 3:
        if (shift_imm == 0) {
            index = (cpu->CFlag << 31) | (rm >> 1);
        } else {
            index = ROTATE_RIGHT_32(rm, shift_imm);
        }
        break;
    }

    if (U_BIT) {
        addr = rn + index;
    } else
        addr = rn - index;

    virt_addr = addr;
}

static shtop_fp_t get_shtop(unsigned int inst) {
    if (BIT(inst, 25)) {
        return DPO(Immediate);
    } else if (BITS(inst, 4, 11) == 0) {
        return DPO(Register);
    } else if (BITS(inst, 4, 6) == 0) {
        return DPO(LogicalShiftLeftByImmediate);
    } else if (BITS(inst, 4, 7) == 1) {
        return DPO(LogicalShiftLeftByRegister);
    } else if (BITS(inst, 4, 6) == 2) {
        return DPO(LogicalShiftRightByImmediate);
    } else if (BITS(inst, 4, 7) == 3) {
        return DPO(LogicalShiftRightByRegister);
    } else if (BITS(inst, 4, 6) == 4) {
        return DPO(ArithmeticShiftRightByImmediate);
    } else if (BITS(inst, 4, 7) == 5) {
        return DPO(ArithmeticShiftRightByRegister);
    } else if (BITS(inst, 4, 6) == 6) {
        return DPO(RotateRightByImmediate);
    } else if (BITS(inst, 4, 7) == 7) {
        return DPO(RotateRightByRegister);
    }
    return nullptr;
}

static get_addr_fp_t get_calc_addr_op(unsigned int inst) {
    if (BITS(inst, 24, 27) == 5 && BIT(inst, 21) == 0) {
        return LnSWoUB(ImmediateOffset);
    } else if (BITS(inst, 24, 27) == 7 && BIT(inst, 21) == 0 && BITS(inst, 4, 11) == 0) {
        return LnSWoUB(RegisterOffset);
    } else if (BITS(inst, 24, 27) == 7 && BIT(inst, 21) == 0 && BIT(inst, 4) == 0) {
        return LnSWoUB(ScaledRegisterOffset);
    } else if (BITS(inst, 24, 27) == 5 && BIT(inst, 21) == 1) {
        return LnSWoUB(ImmediatePreIndexed);
    } else if (BITS(inst, 24, 27) == 7 && BIT(inst, 21) == 1 && BITS(inst, 4, 11) == 0) {
        return LnSWoUB(RegisterPreIndexed);
    } else if (BITS(inst, 24, 27) == 7 && BIT(inst, 21) == 1 && BIT(inst, 4) == 0) {
        return LnSWoUB(ScaledRegisterPreIndexed);
    } else if (BITS(inst, 24, 27) == 4 && BIT(inst, 21) == 0) {
        return LnSWoUB(ImmediatePostIndexed);
    } else if (BITS(inst, 24, 27) == 6 && BIT(inst, 21) == 0 && BITS(inst, 4, 11) == 0) {
        return LnSWoUB(RegisterPostIndexed);
    } else if (BITS(inst, 24, 27) == 6 && BIT(inst, 21) == 0 && BIT(inst, 4) == 0) {
        return LnSWoUB(ScaledRegisterPostIndexed);
    } else if (BITS(inst, 24, 27) == 1 && BITS(inst, 21, 22) == 2 && BIT(inst, 7) == 1 && BIT(inst, 4) == 1) {
        return MLnS(ImmediateOffset);
    } else if (BITS(inst, 24, 27) == 1 && BITS(inst, 21, 22) == 0 && BIT(inst, 7) == 1 && BIT(inst, 4) == 1) {
        return MLnS(RegisterOffset);
    } else if (BITS(inst, 24, 27) == 1 && BITS(inst, 21, 22) == 3 && BIT(inst, 7) == 1 && BIT(inst, 4) == 1) {
        return MLnS(ImmediatePreIndexed);
    } else if (BITS(inst, 24, 27) == 1 && BITS(inst, 21, 22) == 1 && BIT(inst, 7) == 1 && BIT(inst, 4) == 1) {
        return MLnS(RegisterPreIndexed);
    } else if (BITS(inst, 24, 27) == 0 && BITS(inst, 21, 22) == 2 && BIT(inst, 7) == 1 && BIT(inst, 4) == 1) {
        return MLnS(ImmediatePostIndexed);
    } else if (BITS(inst, 24, 27) == 0 && BITS(inst, 21, 22) == 0 && BIT(inst, 7) == 1 && BIT(inst, 4) == 1) {
        return MLnS(RegisterPostIndexed);
    } else if (BITS(inst, 23, 27) == 0x11) {
        return LdnStM(IncrementAfter);
    } else if (BITS(inst, 23, 27) == 0x13) {
        return LdnStM(IncrementBefore);
    } else if (BITS(inst, 23, 27) == 0x10) {
        return LdnStM(DecrementAfter);
    } else if (BITS(inst, 23, 27) == 0x12) {
        return LdnStM(DecrementBefore);
    }
    return nullptr;
}