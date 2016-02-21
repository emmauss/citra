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
#include "core/arm/dyncom/arm_dyncom_run.h"
#include "core/arm/skyeye_common/armstate.h"
#include "core/arm/skyeye_common/armsupp.h"
#include "core/arm/skyeye_common/vfp/vfp.h"

#define RM    BITS(sht_oper, 0, 3)
#define RS    BITS(sht_oper, 8, 11)

#define glue(x, y)            x ## y
#define DPO(s)                glue(DataProcessingOperands, s)
#define ROTATE_RIGHT(n, i, l) ((n << (l - i)) | (n >> i))
#define ROTATE_LEFT(n, i, l)  ((n >> (l - i)) | (n << i))
#define ROTATE_RIGHT_32(n, i) ROTATE_RIGHT(n, i, 32)
#define ROTATE_LEFT_32(n, i)  ROTATE_LEFT(n, i, 32)

#define LnSWoUB(s)   glue(LnSWoUB, s)
#define MLnS(s)      glue(MLnS, s)
#define LdnStM(s)    glue(LdnStM, s)

#define W_BIT        BIT(inst, 21)
#define U_BIT        BIT(inst, 23)
#define I_BIT        BIT(inst, 25)
#define P_BIT        BIT(inst, 24)
#define OFFSET_12    BITS(inst, 0, 11)

constexpr std::size_t NUM_REG_GPR = 16;

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
    Gen::R15, //CPSR. Okay this isn't really a GPR.
}};

constexpr std::size_t NUM_REG_SPILL = 7;

const static std::array<Gen::X64Reg, NUM_REG_SPILL> IntToSpillRegister {{
    Gen::MM0, Gen::MM1, Gen::MM2, Gen::MM3, Gen::MM4, Gen::MM5, Gen::MM6
}};

// MM7 is reserved for host RSP save.

typedef unsigned int(*shtop_fp_t)(ARMul_State* cpu, unsigned int sht_oper);

struct RegisterAllocation {
    RegisterAllocation() {
        /// Default state

        for (int i = 0; i < NUM_REG_GPR; i++) {
            arm_reg_location[i] = -1;
            arm_reg_last_used[i] = 2;
        }

        // Heurestic: When fresh, encourage spilling into less commonly used registers first.
        arm_reg_last_used[0] = 3;
        arm_reg_last_used[1] = 3;
        arm_reg_last_used[7] = 1;
        arm_reg_last_used[8] = 0;
        arm_reg_last_used[9] = 0;

        for (int i = 0; i < NUM_REG_GPR; i++) {
            in_use[i] = false;
        }
    }

    void assert_no_temporaries() {
        for (int i = 0; i < NUM_REG_GPR; i++) {
            ASSERT(!in_use[i]);
        }
    }

    /// -1 == Right where it should be (in the respsective general purpose register)
    ///  0 >= In that spill register
    std::array<int, NUM_REG_GPR> arm_reg_location;

    /// Last ARM11 PC for which this ARM register was referenced.
    /// Used to decide what to spill for temporaries.
    std::array<int, NUM_REG_GPR> arm_reg_last_used;

    /// Does that host gpr have a temporary or is this ARM register marked in use in it?
    std::array<bool, NUM_REG_GPR> in_use;
};

namespace Gen {
struct JitBasicBlock : public XCodeBlock {
public:
    void* GetRunPtr() { return nullptr; }

    int Compile(ARMul_State* cpu, void*& bb_start, u32 addr);

private:
    ARMul_State* cpu;

    FixupBranch current_cond_fixup = {};
    ConditionCode current_cond = ConditionCode::AL;
    void CompileCond(ConditionCode new_cond);

    std::array<bool, NUM_REG_SPILL> is_spill_in_use = {};
    X64Reg AllocateSpill() {
        for (int i = 0; i < NUM_REG_SPILL; i++) {
            if (!is_spill_in_use[i]) {
                is_spill_in_use[i] = true;
                return IntToSpillRegister[i];
            }
        }
        return INVALID_REG;
    }

    RegisterAllocation current_register_allocation = {};
    void AcquireArmRegister(int reg);
    X64Reg AcquireTemporaryRegister();
    void ReleaseAllRegisters();
    void ResetAllocation();

    void CompileInstruction_vmla(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmls(u32 instr, int idx, int inst_size);
    void CompileInstruction_vnmla(u32 instr, int idx, int inst_size);
    void CompileInstruction_vnmls(u32 instr, int idx, int inst_size);
    void CompileInstruction_vnmul(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmul(u32 instr, int idx, int inst_size);
    void CompileInstruction_vadd(u32 instr, int idx, int inst_size);
    void CompileInstruction_vsub(u32 instr, int idx, int inst_size);
    void CompileInstruction_vdiv(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmovi(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmovr(u32 instr, int idx, int inst_size);
    void CompileInstruction_vabs(u32 instr, int idx, int inst_size);
    void CompileInstruction_vneg(u32 instr, int idx, int inst_size);
    void CompileInstruction_vsqrt(u32 instr, int idx, int inst_size);
    void CompileInstruction_vcmp(u32 instr, int idx, int inst_size);
    void CompileInstruction_vcmp2(u32 instr, int idx, int inst_size);
    void CompileInstruction_vcvtbds(u32 instr, int idx, int inst_size);
    void CompileInstruction_vcvtbff(u32 instr, int idx, int inst_size);
    void CompileInstruction_vcvtbfi(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmovbrs(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmsr(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmovbrc(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmrs(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmovbcr(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmovbrrss(u32 instr, int idx, int inst_size);
    void CompileInstruction_vmovbrrd(u32 instr, int idx, int inst_size);
    void CompileInstruction_vstr(u32 instr, int idx, int inst_size);
    void CompileInstruction_vpush(u32 instr, int idx, int inst_size);
    void CompileInstruction_vstm(u32 instr, int idx, int inst_size);
    void CompileInstruction_vpop(u32 instr, int idx, int inst_size);
    void CompileInstruction_vldr(u32 instr, int idx, int inst_size);
    void CompileInstruction_vldm(u32 instr, int idx, int inst_size);
    void CompileInstruction_srs(u32 instr, int idx, int inst_size);
    void CompileInstruction_rfe(u32 instr, int idx, int inst_size);
    void CompileInstruction_bkpt(u32 instr, int idx, int inst_size);
    void CompileInstruction_blx(u32 instr, int idx, int inst_size);
    void CompileInstruction_cps(u32 instr, int idx, int inst_size);
    void CompileInstruction_pld(u32 instr, int idx, int inst_size);
    void CompileInstruction_setend(u32 instr, int idx, int inst_size);
    void CompileInstruction_clrex(u32 instr, int idx, int inst_size);
    void CompileInstruction_rev16(u32 instr, int idx, int inst_size);
    void CompileInstruction_usad8(u32 instr, int idx, int inst_size);
    void CompileInstruction_sxtb(u32 instr, int idx, int inst_size);
    void CompileInstruction_uxtb(u32 instr, int idx, int inst_size);
    void CompileInstruction_sxth(u32 instr, int idx, int inst_size);
    void CompileInstruction_sxtb16(u32 instr, int idx, int inst_size);
    void CompileInstruction_uxth(u32 instr, int idx, int inst_size);
    void CompileInstruction_uxtb16(u32 instr, int idx, int inst_size);
    void CompileInstruction_cpy(u32 instr, int idx, int inst_size);
    void CompileInstruction_uxtab(u32 instr, int idx, int inst_size);
    void CompileInstruction_ssub8(u32 instr, int idx, int inst_size);
    void CompileInstruction_shsub8(u32 instr, int idx, int inst_size);
    void CompileInstruction_ssubaddx(u32 instr, int idx, int inst_size);
    void CompileInstruction_strex(u32 instr, int idx, int inst_size);
    void CompileInstruction_strexb(u32 instr, int idx, int inst_size);
    void CompileInstruction_swp(u32 instr, int idx, int inst_size);
    void CompileInstruction_swpb(u32 instr, int idx, int inst_size);
    void CompileInstruction_ssub16(u32 instr, int idx, int inst_size);
    void CompileInstruction_ssat16(u32 instr, int idx, int inst_size);
    void CompileInstruction_shsubaddx(u32 instr, int idx, int inst_size);
    void CompileInstruction_qsubaddx(u32 instr, int idx, int inst_size);
    void CompileInstruction_shaddsubx(u32 instr, int idx, int inst_size);
    void CompileInstruction_shadd8(u32 instr, int idx, int inst_size);
    void CompileInstruction_shadd16(u32 instr, int idx, int inst_size);
    void CompileInstruction_sel(u32 instr, int idx, int inst_size);
    void CompileInstruction_saddsubx(u32 instr, int idx, int inst_size);
    void CompileInstruction_sadd8(u32 instr, int idx, int inst_size);
    void CompileInstruction_sadd16(u32 instr, int idx, int inst_size);
    void CompileInstruction_shsub16(u32 instr, int idx, int inst_size);
    void CompileInstruction_umaal(u32 instr, int idx, int inst_size);
    void CompileInstruction_uxtab16(u32 instr, int idx, int inst_size);
    void CompileInstruction_usubaddx(u32 instr, int idx, int inst_size);
    void CompileInstruction_usub8(u32 instr, int idx, int inst_size);
    void CompileInstruction_usub16(u32 instr, int idx, int inst_size);
    void CompileInstruction_usat16(u32 instr, int idx, int inst_size);
    void CompileInstruction_usada8(u32 instr, int idx, int inst_size);
    void CompileInstruction_uqsubaddx(u32 instr, int idx, int inst_size);
    void CompileInstruction_uqsub8(u32 instr, int idx, int inst_size);
    void CompileInstruction_uqsub16(u32 instr, int idx, int inst_size);
    void CompileInstruction_uqaddsubx(u32 instr, int idx, int inst_size);
    void CompileInstruction_uqadd8(u32 instr, int idx, int inst_size);
    void CompileInstruction_uqadd16(u32 instr, int idx, int inst_size);
    void CompileInstruction_sxtab(u32 instr, int idx, int inst_size);
    void CompileInstruction_uhsubaddx(u32 instr, int idx, int inst_size);
    void CompileInstruction_uhsub8(u32 instr, int idx, int inst_size);
    void CompileInstruction_uhsub16(u32 instr, int idx, int inst_size);
    void CompileInstruction_uhaddsubx(u32 instr, int idx, int inst_size);
    void CompileInstruction_uhadd8(u32 instr, int idx, int inst_size);
    void CompileInstruction_uhadd16(u32 instr, int idx, int inst_size);
    void CompileInstruction_uaddsubx(u32 instr, int idx, int inst_size);
    void CompileInstruction_uadd8(u32 instr, int idx, int inst_size);
    void CompileInstruction_uadd16(u32 instr, int idx, int inst_size);
    void CompileInstruction_sxtah(u32 instr, int idx, int inst_size);
    void CompileInstruction_sxtab16(u32 instr, int idx, int inst_size);
    void CompileInstruction_qadd8(u32 instr, int idx, int inst_size);
    void CompileInstruction_bxj(u32 instr, int idx, int inst_size);
    void CompileInstruction_clz(u32 instr, int idx, int inst_size);
    void CompileInstruction_uxtah(u32 instr, int idx, int inst_size);
    void CompileInstruction_bx(u32 instr, int idx, int inst_size);

    void CompileInstruction_revsh(u32 instr, int idx, int inst_size);
    void CompileInstruction_qadd(u32 instr, int idx, int inst_size);
    void CompileInstruction_qadd16(u32 instr, int idx, int inst_size);
    void CompileInstruction_qaddsubx(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrex(u32 instr, int idx, int inst_size);
    void CompileInstruction_qdadd(u32 instr, int idx, int inst_size);
    void CompileInstruction_qdsub(u32 instr, int idx, int inst_size);
    void CompileInstruction_qsub(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrexb(u32 instr, int idx, int inst_size);
    void CompileInstruction_qsub8(u32 instr, int idx, int inst_size);
    void CompileInstruction_qsub16(u32 instr, int idx, int inst_size);
    void CompileInstruction_smuad(u32 instr, int idx, int inst_size);
    void CompileInstruction_smmul(u32 instr, int idx, int inst_size);
    void CompileInstruction_smusd(u32 instr, int idx, int inst_size);
    void CompileInstruction_smlsd(u32 instr, int idx, int inst_size);
    void CompileInstruction_smlsld(u32 instr, int idx, int inst_size);
    void CompileInstruction_smmla(u32 instr, int idx, int inst_size);
    void CompileInstruction_smmls(u32 instr, int idx, int inst_size);
    void CompileInstruction_smlald(u32 instr, int idx, int inst_size);
    void CompileInstruction_smlad(u32 instr, int idx, int inst_size);
    void CompileInstruction_smlaw(u32 instr, int idx, int inst_size);
    void CompileInstruction_smulw(u32 instr, int idx, int inst_size);
    void CompileInstruction_pkhtb(u32 instr, int idx, int inst_size);
    void CompileInstruction_pkhbt(u32 instr, int idx, int inst_size);
    void CompileInstruction_smul(u32 instr, int idx, int inst_size);
    void CompileInstruction_smlalxy(u32 instr, int idx, int inst_size);
    void CompileInstruction_smla(u32 instr, int idx, int inst_size);
    void CompileInstruction_mcrr(u32 instr, int idx, int inst_size);
    void CompileInstruction_mrrc(u32 instr, int idx, int inst_size);
    void CompileInstruction_cmp(u32 instr, int idx, int inst_size);
    void CompileInstruction_tst(u32 instr, int idx, int inst_size);
    void CompileInstruction_teq(u32 instr, int idx, int inst_size);
    void CompileInstruction_cmn(u32 instr, int idx, int inst_size);
    void CompileInstruction_smull(u32 instr, int idx, int inst_size);
    void CompileInstruction_umull(u32 instr, int idx, int inst_size);
    void CompileInstruction_umlal(u32 instr, int idx, int inst_size);
    void CompileInstruction_smlal(u32 instr, int idx, int inst_size);
    void CompileInstruction_mul(u32 instr, int idx, int inst_size);
    void CompileInstruction_mla(u32 instr, int idx, int inst_size);
    void CompileInstruction_ssat(u32 instr, int idx, int inst_size);
    void CompileInstruction_usat(u32 instr, int idx, int inst_size);
    void CompileInstruction_mrs(u32 instr, int idx, int inst_size);
    void CompileInstruction_msr(u32 instr, int idx, int inst_size);
    void CompileInstruction_and(u32 instr, int idx, int inst_size);
    void CompileInstruction_bic(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldm(u32 instr, int idx, int inst_size);
    void CompileInstruction_eor(u32 instr, int idx, int inst_size);
    void CompileInstruction_add(u32 instr, int idx, int inst_size);
    void CompileInstruction_rsb(u32 instr, int idx, int inst_size);
    void CompileInstruction_rsc(u32 instr, int idx, int inst_size);
    void CompileInstruction_sbc(u32 instr, int idx, int inst_size);
    void CompileInstruction_adc(u32 instr, int idx, int inst_size);
    void CompileInstruction_sub(u32 instr, int idx, int inst_size);
    void CompileInstruction_orr(u32 instr, int idx, int inst_size);
    void CompileInstruction_mvn(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldr(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrcond(u32 instr, int idx, int inst_size);
    void CompileInstruction_str(u32 instr, int idx, int inst_size);


    void CompileInstruction_ldrb(u32 instr, int idx, int inst_size);
    void CompileInstruction_strb(u32 instr, int idx, int inst_size);
    void CompileInstruction_cdp(u32 instr, int idx, int inst_size);
    void CompileInstruction_mov(u32 instr, int idx, int inst_size);
    void CompileInstruction_stm(u32 instr, int idx, int inst_size);

    void CompileInstruction_ldrsh(u32 instr, int idx, int inst_size);


    void CompileInstruction_ldrsb(u32 instr, int idx, int inst_size);
    void CompileInstruction_strd(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrh(u32 instr, int idx, int inst_size);
    void CompileInstruction_strh(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrd(u32 instr, int idx, int inst_size);
    void CompileInstruction_strt(u32 instr, int idx, int inst_size);
    void CompileInstruction_strbt(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrbt(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrt(u32 instr, int idx, int inst_size);
    void CompileInstruction_mrc(u32 instr, int idx, int inst_size);
    void CompileInstruction_mcr(u32 instr, int idx, int inst_size);


    void CompileInstruction_rev(u32 instr, int idx, int inst_size);

    void CompileInstruction_stc(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldc(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrexd(u32 instr, int idx, int inst_size);
    void CompileInstruction_strexd(u32 instr, int idx, int inst_size);
    void CompileInstruction_ldrexh(u32 instr, int idx, int inst_size);
    void CompileInstruction_strexh(u32 instr, int idx, int inst_size);
    void CompileInstruction_nop(u32 instr, int idx, int inst_size);
    void CompileInstruction_yield(u32 instr, int idx, int inst_size);
    void CompileInstruction_wfe(u32 instr, int idx, int inst_size);
    void CompileInstruction_wfi(u32 instr, int idx, int inst_size);
    void CompileInstruction_sev(u32 instr, int idx, int inst_size);
    void CompileInstruction_swi(u32 instr, int idx, int inst_size);
    void CompileInstruction_bbl(u32 instr, int idx, int inst_size);
private:
    int pc;
    int CompileArmInstruction(u32 inst, int inst_size = 4);
};

struct RunJit : public XCodeBlock {
    RunJit() {
        // TODO: Optimize this

        X64Reg tmp = IntToArmGPR[NUM_REG_GPR - 1];

        AllocCodeSpace(1024);
        ABI_PushRegistersAndAdjustStack(ABI_ALL_CALLEE_SAVED, 8);

        MOVQ_mmx(MM0, ABI_PARAM2);

        PUSH(ABI_PARAM1);
        u8* writeback_position = this->GetWritableCodePtr();
        MOV(64, R(tmp), Imm64(0));
        PUSH(tmp);

        MOVQ_mmx(MM7, RSP);

        MOV(64, R(tmp), R(ABI_PARAM1));
        for (int i = 0; i < NUM_REG_GPR-1; i++) {
            MOV(32, R(IntToArmGPR[i]), MDisp(tmp, offsetof(ARMul_State, Reg) + i * sizeof(u32)));
        }

        MOVQ_mmx(tmp, MM0);
        JMPptr(R(tmp));

        u8* writeback_value = this->GetWritableCodePtr();

        MOVQ_mmx(tmp, MM7);
        MOV(64, R(tmp), MDisp(tmp, 8));
        for (int i = 0; i < NUM_REG_GPR - 1; i++) {
            MOV(32, MDisp(tmp, offsetof(ARMul_State, Reg) + i * sizeof(u32)), R(IntToArmGPR[i]));
        }

        MOVQ_mmx(RSP, MM7);
        POP(tmp);
        POP(tmp);

        MOVQ_mmx(ABI_RETURN, MM0);
        EMMS(); // End MMX State
        ABI_PopRegistersAndAdjustStack(ABI_ALL_CALLEE_SAVED, 8);
        RET();

        SetCodePtr(writeback_position);
        MOV(64, R(tmp), Imm64((u64)writeback_value));
    }

    u64 Run(ARMul_State* cpu, void* bb) {
        auto fn = (u64(*)(u64, u64))region;
        return fn((u64)cpu, (u64)bb);
    }
};
};

Gen::RunJit run_jit;


int Gen::JitBasicBlock::Compile(ARMul_State* cpu, void*& bb_start, u32 addr) {
    this->cpu = cpu;
    this->pc = addr;

    AllocCodeSpace(1024);
    bb_start = this->region;

    CompileArmInstruction(0b11100010100000110000000000000011);

    MOVQ_mmx(R15, MM7);
    MOV(64, R(R15), MDisp(R15, 0));
    JMPptr(R(R15));

    return 0;
}

int Gen::JitBasicBlock::CompileArmInstruction(u32 inst, int inst_size) {
    int idx;
    if (DecodeARMInstruction(inst, &idx) == ARMDecodeStatus::FAILURE) {
        std::string disasm = ARM_Disasm::Disassemble(pc, inst);
        LOG_ERROR(Core_ARM11, "Decode failure.\tPC : [0x%x]\tInstruction : %s [%x]", pc, disasm.c_str(), inst);
    }

#define COMPILE_INSTRUCTION(name) CompileInstruction_ ## name (inst, idx, inst_size)
    switch (idx) {
    case 0: COMPILE_INSTRUCTION(vmla); break;
    case 1: COMPILE_INSTRUCTION(vmls); break;
    case 2: COMPILE_INSTRUCTION(vnmla); break;
    case 3: COMPILE_INSTRUCTION(vnmls); break;
    case 4: COMPILE_INSTRUCTION(vnmul); break;
    case 5: COMPILE_INSTRUCTION(vmul); break;
    case 6: COMPILE_INSTRUCTION(vadd); break;
    case 7: COMPILE_INSTRUCTION(vsub); break;
    case 8: COMPILE_INSTRUCTION(vdiv); break;
    case 9: COMPILE_INSTRUCTION(vmovi); break;
    case 10: COMPILE_INSTRUCTION(vmovr); break;
    case 11: COMPILE_INSTRUCTION(vabs); break;
    case 12: COMPILE_INSTRUCTION(vneg); break;
    case 13: COMPILE_INSTRUCTION(vsqrt); break;
    case 14: COMPILE_INSTRUCTION(vcmp); break;
    case 15: COMPILE_INSTRUCTION(vcmp2); break;
    case 16: COMPILE_INSTRUCTION(vcvtbds); break;
    case 17: COMPILE_INSTRUCTION(vcvtbff); break;
    case 18: COMPILE_INSTRUCTION(vcvtbfi); break;
    case 19: COMPILE_INSTRUCTION(vmovbrs); break;
    case 20: COMPILE_INSTRUCTION(vmsr); break;
    case 21: COMPILE_INSTRUCTION(vmovbrc); break;
    case 22: COMPILE_INSTRUCTION(vmrs); break;
    case 23: COMPILE_INSTRUCTION(vmovbcr); break;
    case 24: COMPILE_INSTRUCTION(vmovbrrss); break;
    case 25: COMPILE_INSTRUCTION(vmovbrrd); break;
    case 26: COMPILE_INSTRUCTION(vstr); break;
    case 27: COMPILE_INSTRUCTION(vpush); break;
    case 28: COMPILE_INSTRUCTION(vstm); break;
    case 29: COMPILE_INSTRUCTION(vpop); break;
    case 30: COMPILE_INSTRUCTION(vldr); break;
    case 31: COMPILE_INSTRUCTION(vldm); break;
    case 32: COMPILE_INSTRUCTION(srs); break;
    case 33: COMPILE_INSTRUCTION(rfe); break;
    case 34: COMPILE_INSTRUCTION(bkpt); break;
    case 35: COMPILE_INSTRUCTION(blx); break;
    case 36: COMPILE_INSTRUCTION(cps); break;
    case 37: COMPILE_INSTRUCTION(pld); break;
    case 38: COMPILE_INSTRUCTION(setend); break;
    case 39: COMPILE_INSTRUCTION(clrex); break;
    case 40: COMPILE_INSTRUCTION(rev16); break;
    case 41: COMPILE_INSTRUCTION(usad8); break;
    case 42: COMPILE_INSTRUCTION(sxtb); break;
    case 43: COMPILE_INSTRUCTION(uxtb); break;
    case 44: COMPILE_INSTRUCTION(sxth); break;
    case 45: COMPILE_INSTRUCTION(sxtb16); break;
    case 46: COMPILE_INSTRUCTION(uxth); break;
    case 47: COMPILE_INSTRUCTION(uxtb16); break;
    case 48: COMPILE_INSTRUCTION(cpy); break;
    case 49: COMPILE_INSTRUCTION(uxtab); break;
    case 50: COMPILE_INSTRUCTION(ssub8); break;
    case 51: COMPILE_INSTRUCTION(shsub8); break;
    case 52: COMPILE_INSTRUCTION(ssubaddx); break;
    case 53: COMPILE_INSTRUCTION(strex); break;
    case 54: COMPILE_INSTRUCTION(strexb); break;
    case 55: COMPILE_INSTRUCTION(swp); break;
    case 56: COMPILE_INSTRUCTION(swpb); break;
    case 57: COMPILE_INSTRUCTION(ssub16); break;
    case 58: COMPILE_INSTRUCTION(ssat16); break;
    case 59: COMPILE_INSTRUCTION(shsubaddx); break;
    case 60: COMPILE_INSTRUCTION(qsubaddx); break;
    case 61: COMPILE_INSTRUCTION(shaddsubx); break;
    case 62: COMPILE_INSTRUCTION(shadd8); break;
    case 63: COMPILE_INSTRUCTION(shadd16); break;
    case 64: COMPILE_INSTRUCTION(sel); break;
    case 65: COMPILE_INSTRUCTION(saddsubx); break;
    case 66: COMPILE_INSTRUCTION(sadd8); break;
    case 67: COMPILE_INSTRUCTION(sadd16); break;
    case 68: COMPILE_INSTRUCTION(shsub16); break;
    case 69: COMPILE_INSTRUCTION(umaal); break;
    case 70: COMPILE_INSTRUCTION(uxtab16); break;
    case 71: COMPILE_INSTRUCTION(usubaddx); break;
    case 72: COMPILE_INSTRUCTION(usub8); break;
    case 73: COMPILE_INSTRUCTION(usub16); break;
    case 74: COMPILE_INSTRUCTION(usat16); break;
    case 75: COMPILE_INSTRUCTION(usada8); break;
    case 76: COMPILE_INSTRUCTION(uqsubaddx); break;
    case 77: COMPILE_INSTRUCTION(uqsub8); break;
    case 78: COMPILE_INSTRUCTION(uqsub16); break;
    case 79: COMPILE_INSTRUCTION(uqaddsubx); break;
    case 80: COMPILE_INSTRUCTION(uqadd8); break;
    case 81: COMPILE_INSTRUCTION(uqadd16); break;
    case 82: COMPILE_INSTRUCTION(sxtab); break;
    case 83: COMPILE_INSTRUCTION(uhsubaddx); break;
    case 84: COMPILE_INSTRUCTION(uhsub8); break;
    case 85: COMPILE_INSTRUCTION(uhsub16); break;
    case 86: COMPILE_INSTRUCTION(uhaddsubx); break;
    case 87: COMPILE_INSTRUCTION(uhadd8); break;
    case 88: COMPILE_INSTRUCTION(uhadd16); break;
    case 89: COMPILE_INSTRUCTION(uaddsubx); break;
    case 90: COMPILE_INSTRUCTION(uadd8); break;
    case 91: COMPILE_INSTRUCTION(uadd16); break;
    case 92: COMPILE_INSTRUCTION(sxtah); break;
    case 93: COMPILE_INSTRUCTION(sxtab16); break;
    case 94: COMPILE_INSTRUCTION(qadd8); break;
    case 95: COMPILE_INSTRUCTION(bxj); break;
    case 96: COMPILE_INSTRUCTION(clz); break;
    case 97: COMPILE_INSTRUCTION(uxtah); break;
    case 98: COMPILE_INSTRUCTION(bx); break;
    case 99: COMPILE_INSTRUCTION(rev); break;
    case 100: COMPILE_INSTRUCTION(blx); break;
    case 101: COMPILE_INSTRUCTION(revsh); break;
    case 102: COMPILE_INSTRUCTION(qadd); break;
    case 103: COMPILE_INSTRUCTION(qadd16); break;
    case 104: COMPILE_INSTRUCTION(qaddsubx); break;
    case 105: COMPILE_INSTRUCTION(ldrex); break;
    case 106: COMPILE_INSTRUCTION(qdadd); break;
    case 107: COMPILE_INSTRUCTION(qdsub); break;
    case 108: COMPILE_INSTRUCTION(qsub); break;
    case 109: COMPILE_INSTRUCTION(ldrexb); break;
    case 110: COMPILE_INSTRUCTION(qsub8); break;
    case 111: COMPILE_INSTRUCTION(qsub16); break;
    case 112: COMPILE_INSTRUCTION(smuad); break;
    case 113: COMPILE_INSTRUCTION(smmul); break;
    case 114: COMPILE_INSTRUCTION(smusd); break;
    case 115: COMPILE_INSTRUCTION(smlsd); break;
    case 116: COMPILE_INSTRUCTION(smlsld); break;
    case 117: COMPILE_INSTRUCTION(smmla); break;
    case 118: COMPILE_INSTRUCTION(smmls); break;
    case 119: COMPILE_INSTRUCTION(smlald); break;
    case 120: COMPILE_INSTRUCTION(smlad); break;
    case 121: COMPILE_INSTRUCTION(smlaw); break;
    case 122: COMPILE_INSTRUCTION(smulw); break;
    case 123: COMPILE_INSTRUCTION(pkhtb); break;
    case 124: COMPILE_INSTRUCTION(pkhbt); break;
    case 125: COMPILE_INSTRUCTION(smul); break;
    case 126: COMPILE_INSTRUCTION(smlalxy); break;
    case 127: COMPILE_INSTRUCTION(smla); break;
    case 128: COMPILE_INSTRUCTION(mcrr); break;
    case 129: COMPILE_INSTRUCTION(mrrc); break;
    case 130: COMPILE_INSTRUCTION(cmp); break;
    case 131: COMPILE_INSTRUCTION(tst); break;
    case 132: COMPILE_INSTRUCTION(teq); break;
    case 133: COMPILE_INSTRUCTION(cmn); break;
    case 134: COMPILE_INSTRUCTION(smull); break;
    case 135: COMPILE_INSTRUCTION(umull); break;
    case 136: COMPILE_INSTRUCTION(umlal); break;
    case 137: COMPILE_INSTRUCTION(smlal); break;
    case 138: COMPILE_INSTRUCTION(mul); break;
    case 139: COMPILE_INSTRUCTION(mla); break;
    case 140: COMPILE_INSTRUCTION(ssat); break;
    case 141: COMPILE_INSTRUCTION(usat); break;
    case 142: COMPILE_INSTRUCTION(mrs); break;
    case 143: COMPILE_INSTRUCTION(msr); break;
    case 144: COMPILE_INSTRUCTION(and); break;
    case 145: COMPILE_INSTRUCTION(bic); break;
    case 146: COMPILE_INSTRUCTION(ldm); break;
    case 147: COMPILE_INSTRUCTION(eor); break;
    case 148: COMPILE_INSTRUCTION(add); break;
    case 149: COMPILE_INSTRUCTION(rsb); break;
    case 150: COMPILE_INSTRUCTION(rsc); break;
    case 151: COMPILE_INSTRUCTION(sbc); break;
    case 152: COMPILE_INSTRUCTION(adc); break;
    case 153: COMPILE_INSTRUCTION(sub); break;
    case 154: COMPILE_INSTRUCTION(orr); break;
    case 155: COMPILE_INSTRUCTION(mvn); break;
    case 156: COMPILE_INSTRUCTION(ldr); break;
    case 157: COMPILE_INSTRUCTION(ldrcond); break;
    case 158: COMPILE_INSTRUCTION(str); break;
    case 159: COMPILE_INSTRUCTION(msr); break;
    case 160: COMPILE_INSTRUCTION(msr); break;
    case 161: COMPILE_INSTRUCTION(ldrb); break;
    case 162: COMPILE_INSTRUCTION(strb); break;
    case 163: COMPILE_INSTRUCTION(cdp); break;
    case 164: COMPILE_INSTRUCTION(mov); break;
    case 165: COMPILE_INSTRUCTION(stm); break;
    case 166: COMPILE_INSTRUCTION(ldm); break;
    case 167: COMPILE_INSTRUCTION(ldrsh); break;
    case 168: COMPILE_INSTRUCTION(stm); break;
    case 169: COMPILE_INSTRUCTION(ldm); break;
    case 170: COMPILE_INSTRUCTION(ldrsb); break;
    case 171: COMPILE_INSTRUCTION(strd); break;
    case 172: COMPILE_INSTRUCTION(ldrh); break;
    case 173: COMPILE_INSTRUCTION(strh); break;
    case 174: COMPILE_INSTRUCTION(ldrd); break;
    case 175: COMPILE_INSTRUCTION(strt); break;
    case 176: COMPILE_INSTRUCTION(strbt); break;
    case 177: COMPILE_INSTRUCTION(ldrbt); break;
    case 178: COMPILE_INSTRUCTION(ldrt); break;
    case 179: COMPILE_INSTRUCTION(mrc); break;
    case 180: COMPILE_INSTRUCTION(mcr); break;
    case 181: COMPILE_INSTRUCTION(msr); break;
    case 182: COMPILE_INSTRUCTION(msr); break;
    case 183: COMPILE_INSTRUCTION(msr); break;
    case 184: COMPILE_INSTRUCTION(stc); break;
    case 185: COMPILE_INSTRUCTION(ldc); break;
    case 186: COMPILE_INSTRUCTION(ldrexd); break;
    case 187: COMPILE_INSTRUCTION(strexd); break;
    case 188: COMPILE_INSTRUCTION(ldrexh); break;
    case 189: COMPILE_INSTRUCTION(strexh); break;
    case 190: COMPILE_INSTRUCTION(nop); break;
    case 191: COMPILE_INSTRUCTION(yield); break;
    case 192: COMPILE_INSTRUCTION(wfe); break;
    case 193: COMPILE_INSTRUCTION(wfi); break;
    case 194: COMPILE_INSTRUCTION(sev); break;
    case 195: COMPILE_INSTRUCTION(swi); break;
    case 196: COMPILE_INSTRUCTION(bbl); break;
    }
    return 0;
}

unsigned JitMainLoop(ARMul_State* cpu) {
//    cpu->jit_cache[];

    void *ptr;

    auto itr = cpu->jit_cache.find(cpu->Reg[15]);
    if (itr != cpu->jit_cache.end()) {
        ptr = itr->second->GetRunPtr();
    } else {
        auto bb = new Gen::JitBasicBlock();
        bb->Compile(cpu, ptr, cpu->Reg[15]);
        cpu->jit_cache[cpu->Reg[15]] = bb;
    }

    run_jit.Run(cpu, ptr);
    return 0;
}

void TestSystem() {
    ARMul_State state(PrivilegeMode::USER32MODE);
    state.Reg[0] = 1;
    state.Reg[1] = 1;

    void* ptr;

    for (int i = 0; i < 15; i++) {
        printf("R%i : %08X\n", i, state.Reg[i]);
    }

    auto bb = new Gen::JitBasicBlock();
    bb->Compile(&state, ptr, 0);
    bb->RET();
    run_jit.Run(&state, ptr);

    for (int i = 0; i < 15; i++) {
        printf("R%i : %08X\n", i, state.Reg[i]);
    }

    system("pause");
}

enum ShtOp {
    DPO(Immediate),
    DPO(Register),
    DPO(LogicalShiftLeftByImmediate),
    DPO(LogicalShiftLeftByRegister),
    DPO(LogicalShiftRightByImmediate),
    DPO(LogicalShiftRightByRegister),
    DPO(ArithmeticShiftRightByImmediate),
    DPO(ArithmeticShiftRightByRegister),
    DPO(RotateRightByImmediate),
    DPO(RotateRightByRegister),
    ShtOp_ERROR
};

static ShtOp get_shtop(unsigned int inst) {
    if (BIT(inst, 25)) {
        return DPO(Immediate);
    }
    else if (BITS(inst, 4, 11) == 0) {
        return DPO(Register);
    }
    else if (BITS(inst, 4, 6) == 0) {
        return DPO(LogicalShiftLeftByImmediate);
    }
    else if (BITS(inst, 4, 7) == 1) {
        return DPO(LogicalShiftLeftByRegister);
    }
    else if (BITS(inst, 4, 6) == 2) {
        return DPO(LogicalShiftRightByImmediate);
    }
    else if (BITS(inst, 4, 7) == 3) {
        return DPO(LogicalShiftRightByRegister);
    }
    else if (BITS(inst, 4, 6) == 4) {
        return DPO(ArithmeticShiftRightByImmediate);
    }
    else if (BITS(inst, 4, 7) == 5) {
        return DPO(ArithmeticShiftRightByRegister);
    }
    else if (BITS(inst, 4, 6) == 6) {
        return DPO(RotateRightByImmediate);
    }
    else if (BITS(inst, 4, 7) == 7) {
        return DPO(RotateRightByRegister);
    }
    return ShtOp_ERROR;
}


enum CalcAddrOp {
    LnSWoUB(ImmediateOffset),
    LnSWoUB(RegisterOffset),
    LnSWoUB(ScaledRegisterOffset),
    LnSWoUB(ImmediatePreIndexed),
    LnSWoUB(RegisterPreIndexed),
    LnSWoUB(ScaledRegisterPreIndexed),
    LnSWoUB(ImmediatePostIndexed),
    LnSWoUB(RegisterPostIndexed),
    LnSWoUB(ScaledRegisterPostIndexed),
    MLnS(ImmediateOffset),
    MLnS(RegisterOffset),
    MLnS(ImmediatePreIndexed),
    MLnS(RegisterPreIndexed),
    MLnS(ImmediatePostIndexed),
    MLnS(RegisterPostIndexed),
    LdnStM(IncrementAfter),
    LdnStM(IncrementBefore),
    LdnStM(DecrementAfter),
    LdnStM(DecrementBefore),
    CalcAddrOp_ERROR
};

static CalcAddrOp get_calc_addr_op(unsigned int inst) {
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
    return CalcAddrOp_ERROR;
}

void Gen::JitBasicBlock::CompileCond(ConditionCode new_cond) {
    if (new_cond == current_cond)
        return;

    if (current_cond != ConditionCode::AL) {
        ResetAllocation();
        SetJumpTarget(current_cond_fixup);
    }

    if (new_cond != ConditionCode::AL) {
        X64Reg save_CSPR = AllocateSpill();

        // In the CPSR,
        // N is bit 31
        // Z is bit 30
        // C is bit 29
        // V is bit 28

        constexpr int N_BIT = 31;
        constexpr int Z_BIT = 30;
        constexpr int C_BIT = 29;
        constexpr int V_BIT = 28;
        constexpr int N_MASK = 1 << N_BIT;
        constexpr int Z_MASK = 1 << Z_BIT;
        constexpr int C_MASK = 1 << C_BIT;
        constexpr int V_MASK = 1 << V_BIT;

        const X64Reg CSPR_reg = IntToArmGPR[15];
        CCFlags cc;

        MOVQ_mmx(save_CSPR, CSPR_reg);
        switch (new_cond) {
        case ConditionCode::EQ: //z
            AND(32, R(CSPR_reg), Imm32(Z_MASK));
            cc = CC_Z;
            break;
        case ConditionCode::NE: //!z
            AND(32, R(CSPR_reg), Imm32(Z_MASK));
            cc = CC_NZ;
            break;
        case ConditionCode::CS: //c
            AND(32, R(CSPR_reg), Imm32(C_MASK));
            cc = CC_Z;
            break;
        case ConditionCode::CC: //!c
            AND(32, R(CSPR_reg), Imm32(C_MASK));
            cc = CC_NZ;
            break;
        case ConditionCode::MI: //n
            AND(32, R(CSPR_reg), Imm32(N_MASK));
            cc = CC_Z;
            break;
        case ConditionCode::PL: //!n
            AND(32, R(CSPR_reg), Imm32(N_MASK));
            cc = CC_NZ;
            break;
        case ConditionCode::VS: //v
            AND(32, R(CSPR_reg), Imm32(V_MASK));
            cc = CC_Z;
            break;
        case ConditionCode::VC: //!v
            AND(32, R(CSPR_reg), Imm32(V_MASK));
            cc = CC_NZ;
            break;
        case ConditionCode::HI: //c & !z
            AND(32, R(CSPR_reg), Imm32(C_MASK | Z_MASK));
            CMP(32, R(CSPR_reg), Imm32(C_MASK));
            cc = CC_NE;
            break;
        case ConditionCode::LS: //!c | z
            AND(32, R(CSPR_reg), Imm32(C_MASK | Z_MASK));
            CMP(32, R(CSPR_reg), Imm32(C_MASK));
            cc = CC_E;
            break;
        case ConditionCode::GE: { // n == v
            X64Reg n = CSPR_reg;
            X64Reg v = AcquireTemporaryRegister();
            MOV(32, R(v), R(n));
            SHR(32, R(n), Imm8(N_BIT));
            SHR(32, R(v), Imm8(V_BIT));
            AND(32, R(v), Imm8(1));
            CMP(32, R(v), R(n));
            cc = CC_NE;
            break;
        }
        case ConditionCode::LT: { // n != v
            X64Reg n = CSPR_reg;
            X64Reg v = AcquireTemporaryRegister();
            MOV(32, R(v), R(n));
            SHR(32, R(n), Imm8(N_BIT));
            SHR(32, R(v), Imm8(V_BIT));
            AND(32, R(v), Imm8(1));
            CMP(32, R(v), R(n));
            cc = CC_E;
            break;
        }
        case ConditionCode::GT: { // !z & (n == v)
            X64Reg z = CSPR_reg;
            X64Reg n = AcquireTemporaryRegister();
            X64Reg v = AcquireTemporaryRegister();
            MOV(32, R(n), R(z));
            MOV(32, R(v), R(z));
            SHR(32, R(n), Imm8(N_BIT));
            SHR(32, R(v), Imm8(V_BIT));
            SHR(32, R(z), Imm8(Z_BIT));
            XOR(32, R(v), R(n));
            OR(32, R(z), R(v));
            AND(32, R(z), Imm8(1));
            cc = CC_NZ;
            break;
        }
        case ConditionCode::LE: { // z | (n != v)
            X64Reg z = CSPR_reg;
            X64Reg n = AcquireTemporaryRegister();
            X64Reg v = AcquireTemporaryRegister();
            MOV(32, R(n), R(z));
            MOV(32, R(v), R(z));
            SHR(32, R(n), Imm8(N_BIT));
            SHR(32, R(v), Imm8(V_BIT));
            SHR(32, R(z), Imm8(Z_BIT));
            XOR(32, R(v), R(n));
            OR(32, R(z), R(v));
            AND(32, R(z), Imm8(1));
            cc = CC_Z;
            break;
        }
        default:
            ASSERT_MSG(false, "This should never happen.");
            break;
        }

        MOVQ_mmx(CSPR_reg, save_CSPR);
        ReleaseAllRegisters();
        ResetAllocation();
        this->current_cond_fixup = J_CC(cc);
    }

    current_cond = new_cond;
}

void Gen::JitBasicBlock::AcquireArmRegister(int arm_reg) {
    current_register_allocation.arm_reg_last_used[arm_reg] = this->pc;

    ASSERT(!current_register_allocation.in_use[arm_reg]);

    current_register_allocation.in_use[arm_reg] = true;

    int spill = current_register_allocation.arm_reg_location[arm_reg];
    current_register_allocation.arm_reg_location[arm_reg] = -1;

    if (spill == -1)
        return;

    MOVQ_mmx(IntToArmGPR[arm_reg], (X64Reg)spill);

    is_spill_in_use[spill - Gen::MM0] = false;
}

Gen::X64Reg Gen::JitBasicBlock::AcquireTemporaryRegister() {
    // First try to allocate something which was spilled previously and is now free.
    for (int i = 0; i < NUM_REG_GPR; i++) {
        if (current_register_allocation.arm_reg_location[i] != -1 && !current_register_allocation.in_use[i]) {
            current_register_allocation.in_use[i] = true;
            return IntToArmGPR[i];
        }
    }

    // Otherwise, spill something.
    int bestreg = -1;
    int bestreg_lastuse = INT_MAX;
    for (int i = 0; i < NUM_REG_GPR; i++) {
        if (!current_register_allocation.in_use[i]) {
            if (current_register_allocation.arm_reg_last_used[i] < bestreg_lastuse) {
                bestreg_lastuse = current_register_allocation.arm_reg_last_used[i];
                bestreg = i;
            }
        }
    }

    ASSERT(bestreg != -1);

    X64Reg spill = AllocateSpill();
    ASSERT(spill != Gen::INVALID_REG);

    MOVQ_mmx(spill, IntToArmGPR[bestreg]);
    current_register_allocation.arm_reg_location[bestreg] = spill;

    return IntToArmGPR[bestreg];
}

void Gen::JitBasicBlock::ReleaseAllRegisters() {
    for (int i = 0; i < NUM_REG_GPR; i++) {
        current_register_allocation.in_use[i] = false;
    }

    current_register_allocation.assert_no_temporaries();
}

// Reset allocation MUST NOT TOUCH any conditional flags
void Gen::JitBasicBlock::ResetAllocation() {
    ReleaseAllRegisters();

    for (int i = 0; i < NUM_REG_GPR; i++) {
        int spill = current_register_allocation.arm_reg_location[i];
        current_register_allocation.arm_reg_location[i] = -1;

        if (spill == -1)
            continue;

        MOVQ_mmx(IntToArmGPR[i], (X64Reg)spill);
    }

    for (int i = 0; i < NUM_REG_SPILL; i++) {
        is_spill_in_use[i] = false;
    }

    current_register_allocation.assert_no_temporaries();
}

#define BEFORE_COMPILE_INSTRUCTION                      \
    if (BITS(inst, 28, 31) == ConditionCode::NV)   \
        return;                                         \
    CompileCond((ConditionCode)BITS(inst, 28, 31));

void Gen::JitBasicBlock::CompileInstruction_vmla(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmls(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vnmla(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vnmls(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vnmul(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmul(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vadd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vsub(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vdiv(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmovi(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmovr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vabs(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vneg(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vsqrt(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vcmp(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vcmp2(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vcvtbds(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vcvtbff(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vcvtbfi(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmovbrs(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmsr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmovbrc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmrs(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmovbcr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmovbrrss(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vmovbrrd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vstr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vpush(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vstm(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vpop(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vldr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_vldm(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_srs(u32 instr, int idx, int inst_size) {

}
void Gen::JitBasicBlock::CompileInstruction_rfe(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_bkpt(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_blx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_cps(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_pld(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_setend(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_clrex(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_rev16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_usad8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sxtb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uxtb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sxth(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sxtb16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uxth(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uxtb16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_cpy(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uxtab(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ssub8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_shsub8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ssubaddx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strex(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strexb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_swp(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_swpb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ssub16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ssat16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_shsubaddx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qsubaddx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_shaddsubx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_shadd8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_shadd16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sel(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_saddsubx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sadd8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sadd16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_shsub16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_umaal(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uxtab16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_usubaddx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_usub8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_usub16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_usat16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_usada8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uqsubaddx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uqsub8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uqsub16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uqaddsubx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uqadd8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uqadd16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sxtab(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uhsubaddx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uhsub8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uhsub16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uhaddsubx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uhadd8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uhadd16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uaddsubx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uadd8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uadd16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sxtah(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sxtab16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qadd8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_bxj(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_clz(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_uxtah(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_bx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_rev(u32 inst, int idx, int inst_size) {}

void Gen::JitBasicBlock::CompileInstruction_revsh(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qadd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qadd16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qaddsubx(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrex(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qdadd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qdsub(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qsub(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrexb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qsub8(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_qsub16(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smuad(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smmul(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smusd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smlsd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smlsld(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smmla(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smmls(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smlald(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smlad(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smlaw(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smulw(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_pkhtb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_pkhbt(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smul(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smlalxy(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smla(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mcrr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mrrc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_cmp(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_tst(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_teq(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_cmn(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smull(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_umull(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_umlal(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_smlal(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mul(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mla(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ssat(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_usat(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mrs(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_msr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_and(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_bic(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldm(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_eor(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_add(u32 inst, int idx, int inst_size) {
    BEFORE_COMPILE_INSTRUCTION;

    const u32 I = BIT(inst, 25);
    const u32 S = BIT(inst, 20);
    const u32 Rn = BITS(inst, 16, 19);
    const u32 Rd = BITS(inst, 12, 15);
    const u32 shifter_operand = BITS(inst, 0, 11);
    const u32 shtop_func = get_shtop(inst);

    AcquireArmRegister(Rn);
    AcquireArmRegister(Rd);

    if (Rd == 15) {
        ASSERT_MSG(0, "Unimplemented");

        // rn_val = RN + 2 * cpu->GetInstructionSize();
        // goto DISPATCH
        return;
    }

    const X64Reg Rn_reg = IntToArmGPR[Rn];
    const X64Reg Rd_reg = IntToArmGPR[Rd];

    switch (shtop_func) {
    case DPO(Immediate): {
        u32 imm8 = BITS(inst, 0, 7);
        u32 rotate = BITS(inst, 8, 11);
        u32 operand = ROTATE_RIGHT_32(imm8, rotate * 2);

        ADC(32, R(Rd_reg), R(Rn_reg));
        if (operand != 0) ADC(32, R(Rd_reg), Imm32(operand));
        break;
    }
    default:
        ASSERT(0, "Unimplemented");
    }

    /*    else if (inst_cream->S) {
            UPDATE_NFLAG(RD);
            UPDATE_ZFLAG(RD);
            cpu->CFlag = carry;
            cpu->VFlag = overflow;
        }*/

    this->pc += inst_size;
}
void Gen::JitBasicBlock::CompileInstruction_rsb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_rsc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sbc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_adc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sub(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_orr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mvn(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrcond(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_str(u32 inst, int idx, int inst_size) {}

void Gen::JitBasicBlock::CompileInstruction_ldrb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_cdp(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mov(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_stm(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrsh(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrsb(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrh(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strh(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strt(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strbt(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrbt(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrt(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mrc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_mcr(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_stc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldc(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrexd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strexd(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_ldrexh(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_strexh(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_nop(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_yield(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_wfe(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_wfi(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_sev(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_swi(u32 inst, int idx, int inst_size) {}
void Gen::JitBasicBlock::CompileInstruction_bbl(u32 inst, int idx, int inst_size) {}
