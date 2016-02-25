// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>

#include "common/common_types.h"

#include "common/common_types.h"
#include "common/x64/abi.h"
#include "common/x64/emitter.h"

#include "core/arm/dyncom/arm_dyncom_translate.h"
#include "core/arm/dyncom/jit/common.h"
#include "core/arm/dyncom/jit/jit.h"

namespace Jit {
struct RegisterAllocation {
    RegisterAllocation() {
        Reset();
    }

    void Reset() {
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
}

namespace Gen {

struct JitCompiler final : private XCodeBlock {
public:
    JitCompiler() {
        AllocCodeSpace(64 * 1024 * 2000 * 100);
    }

    int Compile(void*& bb_start, u32 addr, bool TFlag);

    void ClearCache() {
        ResetCodePtr();
        basic_blocks.clear();
        update_jmps.clear();
    }

    std::unordered_map<u32, u8*> basic_blocks;
    std::unordered_map<u32, std::vector<u8*>> update_jmps;

    bool debug;

private:
    Jit::RegisterAllocation current_register_allocation;
    X64Reg AcquireArmRegister(int reg);
    X64Reg AcquireTemporaryRegister();
    X64Reg AcquireCopyOfArmRegister(int reg);
    X64Reg EnsureTemp(X64Reg);
    void ReleaseTemporaryRegister(Gen::X64Reg reg);
    void ReleaseAllRegisters();
    void SpillAllRegisters();
    void ResetAllocation();
    void RestoreRSP();

    bool cl_active = false;
    Gen::X64Reg cl_active_tmp = INVALID_REG;
    void AcquireCLRegister(int arm_reg_to_copy = -1);
    void ReleaseCLRegister();

    void CallHostFunction(Jit::JitState*(*fn)(Jit::JitState*, u64, u64, u64), u64, u64, u64);

    bool status_flag_update = false;
    FixupBranch current_cond_fixup = {};
    ConditionCode current_cond = ConditionCode::AL;
    void CompileCond(ConditionCode new_cond);

    Gen::X64Reg CompileShifterOperand(shtop_fp_t shtop_func, unsigned shifter_operand, bool CSO, unsigned inst_size);

    /// Warning: This destroys addr_reg
    void CompileMemoryRead(unsigned bits, Gen::X64Reg dest, Gen::X64Reg src_addr_reg);
    void CompileMemoryWrite(unsigned bits, Gen::X64Reg dest_addr_reg, Gen::X64Reg src);

    /// Update cycles_remaining before calling this function.
    void CompileMaybeJumpToBB(u32 new_pc);
    /// Update Reg[15] before calling this function.
    bool CompileReturnToDispatch();

private:
    u32 GetReg15(unsigned inst_size) { return this->pc + inst_size * 2;  }
    int cycles;
    u32 pc;
    int TFlag;
    bool CompileSingleInstruction();
    bool CompileInstruction_Interpret(unsigned inst_size);

    template<typename T>
    bool CompileInstruction_Arithmetic(arm_inst* inst, unsigned inst_size, void (Gen::XEmitter::*fn)(int bits, const OpArg& a1, const OpArg& a2), int carry, bool commutative);
    bool CompileInstruction_adc(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_add(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_sbc(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_sub(arm_inst* inst, unsigned inst_size);

    template<typename T>
    bool CompileInstruction_ReverseSubtraction(arm_inst* inst, unsigned inst_size, void (Gen::XEmitter::*fn)(int bits, const OpArg& a1, const OpArg& a2), bool carry);
    bool CompileInstruction_rsb(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_rsc(arm_inst* inst, unsigned inst_size);

    template<typename T>
    bool CompileInstruction_Logical(arm_inst* inst, unsigned inst_size, void (Gen::XEmitter::*fn)(int bits, const OpArg& a1, const OpArg& a2), bool invert_operand);
    bool CompileInstruction_and(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_eor(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_orr(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_bic(arm_inst* inst, unsigned inst_size);

    bool CompileInstruction_cmp(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_cmn(arm_inst* inst, unsigned inst_size);

    bool CompileInstruction_mov(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_mvn(arm_inst* inst, unsigned inst_size);

    bool CompileInstruction_teq(arm_inst* inst, unsigned inst_size);
    bool CompileInstruction_tst(arm_inst* inst, unsigned inst_size);

    bool CompileInstruction_ldr(arm_inst* inst, unsigned inst_size);

    bool CompileInstruction_bl(arm_inst* inst, unsigned inst_size);
};

}

namespace Jit {
JitState* InterpretSingleInstruction(JitState* jit_state, u64 pc, u64 TFlag, u64);
}