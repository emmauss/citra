// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>
#include <unordered_map>

#include "common/common_types.h"

#include "core/arm/arm_interface.h"
#include "core/arm/skyeye_common/arm_regformat.h"
#include "core/arm/skyeye_common/armstate.h"

namespace Core {
struct ThreadContext;
}

namespace Jit {

struct BasicBlock;

constexpr std::size_t NUM_REG_GPR = 15;

struct alignas(64) JitState {
    void Reset();

    // Emulated CPU state
    u32 spill[NUM_REG_GPR];
    u8 N, Z, C, V;
    // Try to keep everything above here in one cache line.

    u8 T;

    //std::array<u32, 64> vfp_banks;

    u8 reschedule;

    void* bb;
    u64 save_host_RSP;
    s32 cycles_remaining;
    u32 final_PC;
    u64 return_RIP;

    ARMul_State* interp_state;
};
static_assert(std::is_pod<JitState>::value, "JitState needs to be POD");

class ARM_Jit final : virtual public ARM_Interface {
public:
    ARM_Jit(PrivilegeMode initial_mode);
    ~ARM_Jit();

    void SetPC(u32 pc) override;
    u32 GetPC() const override;
    u32 GetReg(int index) const override;
    void SetReg(int index, u32 value) override;
    u32 GetVFPReg(int index) const override;
    void SetVFPReg(int index, u32 value) override;
    u32 GetVFPSystemReg(VFPSystemRegister reg) const override;
    void SetVFPSystemReg(VFPSystemRegister reg, u32 value) override;
    u32 GetCPSR() const override;
    void SetCPSR(u32 cpsr) override;
    u32 GetCP15Register(CP15Register reg) override;
    void SetCP15Register(CP15Register reg, u32 value) override;

    void AddTicks(u64 ticks) override;

    void ResetContext(Core::ThreadContext& context, u32 stack_top, u32 entry_point, u32 arg) override;
    void SaveContext(Core::ThreadContext& ctx) override;
    void LoadContext(const Core::ThreadContext& ctx) override;

    void PrepareReschedule() override;
    void ExecuteInstructions(int num_instructions) override;

private:
    std::unordered_map<u32, u8*> basic_blocks;
    std::unique_ptr<JitState> state;
    std::unique_ptr<ARMul_State> interp_state;
    //u32 pc;
};

}
