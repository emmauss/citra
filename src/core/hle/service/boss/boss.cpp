// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/logging/log.h"
#include "common/string_util.h"

#include "core/hle/service/service.h"
#include "core/hle/service/boss/boss.h"
#include "core/hle/service/boss/boss_p.h"
#include "core/hle/service/boss/boss_u.h"

namespace Service {
namespace BOSS {

static bool optout_flag = false;

void InitializeSession(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 low = cmd_buff[1];
    u32 hi = cmd_buff[2];
    u64 p = ((u64)hi << 32) | low;

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_BOSS, "(STUBBED) called");
}

void SetOptoutFlag(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    optout_flag = cmd_buff[1] & 0xFF;

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_BOSS, "(STUBBED) called");
}

void GetOptoutFlag(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = optout_flag;
    LOG_WARNING(Service_BOSS, "(STUBBED) called");
}

void GetTaskIdList(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_BOSS, "(STUBBED) called");
}

//SendProperty(nn::boss::PropertyType, unsigned char const*, unsigned int)
void SendProperty(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    PropertyType property_id = static_cast<PropertyType>(cmd_buff[1] & 0xFFFF);
    u32 size = cmd_buff[2];
    u32 addr = cmd_buff[4];

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_BOSS, "(STUBBED) called, id=%d, size=%d, addr=0x%08x", property_id, size, addr);
    Common::Dump(addr, size);
}

//ReceiveProperty(nn::boss::PropertyType, unsigned char *, unsigned int, unsigned int *)
void ReceiveProperty(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    PropertyType property_id = static_cast<PropertyType>(cmd_buff[1] & 0xFFFF);
    u32 size = cmd_buff[2];
    u32 addr = cmd_buff[4];

    for (u32 i = 0; i < size; ++i) {
        Memory::Write8(addr + i, 0);
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_BOSS, "(STUBBED) called, id=%d, size=%d, addr=0x%08x", property_id, size, addr);
}

//nn::boss::GetTaskInfo(
//unsigned long long,
//char const*,
//nn::boss::TaskPolicy *,
//nn::boss::TaskAction *,
//nn::boss::TaskOption *,
//unsigned char)
void GetTaskInfo(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_BOSS, "(STUBBED) called");
}

void Init() {
    using namespace Kernel;

    AddService(new BOSS_P_Interface);
    AddService(new BOSS_U_Interface);
}

void Shutdown() {
}

} // namespace BOSS

} // namespace Service
