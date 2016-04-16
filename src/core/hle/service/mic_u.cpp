// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/logging/log.h"

#include "core/hle/kernel/event.h""
#include <core/hle/kernel/shared_memory.h>

#include "core/hle/service/mic_u.h"



////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace MIC_U

namespace MIC_U {

    static Kernel::SharedPtr<Kernel::Event> mic_event;
static Kernel::SharedPtr<Kernel::SharedMemory> shared_memory;
static u32 gain = 0x28;
static bool power = false;
static bool started = false;

static void AllocateBuffer(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 size = cmd_buff[1];
    Handle mem_handle = cmd_buff[3];

    shared_memory = Kernel::g_handle_table.Get<Kernel::SharedMemory>(cmd_buff[3]);
    if (shared_memory) {
        shared_memory->name = "MIC_U:shared_memory";
        memset(shared_memory->GetPointer(), 0, size);
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_MIC, "(STUBBED) called, size=%d", size);
}

static void GetGain(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = gain;

    LOG_WARNING(Service_MIC, "(STUBBED) called");
}

static void StartSampling(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    started = true;

    LOG_WARNING(Service_MIC, "(STUBBED) called");
}

static void StopSampling(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    started = false;

    LOG_WARNING(Service_MIC, "(STUBBED) called");
}

static void SetGain(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    gain = cmd_buff[1];

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_MIC, "(STUBBED) called, gain = %d", gain);
}

static void GetPower(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = power;

    LOG_WARNING(Service_MIC, "(STUBBED) called");
}

static void SetPower(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    // SetMicBias(bool)
    power = cmd_buff[1] & 0xFF;

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_MIC, "(STUBBED) called, power = %u", power);
}

static void GetEventHandle(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[3] = Kernel::g_handle_table.Create(mic_event).MoveFrom();

    LOG_WARNING(Service_MIC, "(STUBBED) called");
}

static void IsSampling(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = started;

    LOG_WARNING(Service_MIC, "(STUBBED) called");
}

const Interface::FunctionInfo FunctionTable[] = {
    {0x00010042, AllocateBuffer,        "AllocateBuffer"},
    {0x00020000, nullptr,               "UnmapSharedMem"},
    {0x00030140, StartSampling,         "StartSampling"},
    {0x00040040, nullptr,               "AdjustSampling"},
    {0x00050000, StopSampling,          "StopSampling"},
    {0x00060000, IsSampling,            "IsSampling"},
    {0x00070000, GetEventHandle,        "GetEventHandle"},
    {0x00080040, SetGain,               "SetGain"},
    {0x00090000, GetGain,               "GetGain"},
    {0x000A0040, SetPower,              "SetPower"},
    {0x000B0000, GetPower,              "GetPower"},
    {0x000C0042, nullptr,               "SetIirFilterMic"},
    {0x000D0040, nullptr,               "SetClamp"},
    {0x000E0000, nullptr,               "GetClamp"},
    {0x000F0040, nullptr,               "SetAllowShellClosed"},
    {0x00100040, nullptr,               "SetClientSDKVersion"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface class

Interface::Interface() {
    Register(FunctionTable);
    shared_memory = nullptr;
    mic_event = Kernel::Event::Create(Kernel::ResetType::OneShot, "MIC_U::mic_event");
}

Interface::~Interface() {
    shared_memory = nullptr;
    mic_event = nullptr;
}

} // namespace
