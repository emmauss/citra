// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/hle/service/service.h"
#include "core/hle/service/ir/ir.h"
#include "core/hle/service/ir/ir_rst.h"
#include "core/hle/service/ir/ir_u.h"
#include "core/hle/service/ir/ir_user.h"

#include "core/hle/kernel/event.h"
#include "core/hle/kernel/shared_memory.h"

namespace Service {
namespace IR {

static Kernel::SharedPtr<Kernel::Event> handle_event;
static Kernel::SharedPtr<Kernel::Event> conn_status_event;
static Kernel::SharedPtr<Kernel::SharedMemory> shared_memory;

void GetHandles(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = 0x4000000;
    cmd_buff[3] = Kernel::g_handle_table.Create(Service::IR::shared_memory).MoveFrom();
    cmd_buff[4] = Kernel::g_handle_table.Create(Service::IR::handle_event).MoveFrom();
}

void InitializeIrNopShared(Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_INFO(Service_IR, "(STUBBED) called");
}

void RequireConnection(Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 param1 = cmd_buff[1] & 0xFF;

    conn_status_event->Signal();
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_INFO(Service_IR, "(STUBBED) called: param1 = %u", param1);
}

void Disconnect(Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_INFO(Service_IR, "(STUBBED) called");
}

void GetConnectionStatusEvent(Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[3] = Kernel::g_handle_table.Create(Service::IR::conn_status_event).MoveFrom();

    LOG_INFO(Service_IR, "(STUBBED) called");
}

void FinalizeIrNop(Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_INFO(Service_IR, "(STUBBED) called");
}

void Init() {
    using namespace Kernel;

    AddService(new IR_RST_Interface);
    AddService(new IR_U_Interface);
    AddService(new IR_User_Interface);

    using Kernel::MemoryPermission;
    shared_memory = SharedMemory::Create(0x1000, Kernel::MemoryPermission::ReadWrite,
            Kernel::MemoryPermission::ReadWrite, "IR:SharedMemory");

    // Create event handle(s)
    handle_event  = Event::Create(RESETTYPE_ONESHOT, "IR:HandleEvent");
    conn_status_event = Event::Create(RESETTYPE_ONESHOT, "IR:ConnectionStatusEvent");
}

void Shutdown() {
    shared_memory = nullptr;
    handle_event = nullptr;
    conn_status_event = nullptr;
}

} // namespace IR

} // namespace Service
