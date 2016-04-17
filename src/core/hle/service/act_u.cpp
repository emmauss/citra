// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/hle/service/act_u.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace ACT_U

namespace ACT_U {


static u8 nnid[0x11] = {"CitraId"};
static u8 country[3] = {"US"};
static u32 nnid_number = 1;
static u8 unk_13 = '\0';
static u8 unk_19[8] = {};
static u8 time_zone[0x41] = {};
static u8 unk_2c[2] = {};
static u32 country_info = 0;




static void GetAccountDataBlock(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u32 param1 = cmd_buff[1];
    u32 size = cmd_buff[2];
    u32 blk_id = cmd_buff[3];
    VAddr addr = cmd_buff[5];
    switch (blk_id) {
    case 0x8:
        Memory::WriteBlock(addr, nnid, size);
        break;
    case 0xB:
        Memory::WriteBlock(addr, country, size);
        break;
    case 0xC:
        Memory::WriteBlock(addr, (u8*)&nnid_number, size);
        break;
    case 0x13:
        Memory::Write8(addr, unk_13);
        break;
    case 0x19:
        Memory::WriteBlock(addr, unk_19, size);
        break;
    case 0x1E:
        Memory::WriteBlock(addr, time_zone, size);
        break;
    case 0x2C:
        Memory::WriteBlock(addr, unk_2c, size);
        break;
    case 0x2F:
        Memory::WriteBlock(addr, (u8*)&country_info, size);
        break;
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_APT, "(STUBBED) called ");
}

const Interface::FunctionInfo FunctionTable[] = {
    {0x00010084, nullptr,             "Initialize"},
    {0x000600C2, GetAccountDataBlock, "GetAccountDataBlock"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface class

Interface::Interface() {
    Register(FunctionTable);
}

} // namespace
