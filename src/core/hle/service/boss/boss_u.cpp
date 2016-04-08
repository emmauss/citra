// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/hle/service/boss/boss.h"
#include "core/hle/service/boss/boss_u.h"

namespace Service {
namespace BOSS {

const Interface::FunctionInfo FunctionTable[] = {
    {0x00010082, InitializeSession,     "InitializeSession"},
    {0x00020100, nullptr,               "RegisterStorage"},
    {0x00040000, nullptr,               "GetStorageInfo"},
    {0x00070000, nullptr,               "GetNewArrivalFlag"},
    {0x00090040, SetOptoutFlag,         "SetOptoutFlag"},
    {0x000A0000, GetOptoutFlag,         "GetOptoutFlag"},
    {0x000B00C2, nullptr,               "RegisterTask"},
    {0x000E0000, GetTaskIdList,         "GetTaskIdList"},
    {0x000C0082, nullptr,               "UnregisterTask"},
    {0x00100102, nullptr,               "GetNsDataIdList"},
    {0x00140082, SendProperty,          "SendProperty"},
    {0x00160082, ReceiveProperty,       "ReceiveProperty"},
    {0x00180082, nullptr,               "UpdateTaskCount"},
    {0x001C0042, nullptr,               "StartTask"},
    {0x001E0042, nullptr,               "CancelTask"},
    {0x00250082, nullptr,               "GetTaskInfo"},
    {0x00260040, nullptr,               "DeleteNsData"},
    {0x002700C2, nullptr,               "GetNsDataHeaderInfo"},
    {0x00280102, nullptr,               "ReadNsData"},
    {0x00330042, nullptr,               "StartTaskBgImmediate"},
};

BOSS_U_Interface::BOSS_U_Interface() {
    Register(FunctionTable);
}

} // namespace BOSS
} // namespace Service
