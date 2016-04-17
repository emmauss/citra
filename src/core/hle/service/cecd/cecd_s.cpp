// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/hle/service/cecd/cecd.h"
#include "core/hle/service/cecd/cecd_s.h"

namespace Service {
namespace CECD {

// Empty arrays are illegal -- commented out until an entry is added.
const Interface::FunctionInfo FunctionTable[] = {
    {0x000100C2, nullptr,                   "Open"},
    {0x00020042, nullptr,                   "Read"},
    {0x00050042, nullptr,                   "Write"},
    {0x000900C2, SetData,                   "SetData"},
    {0x000E0000, GetCecStateAbbreviated,    "GetCecStateAbbreviated"},
    {0x000F0000, GetCecInfoEventHandle,     "GetCecInfoEventHandle"},
    {0x00100000, GetChangeStateEventHandle, "GetChangeStateEventHandle"},
    {0x00120104, OpenAndReadFile,           "OpenAndReadFile"},
    {0x04020002, GetCecInfoEventHandleSys,  "GetCecInfoEventHandleSys"},
};

CECD_S_Interface::CECD_S_Interface() {
    Register(FunctionTable);
}

} // namespace CECD
} // namespace Service
