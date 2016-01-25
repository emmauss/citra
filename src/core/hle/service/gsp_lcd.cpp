// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/bit_field.h"

#include "core/hle/service/gsp_lcd.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace GSP_LCD

namespace GSP_LCD {

const Interface::FunctionInfo FunctionTable[] = {
    {0x000F0000, nullptr, "PowerOnAllBacklights"},
    {0x00100000, nullptr, "PowerOffAllBacklights"},
    {0x00110040, nullptr, "PowerOnBacklight"},
    {0x00120040, nullptr, "PowerOffBacklight"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface class

Interface::Interface() {
    Register(FunctionTable);
}

} // namespace
