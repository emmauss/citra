// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/hle/service/hid/hid.h"
#include "core/hle/service/hid/hid_user.h"

namespace Service {
namespace HID {

const Interface::FunctionInfo FunctionTable[] = {
    {0x00010100, nullptr,                            "SetTouchPanelCalibrateParam"},
    {0x00020000, nullptr,                            "FlushTouchPanelCalibrateParam"},
    {0x00030000, nullptr,                            "ResetTouchPanelCalibrateParam"},
    {0x000400C0, nullptr,                            "SetAccelerometerCalibrateParam"},
    {0x00050000, nullptr,                            "FlushAccelerometerCalibrateParam"},
    {0x00060000, nullptr,                            "ResetAccelerometerCalibrateParam"},
    {0x00070140, nullptr,                            "SetGyroscopeLowCalibrateParam"},
    {0x00080000, nullptr,                            "FlushGyroscopeLowCalibrateParam"},
    {0x00090000, nullptr,                            "ResetGyroscopeLowCalibrateParam"},
    {0x000A0000, GetIPCHandles,                      "GetIPCHandles"},
    {0x000B0000, nullptr,                            "StartAnalogStickCalibration"},
    {0x000C0040, nullptr,                            "StopAnalogStickCalibration"},
    {0x000D01C0, nullptr,                            "SetAnalogStickCalibrateParam"},
    {0x000E0000, nullptr,                            "GetAnalogStickCalibrateParam"},
    {0x000F00C0, nullptr,                            "SetPlusButtonEmulationParams"},
    {0x00100000, nullptr,                            "GetPlusButtonEmulationParams"},
    {0x00110000, EnableAccelerometer,                "EnableAccelerometer"},
    {0x00120000, DisableAccelerometer,               "DisableAccelerometer"},
    {0x00130000, EnableGyroscopeLow,                 "EnableGyroscopeLow"},
    {0x00140000, DisableGyroscopeLow,                "DisableGyroscopeLow"},
    {0x00150000, GetGyroscopeLowRawToDpsCoefficient, "GetGyroscopeLowRawToDpsCoefficient"},
    {0x00160000, GetGyroscopeLowCalibrateParam,      "GetGyroscopeLowCalibrateParam"},
    {0x00170000, GetSoundVolume,                     "GetSoundVolume"},
};

HID_U_Interface::HID_U_Interface() {
    Register(FunctionTable);
}

} // namespace HID
} // namespace Service
