// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/logging/log.h"

#include "core/hle/kernel/event.h"

#include "core/hle/service/ac_u.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace AC_U

namespace AC_U {

struct AcConfig {
    u8 unknown[512];
};

enum class AcApType {
    AC_AP_TYPE_NONE = 0,
    AC_AP_TYPE_USER_SETTING_1 = 1 << 0,
    AC_AP_TYPE_USER_SETTING_2 = 1 << 1,
    AC_AP_TYPE_USER_SETTING_3 = 1 << 2,
    AC_AP_TYPE_NINTENDO_WIFI_USB_CONNECTOR = 1 << 3,
    AC_AP_TYPE_NINTENDO_ZONE = 1 << 4,
    AC_AP_TYPE_WIFI_STATION = 1 << 5,
    AC_AP_TYPE_FREESPOT = 1 << 6,
    AC_AP_TYPE_HOTSPOT = 1 << 7,

    NN_AC_AP_TYPE_ALL = 0x7FFFFFFF
};

static AcApType ap_type = AcApType::AC_AP_TYPE_NONE;
static bool ac_connected = false;

static Kernel::SharedPtr<Kernel::Event> close_event = nullptr;
static Kernel::SharedPtr<Kernel::Event> connect_event = nullptr;
static Kernel::SharedPtr<Kernel::Event> disconnect_event = nullptr;

static void CreateDefaultConfig(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

static void ConnectAsync(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    // cmd_buff[6] -> AcConfig*
    connect_event = Kernel::g_handle_table.Get<Kernel::Event>(cmd_buff[4]);
    if (connect_event) {
        connect_event->name = "AC_U:connect_event";
        connect_event->Signal();
        ac_connected = true;
    }
    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

static void GetConnectResult(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

/**
 * AC_U::GetWifiStatus service function
 *  Outputs:
 *      1 : Result of function, 0 on success, otherwise error code
 *      2 : Output connection type, 0 = none, 1 = Old3DS Internet, 2 = New3DS Internet.
 */
static void GetConnectingApType(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = static_cast<u32>(ap_type); // Connection type set to none

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

static void GetInfraPriority(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    //cmd_buff[1] AcConfig*

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = 0;

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

static void RegisterDisconnectEvent(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    // cmd_buff[6] -> AcConfig*
    disconnect_event = Kernel::g_handle_table.Get<Kernel::Event>(cmd_buff[4]);
    if (disconnect_event) {
        disconnect_event->name = "AC_U:disconnect_event";
    }
    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}


/**
 * AC_U::IsConnected service function
 *  Outputs:
 *      1 : Result of function, 0 on success, otherwise error code
 *      2 : bool, is connected
 */
static void IsConnected(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = ac_connected;

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

/**
 * AC_U::SetClientVersion service function
 *  Inputs:
 *      1 : Version
 *  Outputs:
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void SetClientVersion(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 version = cmd_buff[1];
    self->SetVersion(version);

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called, version=0x%08X", version);
}
/**
 * AC_U::CloseAsync service function
 *  Inputs:
 *      4 : Event handle, should be signalled when AC connection is closed
 *  Outputs:
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void CloseAsync(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    close_event = Kernel::g_handle_table.Get<Kernel::Event>(cmd_buff[4]);
    if (close_event) {
        close_event->name = "AC_U:close_event";
        close_event->Signal();
        ac_connected = false;
    }
    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

static void GetCloseResult(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

static void GetLastErrorCode(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

static void DebugSetNetworkSetting1(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_AC, "(STUBBED) called");
}

const Interface::FunctionInfo FunctionTable[] = {
    {0x00010000, CreateDefaultConfig,     "CreateDefaultConfig"},
    {0x00040006, ConnectAsync,            "ConnectAsync"},
    {0x00050002, GetConnectResult,        "GetConnectResult"},
    {0x000600C6, DebugSetNetworkSetting1, "DebugSetNetworkSetting1"},
    {0x00080004, CloseAsync,              "CloseAsync"},
    {0x00090002, GetCloseResult,          "GetCloseResult"},
    {0x000A0000, GetLastErrorCode,        "GetLastErrorCode"},
    {0x000D0000, GetConnectingApType,     "GetConnectingApType"},
    {0x000E0042, nullptr,                 "GetConnectingAccessPoint"},
    {0x00100042, nullptr,                 "GetCurrentNZoneInfo"},
    {0x00110042, nullptr,                 "GetNZoneApNumService"},
    {0x00240042, nullptr,                 "AddDenyApType"},
    {0x00270002, GetInfraPriority,        "GetInfraPriority"},
    {0x002D0082, nullptr,                 "SetRequestEulaVersion"},
    {0x00300004, RegisterDisconnectEvent, "RegisterDisconnectEvent"},
    {0x003C0042, nullptr,                 "GetAPSSIDList"},
    {0x003E0042, IsConnected,             "IsConnected"},
    {0x00400042, SetClientVersion,        "SetClientVersion"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface class

Interface::Interface() {
    Register(FunctionTable);
}

Interface::~Interface() {
    close_event = nullptr;
    connect_event = nullptr;
    disconnect_event = nullptr;
}

} // namespace
