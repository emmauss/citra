// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/logging/log.h"
#include "common/emu_window.h"

#include "core/hle/service/service.h"
#include "core/hle/service/hid/hid.h"
#include "core/hle/service/hid/hid_spvr.h"
#include "core/hle/service/hid/hid_user.h"

#include "core/core.h"
#include "core/arm/arm_interface.h"
#include "core/core_timing.h"
#include "core/hle/kernel/event.h"
#include "core/hle/kernel/shared_memory.h"

#include "video_core/video_core.h"

namespace Service {
namespace HID {

static const int MAX_CIRCLEPAD_POS = 0x9C; ///< Max value for a circle pad position

// Handle to shared memory region designated to HID_User service
static Kernel::SharedPtr<Kernel::SharedMemory> shared_mem;

// Event handles
static Kernel::SharedPtr<Kernel::Event> event_pad_or_touch_1;
static Kernel::SharedPtr<Kernel::Event> event_pad_or_touch_2;
static Kernel::SharedPtr<Kernel::Event> event_accelerometer;
static Kernel::SharedPtr<Kernel::Event> event_gyroscope;
static Kernel::SharedPtr<Kernel::Event> event_debug_pad;

static u32 next_pad_index;
static u32 next_touch_index;
static u32 next_accelerometer_index;
static u32 next_gyroscope_index;

static int enable_accelerometer_count = 0; // positive means enabled
static int enable_gyroscope_count = 0; // positive means enabled

const std::array<Service::HID::PadState, Settings::NativeInput::NUM_INPUTS> pad_mapping = {{
    Service::HID::PAD_A, Service::HID::PAD_B, Service::HID::PAD_X, Service::HID::PAD_Y,
    Service::HID::PAD_L, Service::HID::PAD_R, Service::HID::PAD_ZL, Service::HID::PAD_ZR,
    Service::HID::PAD_START, Service::HID::PAD_SELECT, Service::HID::PAD_NONE,
    Service::HID::PAD_UP, Service::HID::PAD_DOWN, Service::HID::PAD_LEFT, Service::HID::PAD_RIGHT,
    Service::HID::PAD_CIRCLE_UP, Service::HID::PAD_CIRCLE_DOWN, Service::HID::PAD_CIRCLE_LEFT, Service::HID::PAD_CIRCLE_RIGHT,
    Service::HID::PAD_C_UP, Service::HID::PAD_C_DOWN, Service::HID::PAD_C_LEFT, Service::HID::PAD_C_RIGHT
}};


// TODO(peachum):
// Add a method for setting analog input from joystick device for the circle Pad.
//
// This method should:
//     * Be called after both PadButton<Press, Release>().
//     * Be called before PadUpdateComplete()
//     * Set current PadEntry.circle_pad_<axis> using analog data
//     * Set PadData.raw_circle_pad_data
//     * Set PadData.current_state.circle_right = 1 if current PadEntry.circle_pad_x >= 41
//     * Set PadData.current_state.circle_up = 1 if current PadEntry.circle_pad_y >= 41
//     * Set PadData.current_state.circle_left = 1 if current PadEntry.circle_pad_x <= -41
//     * Set PadData.current_state.circle_right = 1 if current PadEntry.circle_pad_y <= -41

void Update() {
    if(shared_mem->base_address == 0) {
        return;
    }

    SharedMem* mem = reinterpret_cast<SharedMem*>(shared_mem->GetPointer());
    const PadState state = VideoCore::g_emu_window->GetPadState();

    mem->pad.current_state.hex = state.hex;
    mem->pad.index = next_pad_index;
    next_pad_index = (next_pad_index + 1) % mem->pad.entries.size();

    // Get the previous Pad state
    u32 last_entry_index = (mem->pad.index - 1) % mem->pad.entries.size();
    PadState old_state = mem->pad.entries[last_entry_index].current_state;

    // Compute bitmask with 1s for bits different from the old state
    PadState changed = { { (state.hex ^ old_state.hex) } };

    // Get the current Pad entry
    PadDataEntry& pad_entry = mem->pad.entries[mem->pad.index];

    // Update entry properties
    pad_entry.current_state.hex = state.hex;
    pad_entry.delta_additions.hex = changed.hex & state.hex;
    pad_entry.delta_removals.hex = changed.hex & old_state.hex;;

    // Set circle Pad
    pad_entry.circle_pad_x = state.circle_left  ? -MAX_CIRCLEPAD_POS :
                              state.circle_right ?  MAX_CIRCLEPAD_POS : 0x0;
    pad_entry.circle_pad_y = state.circle_down  ? -MAX_CIRCLEPAD_POS :
                              state.circle_up    ?  MAX_CIRCLEPAD_POS : 0x0;

    // If we just updated index 0, provide a new timestamp
    if (mem->pad.index == 0) {
        mem->pad.index_reset_ticks_previous = mem->pad.index_reset_ticks;
        mem->pad.index_reset_ticks = (s64)CoreTiming::GetTicks();
    }

    mem->touch.index = next_touch_index;
    next_touch_index = (next_touch_index + 1) % mem->touch.entries.size();

    // Get the current touch entry
    TouchDataEntry& touch_entry = mem->touch.entries[mem->touch.index];
    bool pressed = false;

    std::tie(touch_entry.x, touch_entry.y, pressed) = VideoCore::g_emu_window->GetTouchState();
    touch_entry.valid.Assign(pressed ? 1 : 0);

    // TODO(bunnei): We're not doing anything with offset 0xA8 + 0x18 of HID SharedMemory, which
    // supposedly is "Touch-screen entry, which contains the raw coordinate data prior to being
    // converted to pixel coordinates." (http://3dbrew.org/wiki/HID_Shared_Memory#Offset_0xA8).

    // If we just updated index 0, provide a new timestamp
    if (mem->touch.index == 0) {
        mem->touch.index_reset_ticks_previous = mem->touch.index_reset_ticks;
        mem->touch.index_reset_ticks = (s64)CoreTiming::GetTicks();
    }

    // Signal both handles when there's an update to Pad or touch
    event_pad_or_touch_1->Signal();
    event_pad_or_touch_2->Signal();

    // Update accelerometer
    if (enable_accelerometer_count > 0) {
        mem->accelerometer.index = next_accelerometer_index;
        next_accelerometer_index = (next_accelerometer_index + 1) % mem->accelerometer.entries.size();

        AccelerometerDataEntry& accelerometer_entry = mem->accelerometer.entries[mem->accelerometer.index];
        std::tie(accelerometer_entry.x, accelerometer_entry.y, accelerometer_entry.z)
            = VideoCore::g_emu_window->GetAccelerometerState();

        // Make up "raw" entry
        // TODO(wwylele):
        // From hardware testing, the raw_entry values are approximately,
        // but not exactly, as twice as corresponding entries (or with a minus sign).
        // It may caused by system calibration to the accelerometer.
        // Figure out how it works, or, if no game reads raw_entry,
        // the following three lines can be removed and leave raw_entry unimplemented.
        mem->accelerometer.raw_entry.x = -2 * accelerometer_entry.x;
        mem->accelerometer.raw_entry.z = 2 * accelerometer_entry.y;
        mem->accelerometer.raw_entry.y = -2 * accelerometer_entry.z;

        // If we just updated index 0, provide a new timestamp
        if (mem->accelerometer.index == 0) {
            mem->accelerometer.index_reset_ticks_previous = mem->accelerometer.index_reset_ticks;
            mem->accelerometer.index_reset_ticks = (s64)CoreTiming::GetTicks();
        }

        event_accelerometer->Signal();
    }

    // Update gyroscope
    if (enable_gyroscope_count > 0) {
        mem->gyroscope.index = next_gyroscope_index;
        next_gyroscope_index = (next_gyroscope_index + 1) % mem->gyroscope.entries.size();

        GyroscopeDataEntry& gyroscope_entry = mem->gyroscope.entries[mem->gyroscope.index];
        std::tie(gyroscope_entry.x, gyroscope_entry.y, gyroscope_entry.z)
            = VideoCore::g_emu_window->GetGyroscopeState();

        // Make up "raw" entry
        mem->gyroscope.raw_entry.x = gyroscope_entry.x;
        mem->gyroscope.raw_entry.z = -gyroscope_entry.y;
        mem->gyroscope.raw_entry.y = gyroscope_entry.z;

        // If we just updated index 0, provide a new timestamp
        if (mem->gyroscope.index == 0) {
            mem->gyroscope.index_reset_ticks_previous = mem->gyroscope.index_reset_ticks;
            mem->gyroscope.index_reset_ticks = (s64)CoreTiming::GetTicks();
        }

        event_gyroscope->Signal();
    }
}

void GetIPCHandles(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = 0x14000000; // IPC Command Structure translate-header
    // TODO(yuriks): Return error from SendSyncRequest is this fails (part of IPC marshalling)
    cmd_buff[3] = Kernel::g_handle_table.Create(Service::HID::shared_mem).MoveFrom();
    cmd_buff[4] = Kernel::g_handle_table.Create(Service::HID::event_pad_or_touch_1).MoveFrom();
    cmd_buff[5] = Kernel::g_handle_table.Create(Service::HID::event_pad_or_touch_2).MoveFrom();
    cmd_buff[6] = Kernel::g_handle_table.Create(Service::HID::event_accelerometer).MoveFrom();
    cmd_buff[7] = Kernel::g_handle_table.Create(Service::HID::event_gyroscope).MoveFrom();
    cmd_buff[8] = Kernel::g_handle_table.Create(Service::HID::event_debug_pad).MoveFrom();
}

void EnableAccelerometer(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    ++enable_accelerometer_count;
    event_accelerometer->Signal();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_DEBUG(Service_HID, "called");
}

void DisableAccelerometer(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    --enable_accelerometer_count;
    event_accelerometer->Signal();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_DEBUG(Service_HID, "called");
}

void EnableGyroscopeLow(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    ++enable_gyroscope_count;
    event_gyroscope->Signal();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_DEBUG(Service_HID, "called");
}

void DisableGyroscopeLow(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    --enable_gyroscope_count;
    event_gyroscope->Signal();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_DEBUG(Service_HID, "called");
}

void GetGyroscopeLowRawToDpsCoefficient(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    f32 coef = VideoCore::g_emu_window->GetGyroscopeRawToDpsCoefficient();
    memcpy(&cmd_buff[2], &coef, 4);
}

void GetGyroscopeLowCalibrateParam(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw;

    const s16 param_unit = 6700; // an approximate value taken from hw
    GyroscopeCalibrateParam param = {
        { 0, param_unit, -param_unit },
        { 0, param_unit, -param_unit },
        { 0, param_unit, -param_unit },
    };
    memcpy(&cmd_buff[2], &param, sizeof(param));

    LOG_WARNING(Service_HID, "(STUBBED) called");
}

void GetSoundVolume(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    const u8 volume = 0x3F; // TODO(purpasmart): Find out if this is the max value for the volume

    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = volume;

    LOG_TRACE(Service_HID, "(STUBBED) ca lled");
}

void CheckHidRead(u32 address, u32 size) {
    static bool out = false;
    if(!out) {
        return;
    }
    if (shared_mem->base_address == 0) {
        return;
    }
    Kernel::Thread* thread = Kernel::GetCurrentThread();

    u32 ptr = (u32)shared_mem->base_address;

    if((address >= ptr) && (address < (ptr + shared_mem->size))) {
        std::string s;
        u32 index = (address - ptr) / 4;
        switch (index) {
        case 0x00: s = "pad->index_reset_ticks low"; break;
        case 0x01: s = "pad->index_reset_ticks hi"; break;
        case 0x02: s = "pad->index_reset_ticks_previous low"; break;
        case 0x03: s = "pad->index_reset_ticks_previous hi"; break;
        case 0x04: s = "pad->index"; break;
        case 0x07: s = "pad->current_state"; break;
        case 0x08: s = "pad->raw_circle_pad_data"; break;

        case 0x0A: s = "pad->entries[0]->current_state"; break;
        case 0x0B: s = "pad->entries[0]->delta_additions"; break;
        case 0x0C: s = "pad->entries[0]->delta_removals"; break;
        case 0x0D: s = "pad->entries[0]->circle_pad_x/circle_pad_y"; break;

        case 0x0E: s = "pad->entries[1]->current_state"; break;
        case 0x0F: s = "pad->entries[1]->delta_additions"; break;
        case 0x10: s = "pad->entries[1]->delta_removals"; break;
        case 0x11: s = "pad->entries[1]->circle_pad_x/circle_pad_y"; break;

        case 0x12: s = "pad->entries[2]->current_state"; break;
        case 0x13: s = "pad->entries[2]->delta_additions"; break;
        case 0x14: s = "pad->entries[2]->delta_removals"; break;
        case 0x15: s = "pad->entries[2]->circle_pad_x/circle_pad_y"; break;

        case 0x16: s = "pad->entries[3]->current_state"; break;
        case 0x17: s = "pad->entries[3]->delta_additions"; break;
        case 0x18: s = "pad->entries[3]->delta_removals"; break;
        case 0x19: s = "pad->entries[3]->circle_pad_x/circle_pad_y"; break;

        case 0x1A: s = "pad->entries[4]->current_state"; break;
        case 0x1B: s = "pad->entries[4]->delta_additions"; break;
        case 0x1C: s = "pad->entries[4]->delta_removals"; break;
        case 0x1D: s = "pad->entries[4]->circle_pad_x/circle_pad_y"; break;


        case 0x2A: s = "touch->index_reset_ticks low"; break;
        case 0x2B: s = "touch->index_reset_ticks hi"; break;
        case 0x2C: s = "touch->index_reset_ticks_previous low"; break;
        case 0x2D: s = "touch->index_reset_ticks_previous hi"; break;
        case 0x2E: s = "touch->index"; break;

        case 0x42: s = "accelerometer->index_reset_ticks low"; break;
        case 0x43: s = "accelerometer->index_reset_ticks hi"; break;
        case 0x44: s = "accelerometer->index_reset_ticks_previous low"; break;
        case 0x45: s = "accelerometer->index_reset_ticks_previous hi"; break;
        case 0x46: s = "accelerometer->index"; break;

        case 0x4A: s = "accelerometer->entries[0]->x/y"; break;
        case 0x4B: s = "accelerometer->entries[0]->z"; break;

        default:
            s = Common::StringFromFormat("0x%02X", index);
        }
        LOG_WARNING(Service_HID, "Reading from HID: size=%d, pc=0x%08X, thread=%d, %s", size,
            Core::g_app_core->GetPC(), thread->GetObjectId(), s.c_str());
    }

}

void Init() {
    using namespace Kernel;

    AddService(new HID_U_Interface);
    AddService(new HID_SPVR_Interface);

    using Kernel::MemoryPermission;
    shared_mem = SharedMemory::Create(0x1000, MemoryPermission::ReadWrite,
            MemoryPermission::Read, "HID:SharedMem");

    next_pad_index = 0;
    next_touch_index = 0;

    // Create event handles
    event_pad_or_touch_1 = Event::Create(ResetType::OneShot, "HID:EventPadOrTouch1");
    event_pad_or_touch_2 = Event::Create(ResetType::OneShot, "HID:EventPadOrTouch2");
    event_accelerometer  = Event::Create(ResetType::OneShot, "HID:EventAccelerometer");
    event_gyroscope      = Event::Create(ResetType::OneShot, "HID:EventGyroscope");
    event_debug_pad      = Event::Create(ResetType::OneShot, "HID:EventDebugPad");
}

void Shutdown() {
    shared_mem = nullptr;
    event_pad_or_touch_1 = nullptr;
    event_pad_or_touch_2 = nullptr;
    event_accelerometer = nullptr;
    event_gyroscope = nullptr;
    event_debug_pad = nullptr;
}

} // namespace HID

} // namespace Service
