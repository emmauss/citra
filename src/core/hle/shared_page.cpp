// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <cstring>
#include <ctime>

#include "core/core_timing.h"
#include "core/hle/shared_page.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

// 3DS Uses 1900 for Epoch instead of 1970
static constexpr u64_le console_epoch_offset = 2208988800ULL;

namespace SharedPage {

SharedPageDef shared_page;

static int update_time_event;   ///< Time is updated/swapped every hour

void Init() {
    std::memset(&shared_page, 0, sizeof(shared_page));

    shared_page.running_hw = 0x1; // product

    // Some games wait until this value becomes 0x1, before asking running_hw
    shared_page.unknown_value = 0x1;

    update_time_event = CoreTiming::RegisterEvent("SharedPage::update_time_event", UpdateTimeCallback);
    // update now
    CoreTiming::ScheduleEvent(0, update_time_event);
}

static void UpdateTimeCallback(u64 /*userdata*/, int /*cycles_late*/) {
    u32_le next_count = shared_page.date_time_update_counter + 1;

    // 3DS uses 1/1/1900 for Epoch
    time_t plat_time = std::time(nullptr);
    u64_le console_time = (static_cast<u64_le>(plat_time) + console_epoch_offset) * 1000L;

    DateTime* current_time = (next_count & 1) ? &shared_page.date_time_1 : &shared_page.date_time_0;

    current_time->date_time = console_time;
    current_time->tick_rate = (u64_le)CoreTiming::GetClockFrequency();
    current_time->update_tick = CoreTiming::GetTicks();

    // update after changing the opposite time structure to keep updates atomic
    shared_page.date_time_update_counter = next_count;

    // run again in an hour
    CoreTiming::ScheduleEvent((u64_le)CoreTiming::GetClockFrequency() * 3600ULL, update_time_event);
}

} // namespace
