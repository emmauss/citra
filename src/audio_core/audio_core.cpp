// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "audio_core/audio_core.h"
#include "audio_core/hle/dsp.h"

#include "core/core_timing.h"
#include "core/hle/kernel/event.h"
#include "core/hle/kernel/vm_manager.h"
#include "core/hle/service/dsp_dsp.h"

namespace AudioCore {

/**
 * Audio Ticks occur about every 5 miliseconds.
 */

static int tick_event;                               ///< CoreTiming event
static constexpr u64 audio_frame_ticks = 1310252ull; ///< Units: ARM11 cycles

static void AudioTickCallback(u64 /*userdata*/, int cycles_late) {
    DSP::HLE::Tick();

    // HACK: We're not signaling the interrups when they should be, but just firing them all off together.
    // It should be only (interrupt_id = 2, channel_id = 2) that's signalled here.
    // TODO(merry): Understand when the other interrupts are fired.
    DSP_DSP::SignalAllInterrupts();

    // Reschedule recurrent event
    CoreTiming::ScheduleEvent(audio_frame_ticks - cycles_late, tick_event);
}

/**
 * Initialise Audio
 */
void Init() {
    DSP::HLE::Init();

    tick_event = CoreTiming::RegisterEvent("AudioCore::tick_event", AudioTickCallback);
    CoreTiming::ScheduleEvent(audio_frame_ticks, tick_event);
}

/**
 * Add DSP address spaces to Process's address space.
 */
void AddAddressSpace(Kernel::VMManager& address_space) {
    auto r0_vma = address_space.MapBackingMemory(DSP::HLE::region0_base, reinterpret_cast<u8*>(&DSP::HLE::region0), sizeof(DSP::HLE::SharedMemory), Kernel::MemoryState::IO).MoveFrom();
    address_space.Reprotect(r0_vma, Kernel::VMAPermission::ReadWrite);

    auto r1_vma = address_space.MapBackingMemory(DSP::HLE::region1_base, reinterpret_cast<u8*>(&DSP::HLE::region1), sizeof(DSP::HLE::SharedMemory), Kernel::MemoryState::IO).MoveFrom();
    address_space.Reprotect(r1_vma, Kernel::VMAPermission::ReadWrite);
}

/**
 * Shutdown Audio
 */
void Shutdown() {
    CoreTiming::UnscheduleEvent(tick_event, 0);
    DSP::HLE::Shutdown();
}

} //namespace
