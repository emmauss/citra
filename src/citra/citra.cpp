// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <string>
#include <thread>
#include <iostream>

// This needs to be included before getopt.h because the latter #defines symbols used by it
#include "common/microprofile.h"

#ifdef _MSC_VER
#include <getopt.h>
#else
#include <unistd.h>
#include <getopt.h>
#endif

#include "common/logging/log.h"
#include "common/logging/backend.h"
#include "common/logging/filter.h"

#include "core/settings.h"
#include "core/system.h"
#include "core/core.h"
#include "core/gdbstub/gdbstub.h"
#include "core/loader/loader.h"

#include "citra/config.h"
#include "citra/emu_window/emu_window_glfw.h"

#include "video_core/video_core.h"

#include "core/arm/disassembler/arm_disasm.h"
#include "core/arm/dyncom/arm_dyncom.h"
#include "core/arm/dyncom/jit/jit.h"
#include "core/memory.h"
#include "core/memory_setup.h"

static void PrintHelp()
{
    std::cout << "Usage: citra <filename>" << std::endl;
}

/// Application entry point
int main(int argc, char **argv) {
    int option_index = 0;
    std::string boot_filename;
    static struct option long_options[] = {
        { "help", no_argument, 0, 'h' },
        { 0, 0, 0, 0 }
    };

    while (optind < argc) {
        char arg = getopt_long(argc, argv, ":h", long_options, &option_index);
        if (arg != -1) {
            switch (arg) {
            case 'h':
                PrintHelp();
                return 0;
            }
        } else {
            boot_filename = argv[optind];
            optind++;
        }
    }

    Log::Filter log_filter(Log::Level::Debug);
    Log::SetFilter(&log_filter);

    MicroProfileOnThreadCreate("EmuThread");

    if (boot_filename.empty()) {
        LOG_CRITICAL(Frontend, "Failed to load ROM: No ROM specified");
        return -1;
    }

    Config config;
    log_filter.ParseFilterString(Settings::values.log_filter);

    GDBStub::ToggleServer(Settings::values.use_gdbstub);
    GDBStub::SetServerPort(static_cast<u32>(Settings::values.gdbstub_port));

    EmuWindow_GLFW* emu_window = new EmuWindow_GLFW;

    VideoCore::g_hw_renderer_enabled = Settings::values.use_hw_renderer;
    VideoCore::g_shader_jit_enabled = Settings::values.use_shader_jit;

    System::Init(emu_window);

    /////////////////// TESTS ///////////////////

    Jit::ARM_Jit jit(PrivilegeMode::USER32MODE);
    ARM_DynCom interp(PrivilegeMode::USER32MODE);

    int addr_ptr = 0;

    srand(time(nullptr));

    Memory::MapMemoryRegion(0, 409600, new u8[409600]);

    for (int j = 0; j < 10000; j++) {

        for (int i = 0; i < 15; i++) {
            u32 val = rand();
            interp.SetReg(i, val);
            jit.SetReg(i, val);
        }

        int inst_ptr = addr_ptr;

        interp.SetPC(addr_ptr);
        jit.SetPC(addr_ptr);

        for (int i = 0; i < 2; i++) {
            u32 inst = 0b1110 << 28;

            int opcode;
            do {
                opcode = rand() & 0b00011111;
            } while (opcode == 0 || opcode == 0b00010010 || opcode == 0b00010000 || opcode == 0b00010011 || opcode == 0b00010001 || (opcode & 0b11111100) == 0b00010100 || opcode == 16 || opcode == 14);

            inst |= (opcode << 20);
            inst |= ((rand() % 15) << 18);
            inst |= ((rand() % 15) << 12);
            inst |= ((rand() & 0xF) << 8);
            inst |= ((rand() % 8) << 4);
            inst |= (rand() & 0xF);

            Memory::Write32(addr_ptr, inst);
            addr_ptr += 4;
        }

        Memory::Write32(addr_ptr, 0b11100011001000000000111100000000);

        interp.ExecuteInstructions(3);
        jit.ExecuteInstructions(3);

        bool pass = interp.GetCPSR() == jit.GetCPSR();

        for (int i = 0; i <= 15; i++) {
            if (interp.GetReg(i) != jit.GetReg(i)) pass = false;
        }

        printf("%i\r", j);

        if (!pass) {
            std::string disasm;
            printf("\n");
            disasm = ARM_Disasm::Disassemble(inst_ptr, Memory::Read32(inst_ptr));
            printf("%s\n", disasm.c_str());
            disasm = ARM_Disasm::Disassemble(inst_ptr, Memory::Read32(inst_ptr+4));
            printf("%s\n", disasm.c_str());
            for (int i = 0; i <= 15; i++) {
                printf("%4i: %08x %08x %s\n", i, interp.GetReg(i), jit.GetReg(i), interp.GetReg(i) != jit.GetReg(i) ? "*" : "");
            }
            printf("CPSR: %08x %08x %s\n", interp.GetCPSR(), jit.GetCPSR(), interp.GetCPSR() != jit.GetCPSR() ? "*" : "");

            system("pause");
        }
    }

    /*
    Loader::ResultStatus load_result = Loader::LoadFile(boot_filename);
    if (Loader::ResultStatus::Success != load_result) {
        LOG_CRITICAL(Frontend, "Failed to load ROM (Error %i)!", load_result);
        return -1;
    }

    while (emu_window->IsOpen()) {
        Core::RunLoop();
    }
    */

    System::Shutdown();

    delete emu_window;

    MicroProfileShutdown();

    return 0;
}
