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
#include <intrin.h>
#include <random>

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

    printf("JIT self-test in progress:\n");
    Memory::MapMemoryRegion(0, 4096 * 2, new u8[4096 * 2]);
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int<u32> rand;
    for (int j = 0; j < 2000; j++) {
        jit.ClearCache();
        interp.ClearCache();

        u32 initial_regs[15];

        for (int i = 0; i < 15; i++) {
            u32 val = rand(mt);
            interp.SetReg(i, val);
            jit.SetReg(i, val);
            initial_regs[i] = val;
        }
        interp.SetCPSR(0x000001d0);
        jit.SetCPSR(0x000001d0);

        addr_ptr = 0;
        int inst_ptr = addr_ptr;

        interp.SetPC(addr_ptr);
        jit.SetPC(addr_ptr);

        constexpr int NUM_INST = 1024;

        for (int i = 0; i < NUM_INST; i++) {
            u32 inst;

            if (rand(mt) % 10 < 2) {
                inst = (rand(mt) % 0xE) << 28;
            } else {
                inst = 0b1110 << 28;
            }

            int opcode;
            //if (rand() % 2 == 0) opcode = 0b00011010; else opcode = 0b00011011; // mov and movs only.
            do {
                opcode = rand(mt) & 0b00111111;
            } while (opcode == 0b00010010 || opcode == 0b00010000 || opcode == 0b00010100 || opcode == 0b00010110 || opcode == 0b00110010 || opcode == 0b00110110 || opcode == 0b00110100 || opcode == 0b00110000);

            inst |= (opcode << 20);
            inst |= ((rand(mt) % 15) << 18);
            inst |= ((rand(mt) % 15) << 12);
            inst |= ((rand(mt) & 0xF) << 8);
            inst |= ((rand(mt) % 8) << 4);
            inst |= (rand(mt) & 0xF);

            Memory::Write32(addr_ptr, inst);
            addr_ptr += 4;
        }

        Memory::Write32(addr_ptr, 0b11100011001000000000111100000000);

        interp.ExecuteInstructions(NUM_INST+1);
        jit.ExecuteInstructions(NUM_INST+1);

        bool pass = interp.GetCPSR() == jit.GetCPSR();

        for (int i = 0; i <= 15; i++) {
            if (interp.GetReg(i) != jit.GetReg(i)) pass = false;
        }

        if (j % 1 == 0) printf("%i\r", j);

        if (!pass) {
            std::string disasm;
            printf("\n");
            for (int i = 0; i < NUM_INST; i++) {
                disasm = ARM_Disasm::Disassemble(inst_ptr, Memory::Read32(inst_ptr + i*4));
                printf("%s\n", disasm.c_str());
            }
            for (int i = 0; i <= 15; i++) {
                printf("%4i: %08x %08x %s\n", i, interp.GetReg(i), jit.GetReg(i), interp.GetReg(i) != jit.GetReg(i) ? "*" : "");
            }
            printf("CPSR: %08x %08x %s\n", interp.GetCPSR(), jit.GetCPSR(), interp.GetCPSR() != jit.GetCPSR() ? "*" : "");
            printf("\a\a\a\a\a\a");

            printf("\nInterpreter walkthrough:\n");
            interp.ClearCache();
            interp.SetPC(0);
            interp.SetCPSR(0x000001d0);
            for (int i = 0; i < 15; i++) {
                interp.SetReg(i, initial_regs[i]);
                printf("%4i: %08x\n", i, interp.GetReg(i));
            }
            for (int inst = 0; inst < NUM_INST; inst++) {
                printf("%s\n", ARM_Disasm::Disassemble(inst_ptr, Memory::Read32(inst*4)).c_str());
                interp.Step();
                for (int i = 0; i <= 15; i++) {
                    printf("%4i: %08x\n", i, interp.GetReg(i));
                }
                printf("CPSR: %08x\n", interp.GetCPSR());
            }

            __debugbreak();
            jit.DebugRun(0, NUM_INST+1);
            printf("Self-test failed.\n");
            system("pause");
            exit(-2);
        }
    }
    jit.ClearCache();
    interp.ClearCache();
    Memory::UnmapRegion(0, 4096 * 2);
    printf("Self-test passsed, proceeding.\n");

    Loader::ResultStatus load_result = Loader::LoadFile(boot_filename);
    if (Loader::ResultStatus::Success != load_result) {
        LOG_CRITICAL(Frontend, "Failed to load ROM (Error %i)!", load_result);
        return -1;
    }

    while (emu_window->IsOpen()) {
        Core::RunLoop();
    }

    System::Shutdown();

    delete emu_window;

    MicroProfileShutdown();

    return 0;
}
