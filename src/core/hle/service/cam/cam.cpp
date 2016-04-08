// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/logging/log.h"

#include "core/hle/kernel/event.h"
#include "core/hle/service/cam/cam.h"
#include "core/hle/service/cam/cam_c.h"
#include "core/hle/service/cam/cam_q.h"
#include "core/hle/service/cam/cam_s.h"
#include "core/hle/service/cam/cam_u.h"
#include "core/hle/service/service.h"


namespace Service {
namespace CAM {

static u32 transfer_bytes = 5 * 1024;
static CameraSelect camera_selected;
static bool activated = false;
static bool driver_initialized = false;
static bool camera_capture = false;
static FrameRate frame_rate;
static u8 port;

static Kernel::SharedPtr<Kernel::Event> completion_event_cam1;
static Kernel::SharedPtr<Kernel::Event> completion_event_cam2;
static Kernel::SharedPtr<Kernel::Event> interrupt_buffer_error_event1;
static Kernel::SharedPtr<Kernel::Event> interrupt_buffer_error_event2;
static Kernel::SharedPtr<Kernel::Event> vsync_interrupt_event1;
static Kernel::SharedPtr<Kernel::Event> vsync_interrupt_event2;

#pragma pack(push, 1)
typedef struct tagBITMAPFILEHEADER {
    u16    bfType;
    u32    bfSize;
    u16    bfReserved1;
    u16    bfReserved2;
    u32    bfOffBits;
} BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER{
    u32  biSize;
    long   biWidth;
    long   biHeight;
    u16   biPlanes;
    u16   biBitCount;
    u32  biCompression;
    u32  biSizeImage;
    long   biXPelsPerMeter;
    long   biYPelsPerMeter;
    u32  biClrUsed;
    u32  biClrImportant;
} BITMAPINFOHEADER;

#pragma pack(pop)

char* load(const char* file, long& w, long& hh) {

    FILE* f = fopen(file, "rb");
    if(f) {
        BITMAPFILEHEADER h;
        fread(&h, sizeof(h), 1, f);
        BITMAPINFOHEADER info;
        fread(&info, sizeof(info), 1, f);
        fseek(f, h.bfOffBits, SEEK_SET);

        w = info.biWidth;
        hh = info.biHeight;
        char* img = new char[w * hh * 3];
        fread(img, info.biWidth * info.biHeight * 3, 1, f);
        fclose(f);
        return img;
    }
    return nullptr;
}

void SignalVblankInterrupt() {
    if (driver_initialized && camera_capture) {
        static u32 cycles = 0;
        cycles = (cycles + 1) % 60;
        bool signal = false;

        switch (frame_rate) {
        case FrameRate::Rate_30:
        case FrameRate::Rate_30_To_5:
        case FrameRate::Rate_30_To_10:
            signal = (cycles % 2) == 0;
            break;
        case FrameRate::Rate_20:
        case FrameRate::Rate_20_To_5:
        case FrameRate::Rate_20_To_10:
            signal = (cycles % 3) == 0;
            break;
        case FrameRate::Rate_15:
        case FrameRate::Rate_15_To_5:
        case FrameRate::Rate_15_To_2:
        case FrameRate::Rate_15_To_10:
            signal = (cycles % 4) == 0;
            break;
        case FrameRate::Rate_10:
            signal = (cycles % 6) == 0;
            break;
        case FrameRate::Rate_5:
        case FrameRate::Rate_8_5:
            signal = (cycles % 12) == 0;
            break;
        }

        if (signal) {
            Kernel::Event* vsync_interrupt_event = (Port)port == Port::Cam2 ?
                vsync_interrupt_event2.get() : vsync_interrupt_event1.get();
            vsync_interrupt_event->Signal();
        }
    }
}

void StartCapture(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    port = cmd_buff[1] & 0xFF;

    cmd_buff[0] = IPC::MakeHeader(0x1, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    camera_capture = true;

    Kernel::Event* completion_event = (Port)port == Port::Cam2 ?
        completion_event_cam2.get() : completion_event_cam1.get();

    completion_event->Signal();

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d", port);
}

void StopCapture(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 port = cmd_buff[1] & 0xFF;

    cmd_buff[0] = IPC::MakeHeader(0x2, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    camera_capture = false;

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d", port);
}

void IsBusy(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 port = cmd_buff[1] & 0xFF;

    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = 0; // Not busy

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d", port);
}

void ClearBuffer(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 port = cmd_buff[1] & 0xFF;

    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = 0; // Not busy

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d", port);
}

void GetVsyncInterruptEvent(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 port = cmd_buff[1] & 0xFF;

    Kernel::Event* vsync_interrupt_event = (Port)port == Port::Cam2 ?
        vsync_interrupt_event2.get() : vsync_interrupt_event1.get();

    cmd_buff[0] = IPC::MakeHeader(0x5, 1, 2);
    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = IPC::MoveHandleDesc();
    cmd_buff[3] = Kernel::g_handle_table.Create(vsync_interrupt_event).MoveFrom();

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d", port);
}

void GetBufferErrorInterruptEvent(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 port = cmd_buff[1] & 0xFF;

    Kernel::Event* interrupt_buffer_error_event = (Port)port == Port::Cam2 ?
        interrupt_buffer_error_event2.get() : interrupt_buffer_error_event1.get();

    cmd_buff[0] = IPC::MakeHeader(0x6, 1, 2);
    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = IPC::MoveHandleDesc();
    cmd_buff[3] = Kernel::g_handle_table.Create(interrupt_buffer_error_event).MoveFrom();

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d", port);
}

void SetReceiving(Service::Interface* self) {
    u32* cmd_buff  = Kernel::GetCommandBuffer();

    VAddr dest     = cmd_buff[1];
    u8 port        = cmd_buff[2] & 0xFF;
    u32 image_size = cmd_buff[3];
    u16 trans_unit = cmd_buff[4] & 0xFFFF;

    Kernel::Event* completion_event = (Port)port == Port::Cam2 ?
            completion_event_cam2.get() : completion_event_cam1.get();

    completion_event->Signal();

    /*
    long w, h;
    char* data = load("C:\\Downloads\\sams_cert.bmp", w, h);
    char* head = data;
    if (data) {
        for (u32 i = 0; i < w*h; ++i) {
            u16 r = (*(data++)>>3) & 0xFF;
            u16 g = (*(data++)>>2) & 0xFF;
            u16 b = (*(data++)>>3) & 0xFF;
            Memory::Write16(dest + i * 2, ((b & 0x1F) << 11) | ((g & 0x3f) << 5) | r & 0x1F);
        }
        delete[] head;
    }
    */
    cmd_buff[0] = IPC::MakeHeader(0x7, 1, 2);
    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = IPC::MoveHandleDesc();
    cmd_buff[3] = Kernel::g_handle_table.Create(completion_event).MoveFrom();

    LOG_WARNING(Service_CAM, "(STUBBED) called, addr=0x%X, port=%d, image_size=%d, trans_unit=%d",
            dest, port, image_size, trans_unit);
}

void SetTransferLines(Service::Interface* self) {
    u32* cmd_buff      = Kernel::GetCommandBuffer();

    u8  port           = cmd_buff[1] & 0xFF;
    u16 transfer_lines = cmd_buff[2] & 0xFFFF;
    u16 width          = cmd_buff[3] & 0xFFFF;
    u16 height         = cmd_buff[4] & 0xFFFF;

    cmd_buff[0] = IPC::MakeHeader(0x9, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d, lines=%d, width=%d, height=%d",
            port, transfer_lines, width, height);
}

void GetMaxLines(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    s16 width  = cmd_buff[1] & 0xFFFF;
    s16 height = cmd_buff[2] & 0xFFFF;

    cmd_buff[0] = IPC::MakeHeader(0xA, 2, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = transfer_bytes / (2 * width);

    LOG_WARNING(Service_CAM, "(STUBBED) called, width=%d, height=%d, lines = %d",
            width, height, cmd_buff[2]);
}


void SetTransferBytes(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 port = cmd_buff[1] & 0xFF;
    transfer_bytes = cmd_buff[2];
    // u16 w, h in 3, 4
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d, transfer_bytes=%d", port, transfer_bytes);
}

void GetTransferBytes(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 port = cmd_buff[1] & 0xFF;

    cmd_buff[0] = IPC::MakeHeader(0xC, 2, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = transfer_bytes;

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d", port);
}

void SetTrimming(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8   port = cmd_buff[1] & 0xFF;
    bool trim = (cmd_buff[2] & 0xFF) != 0;

    cmd_buff[0] = IPC::MakeHeader(0xE, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d, trim=%d", port, trim);
}

void SetTrimmingParams(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8  port = cmd_buff[1] & 0xFF;
    s16 x_start = cmd_buff[2] & 0xFFFF;
    s16 y_start = cmd_buff[3] & 0xFFFF;
    s16 x_end = cmd_buff[4] & 0xFFFF;
    s16 y_end = cmd_buff[5] & 0xFFFF;

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d, x_start=%d, y_start=%d, x_end=%d, y_end=%d",
        port, x_start, y_start, x_end, y_end);
}

void SetTrimmingParamsCenter(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8  port  = cmd_buff[1] & 0xFF;
    s16 trimW = cmd_buff[2] & 0xFFFF;
    s16 trimH = cmd_buff[3] & 0xFFFF;
    s16 camW  = cmd_buff[4] & 0xFFFF;
    s16 camH  = cmd_buff[5] & 0xFFFF;

    cmd_buff[0] = IPC::MakeHeader(0x12, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, port=%d, trimW=%d, trimH=%d, camW=%d, camH=%d",
            port, trimW, trimH, camW, camH);
}

void Activate(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    camera_selected = static_cast<CameraSelect>(cmd_buff[1] & 0xFF);

    activated = (camera_selected != CameraSelect::None);

    cmd_buff[0] = IPC::MakeHeader(0x13, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, cam_select=%d", camera_selected);
}

void FlipImage(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 cam_select = cmd_buff[1] & 0xFF;
    u8 flip       = cmd_buff[2] & 0xFF;
    u8 context    = cmd_buff[3] & 0xFF;

    cmd_buff[0] = IPC::MakeHeader(0x1D, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, cam_select=%d, flip=%d, context=%d",
            cam_select, flip, context);
}

void SetDetailSize(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 cam_select = cmd_buff[1] & 0xFF;
    s16 width = cmd_buff[2] & 0xFFFF;
    s16 height = cmd_buff[3] & 0xFFFF;
    s16 cropX0 = cmd_buff[4] & 0xFFFF;
    s16 cropY0 = cmd_buff[5] & 0xFFFF;
    s16 cropX1 = cmd_buff[6] & 0xFFFF;
    s16 cropY1 = cmd_buff[7] & 0xFFFF;
    u8 context = cmd_buff[8] & 0xFF;

    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, cam_select=%d, cropX0=%d, cropY0=%d, cropX1=%d, cropY1=%d",
        cam_select, cropX0, cropY0, cropX1, cropY1);
}

void SetSize(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 cam_select = cmd_buff[1] & 0xFF;
    u8 size       = cmd_buff[2] & 0xFF;
    u8 context    = cmd_buff[3] & 0xFF;

    cmd_buff[0] = IPC::MakeHeader(0x1F, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, cam_select=%d, size=%d, context=%d",
            cam_select, size, context);
}

void SetFrameRate(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 cam_select = cmd_buff[1] & 0xFF;
    frame_rate = static_cast<FrameRate>(cmd_buff[2] & 0xFF);

    cmd_buff[0] = IPC::MakeHeader(0x20, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, cam_select=%d, frame_rate=%d",
            cam_select, frame_rate);
}

void GetStereoCameraCalibrationData(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    // Default values taken from yuriks' 3DS. Valid data is required here or games using the
    // calibration get stuck in an infinite CPU loop.
    StereoCameraCalibrationData data = {};
    data.isValidRotationXY = 0;
    data.scale = 1.001776f;
    data.rotationZ = 0.008322907f;
    data.translationX = -87.70484f;
    data.translationY = -7.640977f;
    data.rotationX = 0.0f;
    data.rotationY = 0.0f;
    data.angleOfViewRight = 64.66875f;
    data.angleOfViewLeft = 64.76067f;
    data.distanceToChart = 250.0f;
    data.distanceCameras = 35.0f;
    data.imageWidth = 640;
    data.imageHeight = 480;

    cmd_buff[0] = IPC::MakeHeader(0x2B, 17, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;
    memcpy(&cmd_buff[2], &data, sizeof(data));

    LOG_TRACE(Service_CAM, "called");
}

void GetSuitableY2rStandardCoefficient(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[0] = IPC::MakeHeader(0x36, 2, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = 0;

    LOG_WARNING(Service_CAM, "(STUBBED) called");
}

void PlayShutterSound(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u8 sound_id = cmd_buff[1] & 0xFF;

    cmd_buff[0] = IPC::MakeHeader(0x38, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_CAM, "(STUBBED) called, sound_id=%d", sound_id);
}

void DriverInitialize(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    completion_event_cam1->Clear();
    completion_event_cam2->Clear();
    interrupt_buffer_error_event1->Clear();
    interrupt_buffer_error_event2->Clear();
    vsync_interrupt_event1->Clear();
    vsync_interrupt_event2->Clear();

    cmd_buff[0] = IPC::MakeHeader(0x39, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    driver_initialized = true;

    LOG_WARNING(Service_CAM, "(STUBBED) called");
}

void DriverFinalize(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[0] = IPC::MakeHeader(0x3A, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    driver_initialized = false;

    LOG_WARNING(Service_CAM, "(STUBBED) called");
}

void Init() {
    using namespace Kernel;

    AddService(new CAM_C_Interface);
    AddService(new CAM_Q_Interface);
    AddService(new CAM_S_Interface);
    AddService(new CAM_U_Interface);

    completion_event_cam1 = Kernel::Event::Create(ResetType::OneShot, "CAM_U::completion_event_cam1");
    completion_event_cam2 = Kernel::Event::Create(ResetType::OneShot, "CAM_U::completion_event_cam2");
    interrupt_buffer_error_event1 = Kernel::Event::Create(ResetType::OneShot, "CAM_U::interrupt_buffer_error_event1");
    interrupt_buffer_error_event2 = Kernel::Event::Create(ResetType::OneShot, "CAM_U::interrupt_buffer_error_event2");
    vsync_interrupt_event1 = Kernel::Event::Create(ResetType::OneShot, "CAM_U::vsync_interrupt_event1");
    vsync_interrupt_event2 = Kernel::Event::Create(ResetType::OneShot, "CAM_U::vsync_interrupt_event2");
}

void Shutdown() {
    completion_event_cam1 = nullptr;
    completion_event_cam2 = nullptr;
    interrupt_buffer_error_event1 = nullptr;
    interrupt_buffer_error_event2 = nullptr;
    vsync_interrupt_event1 = nullptr;
    vsync_interrupt_event2 = nullptr;
}

} // namespace CAM

} // namespace Service
