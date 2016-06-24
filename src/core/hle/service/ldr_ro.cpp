// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/common_types.h"
#include "common/logging/log.h"

#include "core/arm/arm_interface.h"
#include "core/hle/kernel/process.h"
#include "core/hle/kernel/vm_manager.h"
#include "core/hle/service/ldr_ro.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace LDR_RO

namespace LDR_RO {

static VAddr loaded_crs;

class CROHelper {
    const VAddr address;

    enum HeaderField {
        Magic = 0,
        NameOffset,
        NextCRO,
        PreviousCRO,
        FileSize,
        BssSize,
        FixedSize,
        Unk1,
        Unk2,
        Unk3,
        Unk4,
        SegmentOffset,
        CodeOffset,
        CodeSize,
        DataOffset,
        DataSize,
        ModuleNameOffset,
        ModuleNameSize,
        SegmentTableOffset,
        SegmentNum,
        SymbolExportTableOffset,
        SymbolExportNum,
        IndexExportTableOffset,
        IndexExportNum,
        ExportStringsOffset,
        ExportStringsSize,
        ExportTreeTableOffset,
        ExportTreeNum,
        ObjectTableOffset,
        ObjectNum,
        ExternalPatchTableOffset,
        ExternalPatchNum,
        SymbolImportTableOffset,
        SymbolImportNum,
        IndexImportTableOffset,
        IndexImportNum,
        OffsetImportTableOffset,
        OffsetImportNum,
        ImportStringsOffset,
        ImportStringsSize,
        OffsetExportTableOffset,
        OffsetExportNum,
        InternalPatchTableOffset,
        InternalPatchNum,
        StaticPatchTableOffset,
        StaticPatchNum,
        Fix0Barrier,

        Fix3Barrier = SymbolExportTableOffset,
        Fix2Barrier = ObjectTableOffset,
        Fix1Barrier = OffsetExportTableOffset,
    };
    static_assert(Fix0Barrier == (0x138 - 0x80) / 4, "CRO Header fields are wrong!");

    VAddr Field(HeaderField field) {
        return address + 0x80 + field * 4;
    }

    u32 GetField(HeaderField field) {
        return Memory::Read32(Field(field));
    }

    void SetField(HeaderField field, u32 value) {
        Memory::Write32(Field(field), value);
    }

    VAddr Next() {
        return GetField(NextCRO);
    }

    VAddr Previous() {
        return GetField(PreviousCRO);
    }

    void SetNext(VAddr next) {
        SetField(NextCRO, next);
    }

    void SetPrevious(VAddr next) {
        SetField(PreviousCRO, next);
    }

    ResultCode RebaseHeader(u32 cro_size) {
        u32 offset = GetField(NameOffset);
        if (offset)
            SetField(NameOffset, offset + address);

        for (int field = CodeOffset; field < Fix0Barrier; field += 2) {
            HeaderField header_field = static_cast<HeaderField>(field);
            offset = GetField(header_field);
            if (offset)
                SetField(header_field, offset + address);
        }

        // TODO Verify
        return RESULT_SUCCESS;
    }

    ResultCode RebaseSegmentTable(VAddr data_segment_addresss, u32 data_segment_size, VAddr bss_segment_address, u32 bss_segment_size) {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode RebaseSymbolExportTable() {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode RebaseObjectTable() {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode RebaseSymbolImportTable() {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode RebaseIndexImportTable() {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode RebaseOffsetImportTable() {
        // TODO
        return RESULT_SUCCESS;
    }

    void UnrebaseOffsetImportTable() {
        // TODO
    }

    void UnrebaseIndexImportTanle() {
        // TODO
    }

    void UnrebaseSymbolImportTable() {
        // TODO
    }

    void UnrebaseObjectTable() {
        // TODO
    }

    void UnrebaseSymbolExportTable() {
        // TODO
    }

    void UnrebaseSegmentTable() {
        // TODO
    }

    void UnrebaseHeader() {
        u32 offset = GetField(NameOffset);
        if (offset)
            SetField(NameOffset, offset - address);

        for (int field = CodeOffset; field < Fix0Barrier; field += 2) {
            HeaderField header_field = static_cast<HeaderField>(field);
            offset = GetField(header_field);
            if (offset)
                SetField(header_field, offset - address);
        }
    }

public:
    CROHelper(VAddr cro_address) : address(cro_address) {
    }

    ResultCode Rebase(u32 cro_size, VAddr data_segment_addresss, u32 data_segment_size, VAddr bss_segment_address, u32 bss_segment_size, bool is_crs = false) {
        ResultCode result = RebaseHeader(cro_size);
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing header %08X", result.raw);
            return result;
        }

        // TODO verify module name

        if (!is_crs) {
            result = RebaseSegmentTable(data_segment_addresss, data_segment_size, bss_segment_address, bss_segment_size);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error rebasing segment table %08X", result.raw);
                return result;
            }
        }

        result = RebaseSymbolExportTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing symbol export table %08X", result.raw);
            return result;
        }

        // TODO verify export tree

        // TODO verify export strings

        result = RebaseObjectTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing object table %08X", result.raw);
            return result;
        }

        // TODO verify external patch table
        // "SomethingAboutExternalPatchTable"

        result = RebaseSymbolImportTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing symbol import table %08X", result.raw);
            return result;
        }

        result = RebaseIndexImportTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing index import table %08X", result.raw);
            return result;
        }

        result = RebaseOffsetImportTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing offset import table %08X", result.raw);
            return result;
        }

        // TODO verify import strings

        // TODO verify offset export table

        // TODO verify internal patch table

        // TODO verify exit function

        return RESULT_SUCCESS;
    }

    void Unrebase(bool is_crs = false) {
        UnrebaseOffsetImportTable();
        UnrebaseIndexImportTanle();
        UnrebaseSymbolImportTable();
        UnrebaseObjectTable();
        UnrebaseSymbolExportTable();

        if (!is_crs)
            UnrebaseSegmentTable();

        SetNext(0);
        SetPrevious(0);

        // TODO clear fix size

        UnrebaseHeader();
    }

    ResultCode Verify(u32 cro_size, VAddr crr) {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode Link() {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode Unlink() {
        // TODO
        return RESULT_SUCCESS;
    }

    void RegisterCRS() {
        SetNext(0);
        SetPrevious(0);
    }

    void Register(bool auto_link) {
        CROHelper crs(loaded_crs);

        CROHelper head(auto_link ? crs.Next() : crs.Previous());
        if (head.address) {
            // there are already CROs registered
            // register as the new tail
            CROHelper tail(head.Previous());

            // link with the old tail
            ASSERT(tail.Next() == 0);
            SetPrevious(tail.address);
            tail.SetNext(address);

            // set previous of the head pointing to the new tail
            head.SetPrevious(address);
        } else {
            // register as the first CRO
            // set previous to self as tail
            SetPrevious(address);

            // set self as head
            if (auto_link)
                crs.SetNext(address);
            else
                crs.SetPrevious(address);
        }

        // the new one is the tail
        SetNext(0);
    }

    void Unregister() {
        CROHelper crs(loaded_crs);
        CROHelper nhead(crs.Next()), phead(crs.Previous());
        CROHelper next(Next()), previous(Previous());
        if (address == nhead.address || address == phead.address) {
            // removing head
            if (next.address) {
                // the next is new head
                // let its previous point to the tail
                next.SetPrevious(previous.address);
            }

            // set new head
            if (address == phead.address) {
                crs.SetPrevious(next.address);
            } else {
                crs.SetNext(next.address);
            }
        } else if (next.address) {
            // link previous and next
            previous.SetNext(next.address);
            next.SetPrevious(previous.address);
        } else {
            // removing tail
            // set previous as new tail
            previous.SetNext(0);

            // let head's previous point to the new tail
            if (nhead.address && nhead.Previous() == address) {
                nhead.SetPrevious(previous.address);
            } else if (phead.address && phead.Previous() == address) {
                phead.SetPrevious(previous.address);
            } else {
                UNREACHABLE();
            }
        }

        // unlink self
        SetNext(0);
        SetPrevious(0);
    }

    u32 Fix(int fix_level) {
        // TODO
        // Note: Write fixed size in cro
        return 0;
    }
};

/**
 * LDR_RO::Initialize service function
 *  Inputs:
 *      1 : CRS buffer pointer
 *      2 : CRS Size
 *      3 : Process memory address where the CRS will be mapped
 *      4 : Copy handle descriptor (zero)
 *      5 : KProcess handle
 *  Outputs:
 *      0 : Return header
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void Initialize(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    VAddr crs_buffer  = cmd_buff[1];
    u32 crs_size      = cmd_buff[2];
    VAddr crs_address = cmd_buff[3];

    LOG_WARNING(Service_LDR, "(STUBBED) called. crs_adress=0x%08X, crs_size=0x%08X, address=0x%08X",
                crs_buffer, crs_size, crs_address);

    cmd_buff[0] = IPC::MakeHeader(1, 1, 0);

    if (loaded_crs) {
        cmd_buff[1] = 0xD9612FF9;
        return;
    }

    // TODO Check process, size, size alignment, address alignment, memory access

    ResultCode result(RESULT_SUCCESS.raw);

    // TODO should be memory aliasing?
    std::shared_ptr<std::vector<u8>> crs_mem = std::make_shared<std::vector<u8>>(crs_size);
    Memory::ReadBlock(crs_buffer, crs_mem->data(), crs_size);
    result = Kernel::g_current_process->vm_manager.MapMemoryBlock(crs_address, crs_mem, 0, crs_size, Kernel::MemoryState::Code).Code();
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error mapping memory block %08X", result.raw);
        cmd_buff[1] = result.raw;
        return;
    }

    CROHelper crs(crs_address);
    crs.RegisterCRS();

    result = crs.Rebase(crs_size, 0, 0, 0, 0, true);
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error Loading CRS %08X", result.raw);
        cmd_buff[1] = result.raw;
        return;
    }

    loaded_crs = crs_address;

    cmd_buff[1] = RESULT_SUCCESS.raw;
}

/**
 * LDR_RO::LoadCRR service function
 *  Inputs:
 *      1 : CRS buffer pointer
 *      2 : CRS Size
 *      3 : Copy handle descriptor (zero)
 *      4 : KProcess handle
 *  Outputs:
 *      0 : Return header
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void LoadCRR(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u32 crs_buffer_ptr = cmd_buff[1];
    u32 crs_size       = cmd_buff[2];
    u32 value          = cmd_buff[3];
    u32 process        = cmd_buff[4];

    if (value != 0) {
        LOG_ERROR(Service_LDR, "This value should be zero, but is actually %u!", value);
    }

    // TODO(purpasmart96): Verify return header on HW

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error

    LOG_WARNING(Service_LDR, "(STUBBED) called. crs_buffer_ptr=0x%08X, crs_size=0x%08X, value=0x%08X, process=0x%08X",
                crs_buffer_ptr, crs_size, value, process);
}

/**
 * LDR_RO::LoadCRO service function
 *  Inputs:
 *      WTF
 */
static void LoadCRO(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    VAddr cro_buffer  = cmd_buff[1];
    VAddr cro_address = cmd_buff[2];
    u32 cro_size      = cmd_buff[3];
    VAddr data_segment_address = cmd_buff[4];
    u32 zero = cmd_buff[5];
    u32 data_segment_size = cmd_buff[6];
    u32 bss_segment_address = cmd_buff[7];
    u32 bss_segment_size = cmd_buff[8];
    bool auto_link = (cmd_buff[9] & 0xFF) != 0;
    u32 fix_level = cmd_buff[10];
    VAddr crr_address = cmd_buff[11];

    cmd_buff[0] = IPC::MakeHeader(1, 2, 0);

    if (!loaded_crs) {
        cmd_buff[1] = 0xD9612FF8;
        return;
    }

    // TODO Check process, size, size alignment, address alignment, memory access

    if (zero) {
        LOG_ERROR(Service_LDR, "Zero is not zero %d", zero);
        cmd_buff[1] = 0xE1612C1D;
        return;
    }

    // TODO should be memory aliasing?
    std::shared_ptr<std::vector<u8>> cro_mem = std::make_shared<std::vector<u8>>(cro_size);
    Memory::ReadBlock(cro_buffer, cro_mem->data(), cro_size);
    ResultCode result = Kernel::g_current_process->vm_manager.MapMemoryBlock(cro_address, cro_mem, 0, cro_size, Kernel::MemoryState::Code).Code();
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error mapping memory block %08X", result.raw);
        cmd_buff[1] = result.raw;
        return;
    }

    CROHelper cro(cro_address);

    result = cro.Verify(cro_size, crr_address);
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error verifying CRO in CRR %08X", result.raw);
        // TODO Unmap memory?
        cmd_buff[1] = result.raw;
        return;
    }

    result = cro.Rebase(cro_size, data_segment_address, data_segment_size, bss_segment_address, bss_segment_size);
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error rebasing CRO %08X", result.raw);
        // TODO Unmap memory?
        cmd_buff[1] = result.raw;
        return;
    }

    result = cro.Link();
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error linking CRO %08X", result.raw);
        // TODO Unmap memory?
        cmd_buff[1] = result.raw;
        return;
    }

    cro.Register(auto_link);

    u32 fix_size = cro.Fix(fix_level);

    // TODO remap memory?
    // what ro module do: commit fixed-out memory, protect segment 0, commit fixed cro?

    cmd_buff[2] = fix_size;

    cmd_buff[1] = RESULT_SUCCESS.raw;
}

/**
 * LDR_RO::UnloadCRO service function
 *  Inputs:
 *      1 : mapped CRO pointer
 *      2 : CRO size
 *      3 : Original CRO pointer
 *      4 : Copy handle descriptor (zero)
 *      5 : KProcess handle
 *  Outputs:
 *      0 : Return header
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void UnloadCRO(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u32 cro_address = cmd_buff[1];
    u32 cro_size = cmd_buff[2];

    // TODO "Validate Something"
    // TODO loc_140033CC

    CROHelper cro(cro_address);

    cro.Unregister();

    ResultCode result = cro.Unlink();
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error unlinking CRO %08X", result.raw);
        // TODO Unmap memory?
        cmd_buff[1] = result.raw;
        return;
    }

    // TODO if fixed {...}

    cro.Unrebase();

    Kernel::g_current_process->vm_manager.UnmapRange(cro_address, cro_size);

    cmd_buff[0] = IPC::MakeHeader(1, 1, 0);
}

/**
 * LDR_RO::LinkCRO service function
 *  Inputs:
 *      1 : mapped CRO pointer
 *      2 : Copy handle descriptor (zero)
 *      3 : KProcess handle
 *  Outputs:
 *      0 : Return header
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void LinkCRO(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u32 cro = cmd_buff[1];

    cmd_buff[0] = IPC::MakeHeader(1, 1, 0);
    cmd_buff[1] = CROHelper(cro).Link().raw;
}

/**
 * LDR_RO::UnlinkCRO service function
 *  Inputs:
 *      1 : mapped CRO pointer
 *      2 : Copy handle descriptor (zero)
 *      3 : KProcess handle
 *  Outputs:
 *      0 : Return header
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void UnlinkCRO(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u32 cro = cmd_buff[1];

    // TODO "Validate Something"

    cmd_buff[0] = IPC::MakeHeader(1, 1, 0);
    cmd_buff[1] = CROHelper(cro).Unlink().raw;
}

const Interface::FunctionInfo FunctionTable[] = {
    {0x000100C2, Initialize,            "Initialize"},
    {0x00020082, LoadCRR,               "LoadCRR"},
    {0x00030042, nullptr,               "UnloadCCR"},
    {0x000402C2, LoadCRO,               "LoadCRO"},
    {0x000500C2, UnloadCRO,             "UnloadCRO"},
    {0x00060042, LinkCRO,               "LinkCRO"},
    {0x00070042, UnlinkCRO,             "UnlinkCRO"},
    {0x00080042, nullptr,               "Shutdown"},
    {0x000902C2, nullptr,               "LoadCRO_New"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface class

Interface::Interface() {
    Register(FunctionTable);

    loaded_crs = 0;
}

} // namespace
