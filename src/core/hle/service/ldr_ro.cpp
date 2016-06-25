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

    struct SegmentEntry {
        u32 offset;
        u32 size;
        u32 id;
    };

    struct SymbolExportEntry {
        u32 name_offset; // pointing to a substring in ExportStrings
        u32 segment_tag;
    };

    struct IndexExportEntry {
        u32 segment_tag;
    };

    struct ExportTreeEntry {
        u16 segment_13_3;
        u16 next;
        u16 next_level;
        u16 export_table_id;
    };

    struct ImportModuleEntry {
        u32 name_offset; // pointing to a substring in ImporStrings
        u32 index_import_table_offset; // pointing to a subtable of IndexImportTable
        u32 index_import_num;
        u32 offset_import_table_offset; // pointing to a subtable of OffsetImportTable
        u32 offset_import_num;
    };

    struct PatchEntry { // for ExternalPatchTable and StaticPatchTable
        u32 segment_tag;
        u8 type;
        u8 is_batch_end;
        u8 batch_resolved; // set at batch begin
        u8 unk3;
        u32 x;
    };

    struct InternalPatchEntry {
        u32 segment_tag;
        u8 type;
        u8 value_segment_index;
        u8 unk2;
        u8 unk3;
        u32 x;
    };

    struct SymbolImportEntry {
        u32 name_offset; // pointing to a substring in ImporStrings
        u32 patch_batch_offset; // pointing to a batch in ExternalPatchTable
    };

    struct IndexImportEntry {
        u32 index; // index in opponent's IndexExportTable
        u32 patch_batch_offset; // pointing to a batch in ExternalPatchTable
    };

    struct OffsetImportEntry {
        u32 segment_tag;
        u32 patch_batch_offset; // pointing to a batch in ExternalPatchTable
    };

    struct OffsetExportEntry {
        u32 segment_tag;
        u32 patch_batch_offset; // pointing to a batch in StaticPatchTable
    };

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
        OnLoad_segment_tag,
        OnExit_segment_tag,
        OnUnresolved_segment_tag,

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

        ImportModuleTableOffset,
        ImportModuleNum,
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
        Fix2Barrier = ImportModuleTableOffset,
        Fix1Barrier = OffsetExportTableOffset,
    };
    static_assert(Fix0Barrier == (0x138 - 0x80) / 4, "CRO Header fields are wrong!");

    static const std::array<int, 17> ENTRY_SIZE;
    static const std::array<HeaderField, 4> FIX_BARRIERS;

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

    template <typename T> // [](CROHelper cro)->ResultCode
    ResultCode ForEachAutoLinkCRO(T func) {
        VAddr current = loaded_crs;
        while (current) {
            CROHelper cro(current);
            ResultCode result = func(cro);
            if (result.IsError())
                return result;
            current = cro.Next();
        }
        return RESULT_SUCCESS;
    }

    template <HeaderField field, typename T>
    void GetEntry(int index, T& data) {
        Memory::ReadBlock(GetField(field) + index * sizeof(T), &data, sizeof(T));
    }

    template <HeaderField field, typename T>
    void SetEntry(int index, const T& data) {
        Memory::WriteBlock(GetField(field) + index * sizeof(T), &data, sizeof(T));
    }

    static std::tuple<u32, u32> DecodeSegmentTag(u32 segment_tag) {
        return std::make_tuple(segment_tag & 0xF, segment_tag >> 4);
    }

    VAddr SegmentTagToAddress(u32 segment_tag) {
        u32 index, offset;
        std::tie(index, offset) = DecodeSegmentTag(segment_tag);
        u32 segment_num = GetField(SegmentNum);
        if (index >= segment_num)
            return 0;
        SegmentEntry entry;
        GetEntry<SegmentTableOffset>(index, entry);
        if (offset >= entry.size)
            return 0;
        return entry.offset + offset;
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

    ResultCode RebaseSegmentTable(VAddr data_segment_address, u32 data_segment_size,
        VAddr bss_segment_address, u32 bss_segment_size, u32& prev_data_segment) {
        prev_data_segment = 0;
        u32 segment_num = GetField(SegmentNum);
        for (u32 i = 0; i < segment_num; ++i) {
            SegmentEntry segment;
            GetEntry<SegmentTableOffset>(i, segment);
            // TODO verify address
            if (segment.id == 2) {
                if (segment.size) {
                    prev_data_segment = segment.offset;
                    segment.offset = data_segment_address;
                }
            } else if (segment.id == 3) {
                if (segment.size) {
                    segment.offset = bss_segment_address;
                }
            } else if (segment.offset) {
                segment.offset += address;
            }
            SetEntry<SegmentTableOffset>(i, segment);
        }
        return RESULT_SUCCESS;
    }

    ResultCode RebaseSymbolExportTable() {
        u32 symbol_export_num = GetField(SymbolExportNum);
        for (u32 i = 0; i < symbol_export_num; ++i) {
            SymbolExportEntry entry;
            GetEntry<SymbolExportTableOffset>(i, entry);
            // TODO verify address, should be in export strings
            if (entry.name_offset) {
                entry.name_offset += address;
            }
            SetEntry<SymbolExportTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    ResultCode RebaseImportModuleTable() {
        u32 object_num = GetField(ImportModuleNum);
        for (u32 i = 0; i < object_num; ++i) {
            ImportModuleEntry entry;
            GetEntry<ImportModuleTableOffset>(i, entry);
            // TODO verify address
            if (entry.name_offset)
                entry.name_offset += address;
            if (entry.index_import_table_offset)
                entry.index_import_table_offset += address;
            if (entry.offset_import_table_offset)
                entry.offset_import_table_offset += address;
            SetEntry<ImportModuleTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    ResultCode RebaseSymbolImportTable() {
        u32 num = GetField(SymbolImportNum);
        for (u32 i = 0; i < num ; ++i) {
            // TODO verify address, patch_batch_offset should be in external patch table
            SymbolImportEntry entry;
            GetEntry<SymbolImportTableOffset>(i, entry);
            if (entry.name_offset)
                entry.name_offset += address;
            if (entry.patch_batch_offset)
                entry.patch_batch_offset += address;
            SetEntry<SymbolImportTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    ResultCode RebaseIndexImportTable() {
        u32 num = GetField(IndexImportNum);
        for (u32 i = 0; i < num ; ++i) {
            // TODO verify address, patch_batch_offset should be in external patch table
            IndexImportEntry entry;
            GetEntry<IndexImportTableOffset>(i, entry);
            if (entry.patch_batch_offset)
                entry.patch_batch_offset += address;
            SetEntry<IndexImportTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    ResultCode RebaseOffsetImportTable() {
        u32 num = GetField(OffsetImportNum);
        for (u32 i = 0; i < num ; ++i) {
            // TODO verify address, patch_batch_offset should be in external patch table
            OffsetImportEntry entry;
            GetEntry<OffsetImportTableOffset>(i, entry);
            if (entry.patch_batch_offset)
                entry.patch_batch_offset += address;
            SetEntry<OffsetImportTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    ResultCode ApplyPatch(VAddr target_address, u8 patch_type, u32 x, u32 value, u32 addressB) {
        // TODO
        return RESULT_SUCCESS;
    }

    ResultCode ResetAllExternalPatches() {
        u32 reset_value = SegmentTagToAddress(GetField(OnUnresolved_segment_tag));

        bool batch_begin = true;
        u32 external_patch_num = GetField(ExternalPatchNum);
        for (u32 i = 0; i < external_patch_num; ++i) {
            PatchEntry patch;
            GetEntry<ExternalPatchTableOffset>(i, patch);
            VAddr patch_target = SegmentTagToAddress(patch.segment_tag);
            if (patch_target == 0) {
                return ResultCode(0xD9012C12);
            }
            ResultCode result = ApplyPatch(patch_target, patch.type, patch.x, reset_value, patch_target);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error applying patch %08X", result.raw);
                return result;
            }

            if (batch_begin) {
                patch.batch_resolved = 0; // reset to unresolved state
                SetEntry<ExternalPatchTableOffset>(i, patch);
            }

            batch_begin = patch.is_batch_end != 0; // current is end, next is begin
        }

        return RESULT_SUCCESS;
    }

    ResultCode ApplyPatchBatch(VAddr batch, u32 patch_value) {
        if (patch_value==0)
            return ResultCode(0xD9012C10);

        VAddr patch_address = batch;
        while (true) {
            PatchEntry patch;
            Memory::ReadBlock(patch_address, &patch, sizeof(PatchEntry));

            VAddr patch_target = SegmentTagToAddress(patch.segment_tag);
            if (patch_target == 0) {
                return ResultCode(0xD9012C12);
            }
            ResultCode result = ApplyPatch(patch_target, patch.type, patch.x, patch_value, patch_target);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error applying patch %08X", result.raw);
                return result;
            }

            if (patch.is_batch_end)
                break;

            patch_address += sizeof(PatchEntry);
        }

        PatchEntry patch;
        Memory::ReadBlock(batch, &patch, sizeof(PatchEntry));
        patch.batch_resolved = 1;
        Memory::WriteBlock(batch, &patch, sizeof(PatchEntry));
    }

    ResultCode ApplyOffsetExportToCRS() {
        CROHelper crs(loaded_crs);
        u32 offset_export_num = GetField(OffsetExportNum);
        for (u32 i = 0; i < offset_export_num; ++i) {
            OffsetExportEntry entry;
            GetEntry<OffsetExportTableOffset>(i, entry);
            // TODO verify address, batch_offset should be in static patch table
            u32 batch_address = entry.patch_batch_offset + address;
            u32 patch_value = SegmentTagToAddress(entry.segment_tag);

            ResultCode result = crs.ApplyPatchBatch(batch_address, patch_value);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                return result;
            }
        }
        return RESULT_SUCCESS;
    }

    ResultCode ApplyInternalPatches(u32 old_data_segment_address) {
        u32 internal_patch_num = GetField(InternalPatchNum);
        for (u32 i = 0; i < internal_patch_num; ++i) {
            InternalPatchEntry patch;
            GetEntry<InternalPatchTableOffset>(i, patch);
            u32 target_segment_index, target_segment_offset;
            std::tie(target_segment_index, target_segment_offset) = DecodeSegmentTag(patch.segment_tag);
            SegmentEntry target_segment;
            // TODO check segment index and offset
            GetEntry<SegmentTableOffset>(target_segment_index, target_segment);
            u32 target_address, target_addressB = target_segment.offset + target_segment_offset;
            if (target_segment.id == 2) {
                target_address = old_data_segment_address + target_segment_offset;
            } else {
                target_address = target_addressB;
            }

            SegmentEntry value_segment;
            // TODO check segment index
            GetEntry<SegmentTableOffset>(patch.value_segment_index, value_segment);

            ResultCode result = ApplyPatch(target_address, patch.type, patch.x, value_segment.offset, target_addressB);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error applying patch %08X", result.raw);
                return result;
            }
        }
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

        u32 prev_data_segment_address = 0;
        if (!is_crs) {
            result = RebaseSegmentTable(
                data_segment_addresss, data_segment_size,
                bss_segment_address, bss_segment_size,
                prev_data_segment_address);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error rebasing segment table %08X", result.raw);
                return result;
            }
        }
        prev_data_segment_address += address;

        result = RebaseSymbolExportTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing symbol export table %08X", result.raw);
            return result;
        }

        // TODO verify export tree

        // TODO verify export strings

        result = RebaseImportModuleTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing object table %08X", result.raw);
            return result;
        }

        // TODO verify external patch

        result = ResetAllExternalPatches();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error resetting all external patches %08X", result.raw);
            return result;
        }

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

        if (!is_crs) {
            result = ApplyOffsetExportToCRS();
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error applying offset export to CRS %08X", result.raw);
                return result;
            }
        }

        result = ApplyInternalPatches(prev_data_segment_address);
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error applying internal patches %08X", result.raw);
            return result;
        }

        // TODO apply exit function patch

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

        SetField(FixedSize, 0);

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

    u32 GetFixEnd(int fix_level) {
        u32 end = 0x138;
        end = std::max<u32>(end, GetField(CodeOffset) + GetField(CodeSize));

        u32 entry_size_i = 2;
        int field = ModuleNameOffset;
        while (true) {
            end = std::max<u32>(end,
                GetField(static_cast<HeaderField>(field)) +
                GetField(static_cast<HeaderField>(field + 1)) * ENTRY_SIZE[entry_size_i]);

            ++entry_size_i;
            field += 2;

            if (field == FIX_BARRIERS[fix_level])
                return end;
        }
    }

    u32 Fix(int fix_level) {
        u32 fix_end = GetFixEnd(fix_level);

        if (fix_level) {
            SetField(Magic, 0x44584946); // FIXD

            for (int field = FIX_BARRIERS[fix_level]; field < Fix0Barrier; field += 2) {
                SetField(static_cast<HeaderField>(field), fix_end);
                SetField(static_cast<HeaderField>(field + 1), 0);
            }
        }

        fix_end += 0xFFF;
        fix_end &= 0xFFFFF000; //round up

        u32 fixed_size = fix_end - address;
        SetField(FixedSize, fixed_size);
        return fixed_size;
    }
};

const std::array<int, 17> CROHelper::ENTRY_SIZE {{
    1, // code
    1, // data
    1, // module name
    sizeof(SegmentEntry),
    sizeof(SymbolExportEntry),
    sizeof(IndexExportEntry),
    1, // export strings
    sizeof(ExportTreeEntry),
    sizeof(ImportModuleEntry),
    sizeof(PatchEntry),
    sizeof(SymbolImportEntry),
    sizeof(IndexImportEntry),
    sizeof(OffsetImportEntry),
    1, // import strings
    sizeof(OffsetExportEntry),
    sizeof(InternalPatchEntry),
    sizeof(PatchEntry)
}};

const std::array<CROHelper::HeaderField, 4> CROHelper::FIX_BARRIERS {{
    Fix0Barrier,
    Fix1Barrier,
    Fix2Barrier,
    Fix3Barrier
}};

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

    Core::g_app_core->ClearInstructionCache();

    cmd_buff[1] = RESULT_SUCCESS.raw;
    cmd_buff[2] = fix_size;
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

    Core::g_app_core->ClearInstructionCache();

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
