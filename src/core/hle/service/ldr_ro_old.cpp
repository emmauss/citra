// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/common_types.h"
#include "common/file_util.h"
#include "common/logging/log.h"

#include "core/core.h"
#include "core/arm/arm_interface.h"
#include "core/hle/hle.h"
#include "core/hle/kernel/process.h"
#include "core/hle/kernel/vm_manager.h"
#include "core/hle/service/ldr_ro.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace LDR_RO

namespace LDR_RO {

struct SegmentTableEntry {
    u32 segment_offset;
    u32 segment_size;
    u32 segment_id;
};

struct ExternalRelocationTableEntry {
    u32 offset;
    u8 type;
    u8 unk;
    u8 unk2;
    u8 unk3;
    u32 x;

    u8 GetTargetSegment() { return offset & 0xF; }
    u32 GetSegmentOffset() { return offset >> 4; }
};

struct OffsetExportTableEntry {
    u32 segment_offset;
    u32 patches_offset;

    u8 GetTargetSegment() { return segment_offset & 0xF; }
    u32 GetSegmentOffset() { return segment_offset >> 4; }
};

struct Unk2TableEntry {
    u32 offset_or_index; ///< Index in the CRO's segment offset table (unk1) for table1 entries, or segment_offset for table2 entries
    u32 patches_offset;
};

struct ObjectInfo {
    u32 string_offset;
    u32 table1_offset;
    u32 table1_num;
    u32 table2_offset;
    u32 table2_num;

    Unk2TableEntry* GetTable1Entry(u32 index);
    Unk2TableEntry* GetTable2Entry(u32 index);
};

Unk2TableEntry* ObjectInfo::GetTable1Entry(u32 index) {
    return reinterpret_cast<Unk2TableEntry*>(Memory::GetPointer(table1_offset) + sizeof(Unk2TableEntry) * index);
}

Unk2TableEntry* ObjectInfo::GetTable2Entry(u32 index) {
    return reinterpret_cast<Unk2TableEntry*>(Memory::GetPointer(table2_offset) + sizeof(Unk2TableEntry) * index);
}

struct SymbolExportTableEntry {
    u32 name_offset;
    u32 segment_offset;

    u8 GetTargetSegment() { return segment_offset & 0xF; }
    u32 GetSegmentOffset() { return segment_offset >> 4; }
};

struct IndexExportTableEntry {
    u32 segment_offset;
};

struct ImportTableEntry {
    u32 name_offset;
    u32 symbol_offset;
};

struct ExportTreeEntry {
    u16 segment_offset;
    u16 next;
    u16 next_level;
    u16 export_table_id;

    u8 GetTargetSegment() { return segment_offset & 0x7; }
    u32 GetSegmentOffset() { return segment_offset >> 3; }
};

struct ExportedSymbol {
    std::string name;
    u32 cro_base;
    u32 cro_offset;
};

struct CROHeader {
    u8 sha2_hash[0x80];
    char magic[4];
    u32 name_offset;
    u32 next_cro;
    u32 previous_cro;
    u32 file_size;
    u32 bss_size;
    u32 fixed_size;
    INSERT_PADDING_WORDS(0x4);
    u32 segment_offset;
    u32 code_offset;
    u32 code_size;
    u32 data_offset;
    u32 data_size;
    u32 module_name_offset;
    u32 module_name_size;
    u32 segment_table_offset;
    u32 segment_table_num;
    u32 symbo_export_table_offset;
    u32 symbo_export_table_num;
    u32 index_export_table_offset;
    u32 index_export_table_num;
    u32 export_strings_offset;
    u32 export_strings_num;
    u32 export_tree_offset;
    u32 export_tree_num;
    u32 object_info_offset;
    u32 object_info_num;
    u32 external_relocation_table_offset;
    u32 external_relocation_table_num;
    u32 symbo_import_table_offset;
    u32 symbo_import_table_num;
    u32 index_import_table_offset;
    u32 index_import_table_num;
    u32 offset_import_table_offset;
    u32 offset_import_table_num;
    u32 import_strings_offset;
    u32 import_strings_num;
    u32 offset_export_table_offset;
    u32 offset_export_table_num;
    u32 internal_relocation_table_offset;
    u32 internal_relocation_table_num;
    u32 static_relocation_table_offset;
    u32 static_relocation_table_num;

    u8 GetImportPatchesTargetSegment() { return segment_offset & 0xF; }
    u32 GetImportPatchesSegmentOffset() { return segment_offset >> 4; }

    SegmentTableEntry GetSegmentTableEntry(u32 index) const;
    void SetSegmentTableEntry(u32 index, const SegmentTableEntry& entry);
    ResultCode RelocateSegmentsTable(u32 base, u32 size, u32 data_section, u32 bss_section, u32& prev_data_section);

    SymbolExportTableEntry* GetSymbolExportTableEntry(u32 index);
    ResultCode RelocateExportsTable(u32 base);

    ExportTreeEntry* GetExportTreeEntry(u32 index);

    ExternalRelocationTableEntry* GetExternalRelocationTableEntry(u32 index);

    ImportTableEntry* GetSymbolImportTableEntry(u32 index);
    void RelocateSymbolImportTable(u32 base);

    ImportTableEntry* GetIndexImportTableEntry(u32 index);
    void RelocateIndexImportTable(u32 base);

    ImportTableEntry* GetOffsetImportTableEntry(u32 index);
    void RelocateOffsetImportTable(u32 base);

    OffsetExportTableEntry* GetOffsetExportTableEntry(u32 index);

    ExternalRelocationTableEntry* GetRelocationPatchEntry(u32 index);

    ObjectInfo* GetObjectInfoEntry(u32 index);

    u32 GetIndexExportTableEntry(u32 index);

    void RelocateObjectInfos(u32 base);

    bool VerifyAndRelocateOffsets(u32 base, u32 size);
};

static_assert(sizeof(CROHeader) == 0x138, "CROHeader has wrong size");

static std::unordered_map<std::string, ExportedSymbol> loaded_exports;
static u32 the_crs_base;

SegmentTableEntry CROHeader::GetSegmentTableEntry(u32 index) const {
    SegmentTableEntry entry;
    memcpy(&entry, Memory::GetPointer(segment_table_offset + sizeof(SegmentTableEntry) * index), sizeof(SegmentTableEntry));
    return entry;
}

void CROHeader::SetSegmentTableEntry(u32 index, const SegmentTableEntry& entry) {
    memcpy(Memory::GetPointer(segment_table_offset + sizeof(SegmentTableEntry) * index), &entry, sizeof(SegmentTableEntry));
}

ResultCode CROHeader::RelocateSegmentsTable(u32 base, u32 size, u32 data_section, u32 bss_section, u32& prev_data_section) {
    u32 cro_end = base + size;

    prev_data_section = 0;
    for (int i = 0; i < segment_table_num; ++i) {
        SegmentTableEntry entry = GetSegmentTableEntry(i);
        if (entry.segment_id == 2) {
            prev_data_section = entry.segment_offset;
            entry.segment_offset = data_section;
        } else if (entry.segment_id == 3) {
            entry.segment_offset = bss_section;
        } else if (entry.segment_offset) {
            entry.segment_offset += base;
            if (entry.segment_offset > cro_end)
                return ResultCode(0xD9012C19);
        }
        SetSegmentTableEntry(i, entry);
    }

    return RESULT_SUCCESS;
}

ExportTreeEntry* CROHeader::GetExportTreeEntry(u32 index) {
    return reinterpret_cast<ExportTreeEntry*>(Memory::GetPointer(export_tree_offset + sizeof(ExportTreeEntry) * index));
}

u32 CROHeader::GetIndexExportTableEntry(u32 index) {
    return *reinterpret_cast<u32*>(Memory::GetPointer(index_export_table_offset + sizeof(u32) * index));
}

SymbolExportTableEntry* CROHeader::GetSymbolExportTableEntry(u32 index) {
    return reinterpret_cast<SymbolExportTableEntry*>(Memory::GetPointer(symbo_export_table_offset + sizeof(SymbolExportTableEntry) * index));
}

ResultCode CROHeader::RelocateExportsTable(u32 base) {
    for (int i = 0; i < symbo_export_table_num; ++i) {
        SymbolExportTableEntry* entry = GetSymbolExportTableEntry(i);
        if (entry->name_offset)
            entry->name_offset += base;

        if (entry->name_offset < export_strings_offset ||
            entry->name_offset > export_strings_offset + export_strings_num)
            return ResultCode(0xD9012C11);
    }

    return RESULT_SUCCESS;
}

ExternalRelocationTableEntry* CROHeader::GetExternalRelocationTableEntry(u32 index) {
    return reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(external_relocation_table_offset + sizeof(ExternalRelocationTableEntry) * index));
}

ImportTableEntry* CROHeader::GetSymbolImportTableEntry(u32 index) {
    return reinterpret_cast<ImportTableEntry*>(Memory::GetPointer(symbo_import_table_offset + sizeof(ImportTableEntry) * index));
}

ImportTableEntry* CROHeader::GetIndexImportTableEntry(u32 index) {
    return reinterpret_cast<ImportTableEntry*>(Memory::GetPointer(index_import_table_offset + sizeof(ImportTableEntry) * index));
}

ImportTableEntry* CROHeader::GetOffsetImportTableEntry(u32 index) {
    return reinterpret_cast<ImportTableEntry*>(Memory::GetPointer(offset_import_table_offset + sizeof(ImportTableEntry) * index));
}

void CROHeader::RelocateSymbolImportTable(u32 base) {
    for (int i = 0; i < symbo_import_table_num; ++i) {
        ImportTableEntry* entry = GetSymbolImportTableEntry(i);
        if (entry->name_offset)
            entry->name_offset += base;
        if (entry->symbol_offset)
            entry->symbol_offset += base;
    }
}

void CROHeader::RelocateIndexImportTable(u32 base) {
    for (int i = 0; i < index_import_table_num; ++i) {
        ImportTableEntry* entry = GetIndexImportTableEntry(i);
        if (entry->symbol_offset)
            entry->symbol_offset += base;
    }
}

void CROHeader::RelocateOffsetImportTable(u32 base) {
    for (int i = 0; i < offset_import_table_num; ++i) {
        ImportTableEntry* entry = GetOffsetImportTableEntry(i);
        if (entry->symbol_offset)
            entry->symbol_offset += base;
    }
}

void CROHeader::RelocateObjectInfos(u32 base) {
    for (int i = 0; i < object_info_num; ++i) {
        ObjectInfo* entry = GetObjectInfoEntry(i);
        if (entry->string_offset)
            entry->string_offset += base;
        if (entry->table1_offset)
            entry->table1_offset += base;
        if (entry->table2_offset)
            entry->table2_offset += base;
    }
}

OffsetExportTableEntry* CROHeader::GetOffsetExportTableEntry(u32 index) {
    return reinterpret_cast<OffsetExportTableEntry*>(Memory::GetPointer(offset_export_table_offset * sizeof(OffsetExportTableEntry) * index));
}

ExternalRelocationTableEntry* CROHeader::GetRelocationPatchEntry(u32 index) {
    return reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(internal_relocation_table_offset + sizeof(ExternalRelocationTableEntry) * index));
}

ObjectInfo* CROHeader::GetObjectInfoEntry(u32 index) {
    return reinterpret_cast<ObjectInfo*>(Memory::GetPointer(object_info_offset + sizeof(ObjectInfo) * index));
}

bool CROHeader::VerifyAndRelocateOffsets(u32 base, u32 size) {
    u32 end = base + size;

    // Error if the magic is invalid
    if (memcmp(magic, "CRO0", 4) != 0)
        return false;

    // If these values are set the game might be trying to load the same CRO multiple times
    if (next_cro || previous_cro)
        return false;

    // This seems to be a hard limit set by the RO module
    if (file_size >= 0x10000000 || bss_size >= 0x10000000)
        return false;

    if (fixed_size != 0)
        return false;

    if (code_offset < sizeof(CROHeader))
        return false;

    if (module_name_offset < code_offset)
        return false;

    if (module_name_offset > segment_table_offset)
        return false;

    if (symbo_export_table_offset < segment_table_offset)
        return false;

    if (symbo_export_table_offset > export_tree_offset)
        return false;

    if (index_export_table_offset < export_tree_offset)
        return false;

    if (index_export_table_offset > export_strings_offset)
        return false;

    if (object_info_offset < export_strings_offset)
        return false;

    if (object_info_offset > external_relocation_table_offset)
        return false;

    if (symbo_import_table_offset < external_relocation_table_offset)
        return false;

    if (symbo_import_table_offset > index_import_table_offset)
        return false;

    if (offset_import_table_offset < index_import_table_offset)
        return false;

    if (offset_import_table_offset > import_strings_offset)
        return false;

    if (offset_export_table_offset < import_strings_offset)
        return false;

    if (offset_export_table_offset > internal_relocation_table_offset)
        return false;

    if (static_relocation_table_offset < internal_relocation_table_offset)
        return false;

    if (static_relocation_table_offset > data_offset)
        return false;

    if (data_offset > file_size)
        return false;

    if (name_offset) {
        name_offset += base;
        if (name_offset > end)
            return false;
    }

    if (code_offset) {
        code_offset += base;
        if (code_offset > end)
            return false;
    }

    if (data_offset) {
        data_offset += base;
        if (data_offset > end)
            return false;
    }

    if (module_name_offset) {
        module_name_offset += base;
        if (module_name_offset > end)
            return false;
    }

    if (segment_table_offset) {
        segment_table_offset += base;
        if (segment_table_offset > end)
            return false;
    }

    if (symbo_export_table_offset) {
        symbo_export_table_offset += base;
        if (symbo_export_table_offset > end)
            return false;
    }

    if (index_export_table_offset) {
        index_export_table_offset += base;
        if (index_export_table_offset > end)
            return false;
    }

    if (export_strings_offset) {
        export_strings_offset += base;
        if (export_strings_offset > end)
            return false;
    }

    if (export_tree_offset) {
        export_tree_offset += base;
        if (export_tree_offset > end)
            return false;
    }

    if (object_info_offset) {
        object_info_offset += base;
        if (object_info_offset > end)
            return false;
    }

    if (external_relocation_table_offset) {
        external_relocation_table_offset += base;
        if (external_relocation_table_offset > end)
            return false;
    }

    if (symbo_import_table_offset) {
        symbo_import_table_offset += base;
        if (symbo_import_table_offset > end)
            return false;
    }

    if (index_import_table_offset) {
        index_import_table_offset += base;
        if (index_import_table_offset > end)
            return false;
    }

    if (offset_import_table_offset) {
        offset_import_table_offset += base;
        if (offset_import_table_offset > end)
            return false;
    }

    if (import_strings_offset) {
        import_strings_offset += base;
        if (import_strings_offset > end)
            return false;
    }

    if (offset_export_table_offset) {
        offset_export_table_offset += base;
        if (offset_export_table_offset > end)
            return false;
    }

    if (internal_relocation_table_offset) {
        internal_relocation_table_offset += base;
        if (internal_relocation_table_offset > end)
            return false;
    }

    if (static_relocation_table_offset) {
        static_relocation_table_offset += base;
        if (static_relocation_table_offset > end)
            return false;
    }

    if (code_offset + code_size > end ||
        data_offset + data_size > end ||
        module_name_offset + module_name_size > end ||
        segment_table_offset + sizeof(SegmentTableEntry) * segment_table_num > end ||
        symbo_export_table_offset + sizeof(SymbolExportTableEntry) * symbo_export_table_num > end ||
        index_export_table_offset + sizeof(IndexExportTableEntry) * index_export_table_num > end ||
        export_strings_offset + export_strings_num > end ||
        export_tree_offset + sizeof(ExportTreeEntry) * export_tree_num > end ||
        object_info_offset + sizeof(ObjectInfo) * object_info_num > end ||
        external_relocation_table_offset + sizeof(ExternalRelocationTableEntry) * external_relocation_table_num > end ||
        symbo_import_table_offset + sizeof(ImportTableEntry) * symbo_import_table_num > end ||
        index_import_table_offset + sizeof(ImportTableEntry) * index_import_table_num > end ||
        offset_import_table_offset + sizeof(ImportTableEntry) * offset_import_table_num > end ||
        import_strings_offset + import_strings_num > end ||
        offset_export_table_offset + sizeof(OffsetExportTableEntry) * offset_export_table_num > end ||
        internal_relocation_table_offset + sizeof(ExternalRelocationTableEntry) * internal_relocation_table_num > end ||
        static_relocation_table_offset + 12 * static_relocation_table_num > end) {
            return false;
    }

    return true;
}

static void ApplyPatch(ExternalRelocationTableEntry* patch, u32 patch_base, u32 patch_address, u32* patch_address1 = nullptr) {
    if (!patch_address1)
        patch_address1 = &patch_address;


    switch (patch->type) {
        case 2:
            Memory::Write32(patch_address, patch_base + patch->x);
            break;
        case 3:
            Memory::Write32(patch_address, patch_base + patch->x - *patch_address1);
            break;
        default:
            LOG_CRITICAL(Service_APT, "Unknown patch type %u", patch->type);
    }
}

static void ApplyImportPatches(CROHeader& header, u32 base) {
    u32 patch_base = 0;

    if (header.GetImportPatchesTargetSegment() < header.segment_table_num) {
        SegmentTableEntry base_segment = header.GetSegmentTableEntry(header.GetImportPatchesTargetSegment());
        patch_base = base_segment.segment_offset + header.GetImportPatchesSegmentOffset();
    }

    u32 v10 = 1;
    for (int i = 0; i < header.external_relocation_table_num; ++i) {
        ExternalRelocationTableEntry* patch = header.GetExternalRelocationTableEntry(i);
        SegmentTableEntry target_segment = header.GetSegmentTableEntry(patch->GetTargetSegment());
        ApplyPatch(patch, patch_base, target_segment.segment_offset + patch->GetSegmentOffset());
        if (v10)
            patch->unk2 = 0;
        v10 = patch->unk;
    }
}

static void ApplyListPatches(CROHeader& header, ExternalRelocationTableEntry* first_patch, u32 patch_base) {
    ExternalRelocationTableEntry* current_patch = first_patch;

    while (current_patch) {
        SegmentTableEntry target_segment = header.GetSegmentTableEntry(current_patch->GetTargetSegment());
        ApplyPatch(current_patch, patch_base, target_segment.segment_offset + current_patch->GetSegmentOffset());

        if (current_patch->unk)
            break;
        ++current_patch;
    }

    first_patch->unk2 = 1;
}

static void ApplyOffsetExportTablePatches(CROHeader& header, CROHeader* crs, u32 base) {
    for (int i = 0; i < header.offset_export_table_num; ++i) {
        OffsetExportTableEntry* patch = header.GetOffsetExportTableEntry(i);
        SegmentTableEntry segment = header.GetSegmentTableEntry(patch->GetTargetSegment());
        u32 patch_base = segment.segment_offset + patch->GetSegmentOffset();
        u32 patches_table = base + patch->patches_offset;

        ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(patches_table));
        ApplyListPatches(*crs, first_patch, patch_base);
    }
}

static void ApplyRelocationPatches(CROHeader& header, u32 base, u32 section0) {
    for (int i = 0; i < header.internal_relocation_table_num; ++i) {
        ExternalRelocationTableEntry* patch = header.GetRelocationPatchEntry(i);
        u32 segment_id = patch->GetTargetSegment();
        SegmentTableEntry target_segment = header.GetSegmentTableEntry(segment_id);
        u32 target_segment_offset = target_segment.segment_offset;

        if (target_segment.segment_id == 2)
            target_segment_offset = section0;

        SegmentTableEntry base_segment = header.GetSegmentTableEntry(patch->unk);

        u32 patch_address = target_segment_offset + patch->GetSegmentOffset();
        u32 patch_address1 = target_segment.segment_offset + patch->GetSegmentOffset();

        ApplyPatch(patch, base_segment.segment_offset, patch_address, &patch_address1);
    }
}

static const u32 EXPORT_TABLE_FINISHED = 0x8000;

static u32 FindExportByName(CROHeader& header, char* str) {
    if (header.export_tree_num) {
        ExportTreeEntry* first_entry = header.GetExportTreeEntry(0);
        u32 len = strlen(str);
        ExportTreeEntry* next_entry = header.GetExportTreeEntry(first_entry->next);
        bool run = false;
        while (!run) {
            u16 next_offset = 0;
            u32 next_len = next_entry->GetSegmentOffset();
            u8 next_segment = str[next_len];
            if (next_len >= len)
                next_offset = next_entry->next;
            else if (!((next_segment >> next_entry->GetTargetSegment()) & 1))
                next_offset = next_entry->next;
            else
                next_offset = next_entry->next_level;
            run = next_offset & EXPORT_TABLE_FINISHED;
            next_entry = header.GetExportTreeEntry(next_offset & ~EXPORT_TABLE_FINISHED);
        }

        u32 export_id = next_entry->export_table_id;
        SymbolExportTableEntry* export_entry = header.GetSymbolExportTableEntry(export_id);
        char* export_name = (char*)Memory::GetPointer(export_entry->name_offset);
        if (!strcmp(export_name, str)) {
            SegmentTableEntry segment = header.GetSegmentTableEntry(export_entry->GetTargetSegment());
            return segment.segment_offset + export_entry->GetSegmentOffset();
        }
    }
    return 0;
}

static void ApplyExitPatches(CROHeader& header, CROHeader* crs, u32 base) {
    // Find the "__aeabi_atexit" in the import table 1
    for (int i = 0; i < header.symbo_import_table_num; ++i) {
        ImportTableEntry* entry = header.GetSymbolImportTableEntry(i);
        // The name is already relocated
        char* entry_name = reinterpret_cast<char*>(Memory::GetPointer(entry->name_offset));
        if (!strcmp(entry_name, "__aeabi_atexit")) {
            // Only apply these patches if the CRS exports "nnroAeabiAtexit_"
            u32 offset = FindExportByName(*crs, "nnroAeabiAtexit_");

            ASSERT_MSG(offset, "Could not find nnroAeabiAtexit_ in the CRS");

            // Patch it!
            ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
            ApplyListPatches(header, first_patch, offset);
            return;
        }
    }
}

static void BackSymbolImportTablePatches(CROHeader& old_header, CROHeader& new_header) {
    for (int i = 0; i < old_header.symbo_import_table_num; ++i) {
        ImportTableEntry* entry = old_header.GetSymbolImportTableEntry(i);
        ExternalRelocationTableEntry* patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
        if (!patch->unk2) {
            u32 patch_base = FindExportByName(new_header, (char*)Memory::GetPointer(entry->name_offset));

            if (patch_base) {
                ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
                ApplyListPatches(old_header, first_patch, patch_base);
            }
        }
    }
}

static void ApplySymbolImportTablePatches(CROHeader& header, CROHeader* head) {
    for (int i = 0; i < header.symbo_import_table_num; ++i) {
        ImportTableEntry* entry = header.GetSymbolImportTableEntry(i);
        ExternalRelocationTableEntry* patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
        if (!patch->unk2) {
            u32 patch_base = 0;
            while (head) {
                patch_base = FindExportByName(*head, (char*)Memory::GetPointer(entry->name_offset));
                if (patch_base)
                    break;
                head = reinterpret_cast<CROHeader*>(Memory::GetPointer(head->next_cro));
            }
            if (patch_base) {
                ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
                ApplyListPatches(header, first_patch, patch_base);
            }
        }
    }
}

static u32 GetCROBaseByName(char* name) {
    u32 base = the_crs_base;
    while (base) {
        CROHeader* header = reinterpret_cast<CROHeader*>(Memory::GetPointer(base));
        char* cro_name = reinterpret_cast<char*>(Memory::GetPointer(header->name_offset));

        if (!strcmp(cro_name, name))
            return base;
        base = header->next_cro;
    }
    return 0;
}

static void ApplyObjectInfoPatches(CROHeader& header, u32 base) {
    for (int i = 0; i < header.object_info_num; ++i) {
        ObjectInfo* entry = header.GetObjectInfoEntry(i);
        u32 cro_base = GetCROBaseByName(reinterpret_cast<char*>(Memory::GetPointer(entry->string_offset)));
        if (cro_base == 0)
            continue;

        CROHeader* patch_cro = reinterpret_cast<CROHeader*>(Memory::GetPointer(cro_base));
        // Apply the patches from the first table
        for (int j = 0; j < entry->table1_num; ++j) {
            Unk2TableEntry* table1_entry = entry->GetTable1Entry(j);
            u32 unk1_table_entry = patch_cro->GetIndexExportTableEntry(table1_entry->offset_or_index);
            u32 base_segment_id = unk1_table_entry & 0xF;
            u32 base_segment_offset = unk1_table_entry >> 4;
            SegmentTableEntry base_segment = patch_cro->GetSegmentTableEntry(base_segment_id);

            ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(table1_entry->patches_offset));
            ApplyListPatches(header, first_patch, base_segment.segment_offset + base_segment_offset);
        }

        // Apply the patches from the second table
        for (int j = 0; j < entry->table2_num; ++j) {
            Unk2TableEntry* table2_entry = entry->GetTable2Entry(j);
            u32 base_segment_id = table2_entry->offset_or_index & 0xF;
            u32 base_segment_offset = table2_entry->offset_or_index >> 4;
            SegmentTableEntry base_segment = patch_cro->GetSegmentTableEntry(base_segment_id);

            ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(table2_entry->patches_offset));
            ApplyListPatches(header, first_patch, base_segment.segment_offset + base_segment_offset);
        }
    }
}

static void BackApplyObjectInfoPatches(CROHeader& header, u32 base, CROHeader& patch_cro, u32 new_base) {
    for (int i = 0; i < header.object_info_num; ++i) {
        ObjectInfo* entry = header.GetObjectInfoEntry(i);
        char* old_cro_name = reinterpret_cast<char*>(Memory::GetPointer(entry->string_offset));
        char* new_cro_name = reinterpret_cast<char*>(Memory::GetPointer(patch_cro.name_offset));
        if (strcmp(old_cro_name, new_cro_name) != 0)
            continue;

        // Apply the patches from the first table
        for (int j = 0; j < entry->table1_num; ++j) {
            Unk2TableEntry* table1_entry = entry->GetTable1Entry(j);
            u32 unk1_table_entry = patch_cro.GetIndexExportTableEntry(table1_entry->offset_or_index);
            u32 base_segment_id = unk1_table_entry & 0xF;
            u32 base_segment_offset = unk1_table_entry >> 4;
            SegmentTableEntry base_segment = patch_cro.GetSegmentTableEntry(base_segment_id);

            ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(table1_entry->patches_offset));
            ApplyListPatches(header, first_patch, base_segment.segment_offset + base_segment_offset);
        }

        // Apply the patches from the second table
        for (int j = 0; j < entry->table2_num; ++j) {
            Unk2TableEntry* table2_entry = entry->GetTable2Entry(j);
            u32 base_segment_id = table2_entry->offset_or_index & 0xF;
            u32 base_segment_offset = table2_entry->offset_or_index >> 4;
            SegmentTableEntry base_segment = patch_cro.GetSegmentTableEntry(base_segment_id);

            ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(table2_entry->patches_offset));
            ApplyListPatches(header, first_patch, base_segment.segment_offset + base_segment_offset);
        }
    }
}

static void LoadExportsTable(CROHeader& header, u32 base) {
    for (int i = 0; i < header.symbo_export_table_num; ++i) {
        SymbolExportTableEntry* entry = header.GetSymbolExportTableEntry(i);
        SegmentTableEntry target_segment = header.GetSegmentTableEntry(entry->GetTargetSegment());
        ExportedSymbol export_;
        export_.cro_base = base;
        export_.cro_offset = target_segment.segment_offset + entry->GetSegmentOffset();
        export_.name = reinterpret_cast<char*>(Memory::GetPointer(entry->name_offset));
        loaded_exports[export_.name] = export_;
    }
}

static void LinkCROs(CROHeader* crs, CROHeader& new_cro, u32 new_cro_base) {
    if (crs->next_cro) {
        auto first_cro = reinterpret_cast<CROHeader*>(Memory::GetPointer(crs->next_cro));
        u32 last_cro_base = first_cro->previous_cro; // note that previous_cro of the first cro is always pointing to the last cro
        auto last_cro = reinterpret_cast<CROHeader*>(Memory::GetPointer(last_cro_base));

        // link with the last cro
        new_cro.previous_cro = last_cro_base;
        last_cro->next_cro = new_cro_base;

        // the new one is the last one
        new_cro.next_cro = 0;

        first_cro->previous_cro = new_cro_base; // set previous_cro of the first cro pointing to the new last cro

    } else {
        new_cro.next_cro = 0; // the new one is the last one
        new_cro.previous_cro = new_cro_base; // set previous_cro of the first cro pointing to the last cro (i.e. itself)
        crs->next_cro = new_cro_base;
    }

}

void LogCRO(u32 base, CROHeader& header) {
    LOG_INFO(Service_LDR, ">>>> log CRO <<<<");
    std::string module_name = std::string((char*)Memory::GetPointer(header.module_name_offset));
    LOG_INFO(Service_LDR, "\tmodule_name = %s", module_name.c_str(), header.module_name_offset);
    std::string export_string = std::string((char*)Memory::GetPointer(header.export_strings_offset));
    LOG_INFO(Service_LDR, "\texport_string = %s", export_string.c_str());
    LOG_INFO(Service_LDR, "<<<< log CRO >>>>");
}

void LogCROList() {
    CROHeader* crs = reinterpret_cast<CROHeader*>(Memory::GetPointer(the_crs_base));
    u32 first = crs->next_cro, next = first, last = 0;
    LOG_INFO(Service_LDR, ">>>> log list <<<<");
    while (next) {
        CROHeader* cro = reinterpret_cast<CROHeader*>(Memory::GetPointer(next));
        std::string module_name = std::string((char*)Memory::GetPointer(cro->module_name_offset));
        LOG_INFO(Service_LDR, "\t%s", module_name.c_str());
        last = next;
        next = cro->next_cro;
    }
    if (last)
        ASSERT(reinterpret_cast<CROHeader*>(Memory::GetPointer(first))->previous_cro == last);
    LOG_INFO(Service_LDR, "<<<< log list >>>>");
}

void DumpCRO(CROHeader& cro) {
    std::FILE* f = std::fopen((std::string("/home/wwylele/3ds_ro/old_dump/") +
        (char*)Memory::GetPointer(cro.module_name_offset)).data(), "wb");
    std::fwrite(&cro, cro.file_size, 1, f);
    std::fclose(f);

    u32 segment_num = cro.segment_table_num;
    for (u32 i = 0; i < segment_num ; ++i) {
        LOG_INFO(Service_LDR,"%d", i);
        SegmentTableEntry entry = cro.GetSegmentTableEntry(i);
        if(Memory::GetPointer(entry.segment_offset)) {
            std::FILE* f = std::fopen((std::string("/home/wwylele/3ds_ro/old_dump/") + (char*)Memory::GetPointer(cro.module_name_offset) + (char)('0'+i)).data(), "wb");
            if (!f) UNREACHABLE();
            std::fwrite(Memory::GetPointer(entry.segment_offset), entry.segment_size, 1, f);
            std::fclose(f);
        }
    }
}

static ResultCode LoadCRO(u32 base, u32 size, CROHeader& header, u32 data_section, u32 bss_section, bool is_crs) {
    // Relocate all offsets
    if (!header.VerifyAndRelocateOffsets(base, size))
        return ResultCode(0xD9012C11);

    if (header.module_name_size &&
        Memory::Read8(header.module_name_offset + header.module_name_size - 1) != 0) {
        // The module name must end with '\0'
        return ResultCode(0xD9012C0B);
    }

    u32 prev_data_section = 0;
    ResultCode result = RESULT_SUCCESS;

    if (!is_crs) {
        // Relocate segments
        result = header.RelocateSegmentsTable(base, size, data_section, bss_section, prev_data_section);
        if (result.IsError())
            return result;
    }

    // Rebase export table
    result = header.RelocateExportsTable(base);

    if (result.IsError())
        return result;

    if (header.export_strings_num &&
        Memory::Read8(header.export_strings_offset + header.export_strings_num - 1) != 0)
        return ResultCode(0xD9012C0B);

    CROHeader* crs = nullptr;
    if (is_crs)
        crs = &header;
    else
        crs = reinterpret_cast<CROHeader*>(Memory::GetPointer(the_crs_base));

    // Rebase objects
    header.RelocateObjectInfos(base);

    // Apply import patches
    ApplyImportPatches(header, base);

    // Rebase symbol import table name & symbol offsets
    header.RelocateSymbolImportTable(base);

    // Rebase index & offset table symbol offsets
    header.RelocateIndexImportTable(base);
    header.RelocateOffsetImportTable(base);

    if (!is_crs) {
        // Apply offset export table patches
        ApplyOffsetExportTablePatches(header, crs, base);
    }

    // Apply relocation patches
    ApplyRelocationPatches(header, base, prev_data_section + base);

    // Apply import table 1 patches
    ApplyExitPatches(header, crs, base);

    // Import Symbo Table
    ApplySymbolImportTablePatches(header, crs);

    // Load exports
    LoadExportsTable(header, base);

    if (!is_crs) {
        // Apply object patches
        ApplyObjectInfoPatches(header, base);

        // Retroactively apply import table 1 patches to the previous CROs
        // Retroactively apply object patches to the previous CROs
        u32 crs_base = the_crs_base;
        CROHeader* crs = reinterpret_cast<CROHeader*>(Memory::GetPointer(crs_base));
        if (crs->next_cro) {
            u32 first_cro_base = crs->next_cro;
            CROHeader* first_cro = reinterpret_cast<CROHeader*>(Memory::GetPointer(first_cro_base));
            u32 cro_base = first_cro->previous_cro;
            while (1) {
                CROHeader* cro_header = reinterpret_cast<CROHeader*>(Memory::GetPointer(cro_base));
                BackSymbolImportTablePatches(*cro_header, header);
                BackApplyObjectInfoPatches(*cro_header, cro_base, header, base);
                if (cro_base == first_cro_base) break;
                cro_base = cro_header->previous_cro;
            }
        }
        BackSymbolImportTablePatches(*crs, header);
        BackApplyObjectInfoPatches(*crs, crs_base, header, base);

    }

    if (!is_crs) {
        // Link the CROs
        LinkCROs(reinterpret_cast<CROHeader*>(Memory::GetPointer(the_crs_base)), header, base);
    } else {
        the_crs_base = base;
    }



    LOG_INFO(Service_LDR, "Load CRO:");
    LogCRO(base, header);
    LogCROList();

    // Clear the instruction cache
    Core::g_app_core->ClearInstructionCache();

    return RESULT_SUCCESS;
}

/**
 * LDR_RO::Initialize service function
 *  Inputs:
 *      1 : CRS buffer pointer
 *      2 : CRS Size
 *      3 : Process memory address where the CRS will be mapped
 *      4 : Value, must be zero
 *      5 : KProcess handle
 *  Outputs:
 *      0 : Return header
 *      1 : Result of function, 0 on success, otherwise error code
 */
static void Initialize(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u8* crs_buffer_ptr = Memory::GetPointer(cmd_buff[1]);
    u32 crs_size       = cmd_buff[2];
    u32 address        = cmd_buff[3];
    u32 value          = cmd_buff[4];
    u32 process        = cmd_buff[5];

    if (value != 0) {
        LOG_WARNING(Service_LDR, "This value should be zero, but is actually %u!", value);
    }

    LOG_WARNING(Service_LDR, "(STUBBED) called. crs_buffer_ptr=0x%08X, crs_size=0x%08X, address=0x%08X, value=0x%08X, process=0x%08X",
                crs_buffer_ptr, crs_size, address, value, process);

    loaded_exports.clear();
    the_crs_base = 0;

    std::shared_ptr<std::vector<u8>> cro = std::make_shared<std::vector<u8>>(crs_size);
    memcpy(cro->data(), crs_buffer_ptr, crs_size);

    // TODO(Subv): Check what the real hardware returns for MemoryState
    auto map_result = Kernel::g_current_process->vm_manager.MapMemoryBlock(address, cro, 0, crs_size, Kernel::MemoryState::Code);

    cmd_buff[0] = IPC::MakeHeader(1, 1, 0);

    if (map_result.Failed()) {
        LOG_ERROR(Service_LDR, "Error mapping memory block %08X", map_result.Code().raw);
        cmd_buff[1] = map_result.Code().raw;
        return;
    }

    CROHeader *header = reinterpret_cast<CROHeader *>(Memory::GetPointer(address));
    //memcpy(&header, Memory::GetPointer(address), sizeof(CROHeader));

    ResultCode result = LoadCRO(address, crs_size, *header, 0, 0, true);
    cmd_buff[1] = result.raw;

    if (result != RESULT_SUCCESS) {
        LOG_ERROR(Service_LDR, "Error loading CRS %08X", result.raw);
        return;
    }

    //memcpy(Memory::GetPointer(address), &header, sizeof(CROHeader));
}

/**
 * LDR_RO::LoadCRR service function
 *  Inputs:
 *      1 : CRR buffer pointer
 *      2 : CRR Size
 *      3 : Value, must be zero
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
        LOG_WARNING(Service_LDR, "This value should be zero, but is actually %u!", value);
    }

    cmd_buff[0] = IPC::MakeHeader(2, 1, 0);
    cmd_buff[1] = RESULT_SUCCESS.raw;

    LOG_WARNING(Service_LDR, "(STUBBED) called. crs_buffer_ptr=0x%08X, crs_size=0x%08X, value=0x%08X, process=0x%08X",
                crs_buffer_ptr, crs_size, value, process);
}

struct FixEndInfo {
    u32 fix0_end;
    u32 fix1_end;
    u32 fix2_end;
    u32 fix3_end;
    u32 unk4;
};

static FixEndInfo GetFixEndInfo(CROHeader& cro, u32 fix_level) {
    u32 v2 = cro.code_offset + cro.code_size;

    if (v2 <= 0x138)
        v2 = 0x138;

    FixEndInfo ret;

    v2 = std::max<u32>(v2, cro.module_name_offset + cro.module_name_size);
    v2 = std::max<u32>(v2, cro.segment_table_offset + sizeof(SegmentTableEntry) * cro.segment_table_num);

    ret.fix3_end = v2;

    v2 = std::max<u32>(v2, cro.symbo_export_table_offset + sizeof(SymbolExportTableEntry) * cro.symbo_export_table_num);
    v2 = std::max<u32>(v2, cro.index_export_table_offset + sizeof(IndexExportTableEntry) * cro.index_export_table_num);
    v2 = std::max<u32>(v2, cro.export_strings_offset + cro.export_strings_num);
    v2 = std::max<u32>(v2, cro.export_tree_offset + sizeof(ExportTreeEntry) * cro.export_tree_num);

    ret.fix2_end = v2;

    v2 = std::max<u32>(v2, cro.object_info_offset + sizeof(ObjectInfo) * cro.object_info_offset);
    v2 = std::max<u32>(v2, cro.external_relocation_table_offset + sizeof(ExternalRelocationTableEntry) * cro.external_relocation_table_num);
    v2 = std::max<u32>(v2, cro.symbo_import_table_offset + sizeof(ImportTableEntry) * cro.symbo_import_table_num);
    v2 = std::max<u32>(v2, cro.index_import_table_offset + sizeof(ImportTableEntry) * cro.index_import_table_num);
    v2 = std::max<u32>(v2, cro.offset_import_table_offset + sizeof(ImportTableEntry) * cro.offset_import_table_num);
    v2 = std::max<u32>(v2, cro.import_strings_offset + cro.import_strings_num);

    ret.fix1_end = v2;

    v2 = std::max<u32>(v2, cro.static_relocation_table_offset + 12 * cro.static_relocation_table_num);
    v2 = std::max<u32>(v2, cro.offset_export_table_offset + sizeof(OffsetExportTableEntry) * cro.offset_export_table_num);
    v2 = std::max<u32>(v2, cro.internal_relocation_table_offset + sizeof(ExternalRelocationTableEntry) * cro.internal_relocation_table_num);

    ret.fix0_end = v2;

    ret.unk4 = 0;
    return ret;
}

static void LoadExeCRO(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u8* cro_buffer = Memory::GetPointer(cmd_buff[1]);
    u32 address = cmd_buff[2];
    u32 size = cmd_buff[3];

    u32 level = cmd_buff[10];

    bool link = cmd_buff[9] & 0xFF;

    ASSERT_MSG(link, "Link must be set");

    std::shared_ptr<std::vector<u8>> cro = std::make_shared<std::vector<u8>>(size);
    memcpy(cro->data(), cro_buffer, size);

    // TODO(Subv): Check what the real hardware returns for MemoryState
    auto map_result = Kernel::g_current_process->vm_manager.MapMemoryBlock(address, cro, 0, size, Kernel::MemoryState::Code);

    cmd_buff[0] = IPC::MakeHeader(4, 2, 0);

    if (map_result.Failed()) {
        LOG_CRITICAL(Service_LDR, "Error when mapping memory: %08X", map_result.Code().raw);
        cmd_buff[1] = map_result.Code().raw;
        return;
    }

    CROHeader& header = *reinterpret_cast<CROHeader *>(Memory::GetPointer(address));
    //memcpy(&header, Memory::GetPointer(address), sizeof(CROHeader));

    ResultCode result = LoadCRO(address, size, header, cmd_buff[4], cmd_buff[7], false);
    cmd_buff[1] = result.raw;

    if (result.IsError()) {
        LOG_CRITICAL(Service_LDR, "Error when loading CRO %08X", result.raw);
        return;
    }

    cmd_buff[2] = 0;

    auto fix_size = GetFixEndInfo(header, level);
    u32 fix_end = fix_size.fix0_end;

    switch (level) {
    case 1:
        fix_end = fix_size.fix1_end;
        break;
    case 2:
        fix_end = fix_size.fix2_end;
        break;
    case 3:
        fix_end = fix_size.fix3_end;
        break;
    default:
        break;
    }

    memcpy(header.magic, "FIXD", 4);
    header.offset_export_table_offset = fix_end;
    header.offset_export_table_num = 0;
    header.internal_relocation_table_offset = fix_end;
    header.internal_relocation_table_num = 0;
    header.static_relocation_table_offset = fix_end;
    header.static_relocation_table_num = 0;

    if (level >= 2) {
        header.object_info_offset = fix_end;
        header.object_info_num = 0;
        header.external_relocation_table_offset = fix_end;
        header.external_relocation_table_num = 0;
        header.symbo_import_table_offset = fix_end;
        header.symbo_import_table_num = 0;
        header.index_import_table_offset = fix_end;
        header.index_import_table_num = 0;
        header.offset_import_table_offset = fix_end;
        header.offset_import_table_num = 0;
        header.import_strings_offset = fix_end;
        header.import_strings_num = 0;

        if (level >= 3) {
            header.symbo_export_table_offset = fix_end;
            header.symbo_export_table_num = 0;
            header.index_export_table_offset = fix_end;
            header.index_export_table_num = 0;
            header.export_strings_offset = fix_end;
            header.export_strings_num = 0;
            header.export_tree_offset = fix_end;
            header.export_tree_num = 0;
        }
    }

    u32 fix_end_round = (fix_end + 0xFFF) >> 12 << 12;
    header.fixed_size = cmd_buff[2] = fix_end_round - address;

    //memcpy(Memory::GetPointer(address), &header, sizeof(CROHeader));
        if(std::strcmp((char*)Memory::GetPointer(header.module_name_offset), "DllField")==0){
            DumpCRO(header);
            DumpCRO(*reinterpret_cast<CROHeader*>(Memory::GetPointer(the_crs_base)));
        }

    LOG_WARNING(Service_LDR, "Loading CRO address=%08X level=%08X", address, level);
}

static void UnlinkCRO(CROHeader* crs, CROHeader* cro, u32 address) {
    u32 phead_base = crs->previous_cro; // "manual relocate" head
    u32 nhead_base = crs->next_cro; // "auto relocate" head

    if (phead_base == address || nhead_base == address) {
        // Remove head item
        LOG_INFO(Service_LDR, "unlink head CRO");
        auto next_base = cro->next_cro;
        if (next_base) {
            auto next = reinterpret_cast<CROHeader*>(Memory::GetPointer(next_base));
            next->previous_cro = cro->previous_cro;
        }

        if (phead_base == address) {
            crs->previous_cro = next_base;
        } else {
            crs->next_cro = next_base;
        }
    }
    else if (cro->next_cro) {
        // Remove body item
        LOG_INFO(Service_LDR, "unlink body CRO");
        u32 prev_base = cro->previous_cro;
        u32 next_base = cro->next_cro;
        auto prev = reinterpret_cast<CROHeader*>(Memory::GetPointer(prev_base));
        auto next = reinterpret_cast<CROHeader*>(Memory::GetPointer(next_base));
        prev->next_cro = next_base;
        next->previous_cro = prev_base;
    } else {
        // Remove tail item
        LOG_INFO(Service_LDR, "unlink tail CRO");
        u32 prev_base = cro->previous_cro;
        auto prev = reinterpret_cast<CROHeader*>(Memory::GetPointer(prev_base));
        prev->next_cro = 0;
        CROHeader* nhead;
        CROHeader* phead;
        if (nhead_base && (nhead = reinterpret_cast<CROHeader*>(Memory::GetPointer(nhead_base)))->previous_cro == address) {
            nhead->previous_cro = prev_base;
        } else if (phead_base && (phead = reinterpret_cast<CROHeader*>(Memory::GetPointer(phead_base)))->previous_cro == address) {
            phead->previous_cro = prev_base;
        } else {
            UNREACHABLE();
        }
    }

    cro->previous_cro = 0;
    cro->next_cro = 0;
}

static void UnloadExternalRelocationPatches(CROHeader* cro, ExternalRelocationTableEntry* first_patch, u32 base_offset) {
    ExternalRelocationTableEntry* patch = first_patch;
    while (patch) {
        SegmentTableEntry target_segment = cro->GetSegmentTableEntry(patch->GetTargetSegment());
        ApplyPatch(patch, base_offset, target_segment.segment_offset + patch->GetSegmentOffset());

        if (patch->unk)
            break;

        patch++;
    }

    first_patch->unk2 = 0;
}

static u32 CalculateBaseOffset(CROHeader* cro) {
    u32 base_offset = 0;

    if (cro->GetImportPatchesTargetSegment() < cro->segment_table_num) {
        SegmentTableEntry base_segment = cro->GetSegmentTableEntry(cro->GetImportPatchesTargetSegment());
        if (cro->GetImportPatchesSegmentOffset() < base_segment.segment_size)
            base_offset = base_segment.segment_offset + cro->GetImportPatchesSegmentOffset();
    }

    return base_offset;
}

static void UnloadSymbolImportTablePatches(CROHeader* cro, u32 base_offset) {
    for (int i = 0; i < cro->symbo_import_table_num; ++i) {
        ImportTableEntry* entry = cro->GetSymbolImportTableEntry(i);
        ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
        UnloadExternalRelocationPatches(cro, first_patch, base_offset);
    }
}

static void UnloadIndexImportTablePatches(CROHeader* cro, u32 base_offset) {
    for (int i = 0; i < cro->index_import_table_num; ++i) {
        ImportTableEntry* entry = cro->GetIndexImportTableEntry(i);
        ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
        UnloadExternalRelocationPatches(cro, first_patch, base_offset);
    }
}

static void UnloadOffsetImportTablePatches(CROHeader* cro, u32 base_offset) {
    for (int i = 0; i < cro->offset_import_table_num; ++i) {
        ImportTableEntry* entry = cro->GetOffsetImportTableEntry(i);
        ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
        UnloadExternalRelocationPatches(cro, first_patch, base_offset);
    }
}

static void ApplyCRSImportTable1UnloadPatches(CROHeader* crs, CROHeader& unload, u32 base_offset) {
    for (int i = 0; i < crs->symbo_import_table_num; ++i) {
        ImportTableEntry* entry = crs->GetSymbolImportTableEntry(i);
        ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(entry->symbol_offset));
        if (first_patch->unk2)
            if (FindExportByName(unload, reinterpret_cast<char*>(Memory::GetPointer(entry->name_offset))))
                UnloadExternalRelocationPatches(crs, first_patch, base_offset);
    }
}

static void UnloadObjectInfoPatches(CROHeader* cro, CROHeader* unload, u32 base_offset) {
    char* unload_name = reinterpret_cast<char*>(Memory::GetPointer(unload->name_offset));
    for (int i = 0; i < cro->object_info_num; ++i) {
        ObjectInfo* entry = cro->GetObjectInfoEntry(i);
        // Find the patch that corresponds to the CRO that is being unloaded
        if (strcmp(reinterpret_cast<char*>(Memory::GetPointer(entry->string_offset)), unload_name) == 0) {

            // Apply the table 1 patches
            for (int j = 0; j < entry->table1_num; ++j) {
                Unk2TableEntry* table1_entry = entry->GetTable1Entry(j);
                ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(table1_entry->patches_offset));
                UnloadExternalRelocationPatches(cro, first_patch, base_offset);
            }

            // Apply the table 2 patches
            for (int j = 0; j < entry->table2_num; ++j) {
                Unk2TableEntry* table2_entry = entry->GetTable2Entry(j);
                ExternalRelocationTableEntry* first_patch = reinterpret_cast<ExternalRelocationTableEntry*>(Memory::GetPointer(table2_entry->patches_offset));
                UnloadExternalRelocationPatches(cro, first_patch, base_offset);
            }
            break;
        }
    }
}

static void ApplyCRSUnloadPatches(CROHeader* crs, CROHeader& unload) {
    u32 base_offset = CalculateBaseOffset(crs);

    ApplyCRSImportTable1UnloadPatches(crs, unload, base_offset);
}

static void UnrebaseOffsetImportTable(CROHeader* cro, u32 address) {
    for (int i = 0; i < cro->offset_import_table_num; ++i) {
        ImportTableEntry* entry = cro->GetOffsetImportTableEntry(i);
        if (entry->symbol_offset)
            entry->symbol_offset -= address;
    }
}

static void UnrebaseIndexImportTable(CROHeader* cro, u32 address) {
    for (int i = 0; i < cro->index_import_table_num; ++i) {
        ImportTableEntry* entry = cro->GetIndexImportTableEntry(i);
        if (entry->symbol_offset)
            entry->symbol_offset -= address;
    }
}

static void UnrebaseSymbolImportTable(CROHeader* cro, u32 address) {
    for (int i = 0; i < cro->symbo_import_table_num; ++i) {
        ImportTableEntry* entry = cro->GetSymbolImportTableEntry(i);
        if (entry->name_offset)
            entry->name_offset -= address;
        if (entry->symbol_offset)
            entry->symbol_offset -= address;
    }
}

static void UnrebaseObjectInfoPatches(CROHeader* cro, u32 address) {
    for (int i = 0; i < cro->object_info_num; ++i) {
        ObjectInfo* entry = cro->GetObjectInfoEntry(i);
        if (entry->string_offset)
            entry->string_offset -= address;
        if (entry->table1_offset)
            entry->table1_offset -= address;
        if (entry->table2_offset)
            entry->table2_offset -= address;
    }
}

static void UnrebaseSymbolExportsTable(CROHeader* cro, u32 address) {
    for (int i = 0; i < cro->symbo_export_table_num; ++i) {
        SymbolExportTableEntry* entry = cro->GetSymbolExportTableEntry(i);
        if (entry->name_offset)
            entry->name_offset -= address;
    }
}

static void UnrebaseSegments(CROHeader* cro, u32 address) {
    for (int i = 0; i < cro->segment_table_num; ++i) {
        SegmentTableEntry entry = cro->GetSegmentTableEntry(i);
        if (entry.segment_id == 3)
            entry.segment_offset = 0;
        else if (entry.segment_id)
            entry.segment_offset -= address;
        cro->SetSegmentTableEntry(i, entry);
    }
}

static void UnrebaseCRO(CROHeader* cro, u32 address) {

    UnrebaseOffsetImportTable(cro, address);
    UnrebaseIndexImportTable(cro, address);
    UnrebaseSymbolImportTable(cro, address);
    UnrebaseObjectInfoPatches(cro, address);
    UnrebaseSymbolExportsTable(cro, address);
    UnrebaseSegments(cro, address);

    if (cro->name_offset)
        cro->name_offset -= address;

    if (cro->code_offset)
        cro->code_offset -= address;

    if (cro->data_offset)
        cro->data_offset -= address;

    if (cro->module_name_offset)
        cro->module_name_offset -= address;

    if (cro->segment_table_offset)
        cro->segment_table_offset -= address;

    if (cro->symbo_export_table_offset)
        cro->symbo_export_table_offset -= address;

    if (cro->index_export_table_offset)
        cro->index_export_table_offset -= address;

    if (cro->export_strings_offset)
        cro->export_strings_offset -= address;

    if (cro->export_tree_offset)
        cro->export_tree_offset -= address;

    if (cro->object_info_offset)
        cro->object_info_offset -= address;

    if (cro->external_relocation_table_offset)
        cro->external_relocation_table_offset -= address;

    if (cro->symbo_import_table_offset)
        cro->symbo_import_table_offset -= address;

    if (cro->index_import_table_offset)
        cro->index_import_table_offset -= address;

    if (cro->offset_import_table_offset)
        cro->offset_import_table_offset -= address;

    if (cro->import_strings_offset)
        cro->import_strings_offset -= address;

    if (cro->offset_export_table_offset)
        cro->offset_export_table_offset -= address;

    if (cro->internal_relocation_table_offset)
        cro->internal_relocation_table_offset -= address;

    if (cro->static_relocation_table_offset)
        cro->static_relocation_table_offset -= address;
}

static void UnloadExports(u32 address) {
    for (auto itr = loaded_exports.begin(); itr != loaded_exports.end();) {
        if (itr->second.cro_base == address)
            itr = loaded_exports.erase(itr);
        else
            ++itr;
    }
}

static ResultCode UnloadCRO(u32 address) {

    CROHeader* crs = reinterpret_cast<CROHeader*>(Memory::GetPointer(the_crs_base));

    // If there's only one loaded CRO, it must be the CRS, which can not be unloaded like this
    if (!crs->next_cro) {
        return ResultCode(0xD9012C1E);
    }

    CROHeader* unload = reinterpret_cast<CROHeader*>(Memory::GetPointer(address));
    u32 size = unload->file_size;

    UnlinkCRO(crs, unload, address);

    LOG_INFO(Service_LDR, "Unload CRO:");
    LogCRO(address, *unload);
    LogCROList();

    u32 base_offset = CalculateBaseOffset(unload);

    UnloadSymbolImportTablePatches(unload, base_offset);
    UnloadIndexImportTablePatches(unload, base_offset);
    UnloadOffsetImportTablePatches(unload, base_offset);

    u32 base = the_crs_base;
    while (base) {
        CROHeader* cro = reinterpret_cast<CROHeader*>(Memory::GetPointer(base));
        if (base != address) {
            ApplyCRSUnloadPatches(cro, *unload);
            base_offset = CalculateBaseOffset(cro);
            UnloadObjectInfoPatches(cro, unload, base_offset);
        }
        base = cro->next_cro;
    }

    UnrebaseCRO(unload, address);
    unload->fixed_size = 0;

    UnloadExports(address);

    Kernel::g_current_process->vm_manager.UnmapRange(address, size);

    // TODO(Subv): Unload symbols and unmap memory
    return RESULT_SUCCESS;
}

static void UnloadCRO(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();
    u32 address = cmd_buff[1];
    ResultCode res = UnloadCRO(address);
    cmd_buff[1] = res.raw;
    // Clear the instruction cache
    Core::g_app_core->ClearInstructionCache();
    LOG_WARNING(Service_LDR, "Unloading CRO address=%08X res=%08X", address, res);
}

const Interface::FunctionInfo FunctionTable[] = {
    {0x000100C2, Initialize,            "Initialize"},
    {0x00020082, LoadCRR,               "LoadCRR"},
    {0x00030042, nullptr,               "UnloadCCR"},
    {0x000402C2, LoadExeCRO,            "LoadExeCRO"},
    {0x000500C2, UnloadCRO,             "UnloadCRO"},
    {0x00060042, nullptr,               "LinkCRO"},
    {0x00070042, nullptr,               "UnlinkCRO"},
    {0x00080042, nullptr,               "Shutdown"},
    {0x000902C2, nullptr,               "LoadExeCRO_New?"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface class

Interface::Interface() {
    Register(FunctionTable);
}

} // namespace
