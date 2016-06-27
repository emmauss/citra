// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/alignment.h"
#include "common/common_types.h"
#include "common/logging/log.h"

#include "core/arm/arm_interface.h"
#include "core/hle/kernel/process.h"
#include "core/hle/kernel/vm_manager.h"
#include "core/hle/service/ldr_ro.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace LDR_RO

namespace LDR_RO {

static VAddr loaded_crs; ///< the virtual address of the static module

class CROHelper {
    const VAddr address; ///< the virtual address of this module

    enum class SegmentType : u32 {
        Text = 0,
        ROData = 1,
        Data = 2,
        BSS = 3,
    };

    struct SegmentEntry {
        u32 offset;
        u32 size;
        SegmentType type;
    };

    struct ExportNamedSymbollEntry {
        u32 name_offset; // pointing to a substring in ExportStrings
        u32 segment_tag;
    };

    struct ExportIndexedSymbolEntry {
        u32 segment_tag;
    };

    struct ExportTreeEntry {
        u16 segment_13_3;
        u16 end_1_next_15; // {end:1, next:15}
        u16 next_level;
        u16 export_table_id;
    };

    struct ImportModuleEntry {
        u32 name_offset; // pointing to a substring in ImporStrings
        u32 import_indexed_symbol_table_offset; // pointing to a subtable of ImportIndexedSymbolTable
        u32 import_indexed_symbol_num;
        u32 import_anonymous_symbol_table_offset; // pointing to a subtable of ImportAnonymousSymbolTable
        u32 import_anonymous_symbol_num;
    };

    struct PatchEntry { // for ExternalPatchTable and StaticPatchTable
        u32 segment_tag; // to self's segment in ExternalPatchTable. to static module segment in StaticPatchTable?
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

    struct ImportNamedSymbolEntry {
        u32 name_offset; // pointing to a substring in ImportStrings
        u32 patch_batch_offset; // pointing to a batch in ExternalPatchTable
    };

    struct ImportIndexedSymbolEntry {
        u32 index; // index in opponent's ExportIndexedSymbolEntry
        u32 patch_batch_offset; // pointing to a batch in ExternalPatchTable
    };

    struct ImportAnonymousSymbolEntry {
        u32 segment_tag; // to the opponent's segment
        u32 patch_batch_offset; // pointing to a batch in ExternalPatchTable
    };

    struct StaticAnonymousSymbolEntry {
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
        Unk_segment_tag,
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

        ExportNamedSymbolTableOffset,
        ExportNamedSymbolNum,
        ExportIndexedSymbolTableOffset,
        ExportIndexedSymbolNum,
        ExportStringsOffset,
        ExportStringsSize,
        ExportTreeTableOffset,
        ExportTreeNum,

        ImportModuleTableOffset,
        ImportModuleNum,
        ExternalPatchTableOffset,
        ExternalPatchNum,
        ImportNamedSymbolTableOffset,
        ImportNamedSymbolNum,
        ImportIndexedSymbolTableOffset,
        ImportIndexedSymbolNum,
        ImportAnonymousSymbolTableOffset,
        ImportAnonymousSymbolNum,
        ImportStringsOffset,
        ImportStringsSize,

        StaticAnonymousSymbolTableOffset,
        StaticAnonymousSymbolNum,
        InternalPatchTableOffset,
        InternalPatchNum,
        StaticPatchTableOffset,
        StaticPatchNum,
        Fix0Barrier,

        Fix3Barrier = ExportNamedSymbolTableOffset,
        Fix2Barrier = ImportModuleTableOffset,
        Fix1Barrier = StaticAnonymousSymbolTableOffset,
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

    /**
     * Interates over all registered auto-link modules, including the static module
     * and do some operation.
     * @param func a function object to operate on a module. It accepts one parameter
     *        CROHelper and returns ResultVal<bool>. It should return true if continues interation,
     *        false if stop interation, or an error code if on error.
     * @returns ResultCode indicating the result of the operation, 0 if all iteration success,
     *         otherwise error code of the last iteration.
     */
    template <typename T> // [](CROHelper cro)->ResultVal<bool>
    static ResultCode ForEachAutoLinkCRO(T func) {
        VAddr current = loaded_crs;
        while (current) {
            CROHelper cro(current);
            bool next;
            CASCADE_RESULT(next, func(cro));
            if (!next)
                break;
            current = cro.Next();
        }
        return RESULT_SUCCESS;
    }

    /**
     * Read an entry in one of module tables
     * @param field one of the ***TableOffset field indicating which table to look up// TODO how to document a template param?
     * @param T the entry type. Must match the entry type in the specified table.
     * @param index the index of the entry
     * @param data where to put the read entry.
     */
    template <HeaderField field, typename T>
    void GetEntry(int index, T& data) {
        static_assert(std::is_pod<T>::value, "The entry type must be POD!");
        Memory::ReadBlock(GetField(field) + index * sizeof(T), &data, sizeof(T));
    }

    /**
     * Writes an entry in one of module tables
     * @param field one of the ***TableOffset field indicating which table to look up// TODO how to document a template param?
     * @param T the entry type. Must match the entry type in the specified table.
     * @param index the index of the entry
     * @param data the entry data to write
     */
    template <HeaderField field, typename T>
    void SetEntry(int index, const T& data) {
        static_assert(std::is_pod<T>::value, "The entry type must be POD!");
        Memory::WriteBlock(GetField(field) + index * sizeof(T), &data, sizeof(T));
    }

    /**
     * Decodes a segment tag into segment index and offset.
     * @param segment_tag the segment tag to decode
     * @returns a tuple of (index, offset)
     */
    static std::tuple<u32, u32> DecodeSegmentTag(u32 segment_tag) {
        return std::make_tuple(segment_tag & 0xF, segment_tag >> 4);
    }

    /// Convert a segment tag to virtual address in this module. returns 0 if invalid
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

    /**
     * Find a exported named symbol in this module.
     * @param name the name of the symbol to find
     * @return VAddr the virtual address of the symbol. 0 if not found
     */
    VAddr FindExportNamedSymbol(VAddr name) {
        return FindExportNamedSymbol(Memory::GetString(name));
    }

    VAddr FindExportNamedSymbol(const std::string& name) {
        // TODO rewrite this!
        u32 symbol_export_num = GetField(ExportNamedSymbolNum);
        for (u32 i = 0; i < symbol_export_num; ++i) {
            ExportNamedSymbollEntry entry;
            GetEntry<ExportNamedSymbolTableOffset>(i, entry);
            if (name == Memory::GetString(entry.name_offset))
                return SegmentTagToAddress(entry.segment_tag);
        }
        return 0;
    }

    /// Rebases offsets in module header according to module address
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

    /**
     * Rebases offsets in segment table according to module address.
     * @param cro_size the size of the CRO file
     * @param data_segment_address buffer address for .data segment
     * @param data_segment_size the buffer size for .data segment
     * @param bss_segment_address the buffer address for .bss segment
     * @param bss_segment_size the buffer size for .bss segment
     * @param prev_data_segment the address of .data segment before rebasing
     * @returns ResultCode indicating the result of the operation, 0 on success
     */
    ResultCode RebaseSegmentTable(u32 cro_size,
        VAddr data_segment_address, u32 data_segment_size,
        VAddr bss_segment_address, u32 bss_segment_size, u32& prev_data_segment) {
        prev_data_segment = 0;
        u32 segment_num = GetField(SegmentNum);
        for (u32 i = 0; i < segment_num; ++i) {
            SegmentEntry segment;
            GetEntry<SegmentTableOffset>(i, segment);
            if (segment.type == SegmentType::Data) {
                if (segment.size) {
                    if (segment.size > data_segment_size)
                        return ResultCode(0xE0E12C1F);
                    prev_data_segment = segment.offset;
                    segment.offset = data_segment_address;
                }
            } else if (segment.type == SegmentType::BSS) {
                if (segment.size) {
                    if (segment.size > bss_segment_size)
                        return ResultCode(0xE0E12C1F);
                    segment.offset = bss_segment_address;
                }
            } else if (segment.offset) {
                segment.offset += address;
                if (segment.offset > address + cro_size)
                    return ResultCode(0xD9012C19);
            }
            SetEntry<SegmentTableOffset>(i, segment);
        }
        return RESULT_SUCCESS;
    }

    /// Rebases offsets in exported named symbol table according to module address
    ResultCode RebaseExportNamedSymbolTable() {
        VAddr export_strings_offset = GetField(ExportStringsOffset);
        VAddr export_strings_end = export_strings_offset + GetField(ExportStringsSize);

        u32 symbol_export_num = GetField(ExportNamedSymbolNum);
        for (u32 i = 0; i < symbol_export_num; ++i) {
            ExportNamedSymbollEntry entry;
            GetEntry<ExportNamedSymbolTableOffset>(i, entry);
            if (entry.name_offset) {
                entry.name_offset += address;
                if (entry.name_offset < export_strings_offset
                    || entry.name_offset >= export_strings_end) {
                    return ResultCode(0xD9012C11);
                }
            }
            SetEntry<ExportNamedSymbolTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    /// Rebases offsets in exported module table according to module address
    ResultCode RebaseImportModuleTable() {
        VAddr import_strings_offset = GetField(ImportStringsOffset);
        VAddr import_strings_end = import_strings_offset + GetField(ImportStringsSize);
        VAddr import_indexed_symbol_table_offset = GetField(ImportIndexedSymbolTableOffset);
        VAddr index_import_table_end = import_indexed_symbol_table_offset + GetField(ImportIndexedSymbolNum) * sizeof(ImportIndexedSymbolEntry);
        VAddr import_anonymous_symbol_table_offset = GetField(ImportAnonymousSymbolTableOffset);
        VAddr offset_import_table_end = import_anonymous_symbol_table_offset + GetField(ImportAnonymousSymbolNum) * sizeof(ImportAnonymousSymbolEntry);

        u32 object_num = GetField(ImportModuleNum);
        for (u32 i = 0; i < object_num; ++i) {
            ImportModuleEntry entry;
            GetEntry<ImportModuleTableOffset>(i, entry);
            if (entry.name_offset) {
                entry.name_offset += address;
                if (entry.name_offset < import_strings_offset
                    || entry.name_offset >= import_strings_end) {
                    return ResultCode(0xD9012C18);
                }
            }
            if (entry.import_indexed_symbol_table_offset) {
                entry.import_indexed_symbol_table_offset += address;
                if (entry.import_indexed_symbol_table_offset < import_indexed_symbol_table_offset
                    || entry.import_indexed_symbol_table_offset > index_import_table_end) {
                    return ResultCode(0xD9012C18);
                }
            }
            if (entry.import_anonymous_symbol_table_offset) {
                entry.import_anonymous_symbol_table_offset += address;
                if (entry.import_anonymous_symbol_table_offset < import_anonymous_symbol_table_offset
                    || entry.import_anonymous_symbol_table_offset > offset_import_table_end) {
                    return ResultCode(0xD9012C18);
                }
            }
            SetEntry<ImportModuleTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    /// Rebases offsets in imported named symbol table according to module address
    ResultCode RebaseImportNamedSymbolTable() {
        VAddr import_strings_offset = GetField(ImportStringsOffset);
        VAddr import_strings_end = import_strings_offset + GetField(ImportStringsSize);
        VAddr external_patch_table_offset = GetField(ExternalPatchTableOffset);
        VAddr external_patch_table_end = external_patch_table_offset + GetField(ExternalPatchNum) * sizeof(PatchEntry);

        u32 num = GetField(ImportNamedSymbolNum);
        for (u32 i = 0; i < num ; ++i) {
            ImportNamedSymbolEntry entry;
            GetEntry<ImportNamedSymbolTableOffset>(i, entry);
            if (entry.name_offset) {
                entry.name_offset += address;
                if (entry.name_offset < import_strings_offset
                    || entry.name_offset >= import_strings_end) {
                    return ResultCode(0xD9012C1B);
                }
            }
            if (entry.patch_batch_offset) {
                entry.patch_batch_offset += address;
                if (entry.patch_batch_offset < external_patch_table_offset
                    || entry.patch_batch_offset > external_patch_table_end) {
                    return ResultCode(0xD9012C1B);
                }
            }
            SetEntry<ImportNamedSymbolTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    /// Rebases offsets in imported indexed symbol table according to module address
    ResultCode RebaseImportIndexedSymbolTable() {
        VAddr external_patch_table_offset = GetField(ExternalPatchTableOffset);
        VAddr external_patch_table_end = external_patch_table_offset + GetField(ExternalPatchNum) * sizeof(PatchEntry);

        u32 num = GetField(ImportIndexedSymbolNum);
        for (u32 i = 0; i < num ; ++i) {
            ImportIndexedSymbolEntry entry;
            GetEntry<ImportIndexedSymbolTableOffset>(i, entry);
            if (entry.patch_batch_offset) {
                entry.patch_batch_offset += address;
                if (entry.patch_batch_offset < external_patch_table_offset
                    || entry.patch_batch_offset > external_patch_table_end) {
                    return ResultCode(0xD9012C14);
                }
            }
            SetEntry<ImportIndexedSymbolTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    /// Rebases offsets in imported anonymous symbol table according to module address
    ResultCode RebaseImportAnonymousSymbolTable() {
        VAddr external_patch_table_offset = GetField(ExternalPatchTableOffset);
        VAddr external_patch_table_end = external_patch_table_offset + GetField(ExternalPatchNum) * sizeof(PatchEntry);

        u32 num = GetField(ImportAnonymousSymbolNum);
        for (u32 i = 0; i < num ; ++i) {
            ImportAnonymousSymbolEntry entry;
            GetEntry<ImportAnonymousSymbolTableOffset>(i, entry);
            if (entry.patch_batch_offset) {
                entry.patch_batch_offset += address;
                if (entry.patch_batch_offset < external_patch_table_offset
                    || entry.patch_batch_offset > external_patch_table_end) {
                    return ResultCode(0xD9012C17);
                }
            }
            SetEntry<ImportAnonymousSymbolTableOffset>(i, entry);
        }
        return RESULT_SUCCESS;
    }

    /**
     * Applies a patch
     * @param target_address where to apply the patch
     * @param patch_type the type of the patch
     * @param x // TODO
     * @param value the value to be patched
     * @param addressB the future address of the target.
     *        Usually equals to target_address, but will be different for a target in .data segment
     * @returns ResultCode indicating the result of the operation, 0 on success
     */
    ResultCode ApplyPatch(VAddr target_address, u8 patch_type, u32 x, u32 value, u32 addressB) {
        switch (patch_type) {
            case 2:
                Memory::Write32(target_address, value + x); // writes an obsolute address value
                break;
            case 3:
                Memory::Write32(target_address, value + x - addressB); // writes an relative address value
                break;
            // TODO implement more types
            default:
                return ResultCode(0xD9012C23);
        }
        return RESULT_SUCCESS;
    }

    /// Resets all external patches to unresolved state.
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

    /**
     * Applies or resets a batch of patch
     * @param batch the virtual address of the first patch in the batch
     * @param patch_value the value to be patched
     * @param reset false to set the batch to resolved state, true to set the batch to unresolved state
     * @returns ResultCode indicating the result of the operation, 0 on success
     */
    ResultCode ApplyPatchBatch(VAddr batch, u32 patch_value, bool reset = false) {
        if (patch_value == 0 && !reset)
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
        patch.batch_resolved = reset ? 0 : 1;
        Memory::WriteBlock(batch, &patch, sizeof(PatchEntry));
    }

    /// Applies all static anonymous symbol to the static module // TODO ???
    ResultCode ApplyStaticAnonymousSymbolToCRS() {
        VAddr static_patch_table_offset = GetField(StaticPatchTableOffset);
        VAddr static_patch_table_end = static_patch_table_offset + GetField(StaticPatchNum) * sizeof(PatchEntry);

        CROHelper crs(loaded_crs);
        u32 offset_export_num = GetField(StaticAnonymousSymbolNum);
        for (u32 i = 0; i < offset_export_num; ++i) {
            StaticAnonymousSymbolEntry entry;
            GetEntry<StaticAnonymousSymbolTableOffset>(i, entry);
            u32 batch_address = entry.patch_batch_offset + address;

            if (batch_address < static_patch_table_offset
                || batch_address > static_patch_table_end) {
                return ResultCode(0xD9012C16);
            }

            u32 patch_value = SegmentTagToAddress(entry.segment_tag);

            ResultCode result = crs.ApplyPatchBatch(batch_address, patch_value);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                return result;
            }
        }
        return RESULT_SUCCESS;
    }

    /// Applies all internal patches to the module itself
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
            if (target_segment.type == SegmentType::Data) {
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

    /// Unrebases offsets in imported anonymous symbol table
    void UnrebaseImportAnonymousSymbolTable() {
        u32 num = GetField(ImportAnonymousSymbolNum);
        for (u32 i = 0; i < num ; ++i) {
            ImportAnonymousSymbolEntry entry;
            GetEntry<ImportAnonymousSymbolTableOffset>(i, entry);
            if (entry.patch_batch_offset) {
                entry.patch_batch_offset -= address;
            }
            SetEntry<ImportAnonymousSymbolTableOffset>(i, entry);
        }
    }

    /// Unrebases offsets in imported indexed symbol table
    void UnrebaseImportIndexedSymbolTable() {
        u32 num = GetField(ImportIndexedSymbolNum);
        for (u32 i = 0; i < num ; ++i) {
            ImportIndexedSymbolEntry entry;
            GetEntry<ImportIndexedSymbolTableOffset>(i, entry);
            if (entry.patch_batch_offset) {
                entry.patch_batch_offset -= address;
            }
            SetEntry<ImportIndexedSymbolTableOffset>(i, entry);
        }
    }

    /// Unrebases offsets in imported named symbol table
    void UnrebaseImportNamedSymbolTable() {
        u32 num = GetField(ImportNamedSymbolNum);
        for (u32 i = 0; i < num ; ++i) {
            ImportNamedSymbolEntry entry;
            GetEntry<ImportNamedSymbolTableOffset>(i, entry);
            if (entry.name_offset) {
                entry.name_offset -= address;
            }
            if (entry.patch_batch_offset) {
                entry.patch_batch_offset -= address;
            }
            SetEntry<ImportNamedSymbolTableOffset>(i, entry);
        }
    }

    /// Unrebases offsets in imported module table
    void UnrebaseImportModuleTable() {
        u32 object_num = GetField(ImportModuleNum);
        for (u32 i = 0; i < object_num; ++i) {
            ImportModuleEntry entry;
            GetEntry<ImportModuleTableOffset>(i, entry);
            if (entry.name_offset) {
                entry.name_offset -= address;
            }
            if (entry.import_indexed_symbol_table_offset) {
                entry.import_indexed_symbol_table_offset -= address;
            }
            if (entry.import_anonymous_symbol_table_offset) {
                entry.import_anonymous_symbol_table_offset -= address;
            }
            SetEntry<ImportModuleTableOffset>(i, entry);
        }
    }

    /// Unrebases offsets in exported named symbol table
    void UnrebaseExportNamedSymbolTable() {
        u32 symbol_export_num = GetField(ExportNamedSymbolNum);
        for (u32 i = 0; i < symbol_export_num; ++i) {
            ExportNamedSymbollEntry entry;
            GetEntry<ExportNamedSymbolTableOffset>(i, entry);
            if (entry.name_offset) {
                entry.name_offset -= address;
            }
            SetEntry<ExportNamedSymbolTableOffset>(i, entry);
        }
    }

    /// Unrebases offsets in segment table
    void UnrebaseSegmentTable() {
        u32 segment_num = GetField(SegmentNum);
        for (u32 i = 0; i < segment_num; ++i) {
            SegmentEntry segment;
            GetEntry<SegmentTableOffset>(i, segment);
            if (segment.type == SegmentType::BSS) {
                segment.offset = 0;
            } else if (segment.offset) {
                segment.offset -= address;
            }
            SetEntry<SegmentTableOffset>(i, segment);
        }
    }

    /// Unrebases offsets in module header
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

    /// Looks up all imported named symbols of this module in all registered auto-link modules, and resolves them if found
    ResultCode ApplyImportNamedSymbol() {
        u32 symbol_import_num = GetField(ImportNamedSymbolNum);
        for (u32 i = 0; i < symbol_import_num; ++i) {
            ImportNamedSymbolEntry entry;
            GetEntry<ImportNamedSymbolTableOffset>(i, entry);
            VAddr patch_addr = entry.patch_batch_offset;
            PatchEntry patch_entry;
            Memory::ReadBlock(patch_addr, &patch_entry, sizeof(PatchEntry));

            if (!patch_entry.batch_resolved) {
                LOG_INFO(Service_LDR, "Try resolving \"%s\" in %s", (char*)Memory::GetPointer(entry.name_offset), ModuleName().data());
                ResultCode result = ForEachAutoLinkCRO([&](CROHelper source) -> ResultVal<bool> {
                    u32 value = source.FindExportNamedSymbol(entry.name_offset);
                    if (value) {
                        LOG_INFO(Service_LDR, "resolve from %s", source.ModuleName().data());
                        ResultCode result = ApplyPatchBatch(patch_addr, value);
                        if (result.IsError()) {
                            LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                            return result;
                        }
                        return MakeResult<bool>(false);
                    }
                    return MakeResult<bool>(true);
                });
                if (result.IsError()) {
                    return result;
                }
            }
        }
        return RESULT_SUCCESS;
    }

    /// Resets all imported named symbols of this module to unresolved state
    ResultCode ResetImportNamedSymbol() {
        u32 reset_value = SegmentTagToAddress(GetField(OnUnresolved_segment_tag));

        u32 symbol_import_num = GetField(ImportNamedSymbolNum);
        for (u32 i = 0; i < symbol_import_num; ++i) {
            ImportNamedSymbolEntry entry;
            GetEntry<ImportNamedSymbolTableOffset>(i, entry);
            VAddr patch_addr = entry.patch_batch_offset;
            PatchEntry patch_entry;
            Memory::ReadBlock(patch_addr, &patch_entry, sizeof(PatchEntry));

            LOG_INFO(Service_LDR, "Resetting \"%s\" in %s", (char*)Memory::GetPointer(entry.name_offset), ModuleName().data());
            ResultCode result = ApplyPatchBatch(patch_addr, reset_value, true);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error reseting patch batch %08X", result.raw);
                return result;
            }

        }
        return RESULT_SUCCESS;
    }

    /// Resets all imported indexed symbols of this module to unresolved state
    ResultCode ResetImportIndexedSymbol() {
        u32 reset_value = SegmentTagToAddress(GetField(OnUnresolved_segment_tag));

        u32 import_num = GetField(ImportIndexedSymbolNum);
        for (u32 i = 0; i < import_num; ++i) {
            ImportIndexedSymbolEntry entry;
            GetEntry<ImportIndexedSymbolTableOffset>(i, entry);
            VAddr patch_addr = entry.patch_batch_offset;
            PatchEntry patch_entry;
            Memory::ReadBlock(patch_addr, &patch_entry, sizeof(PatchEntry));

            ResultCode result = ApplyPatchBatch(patch_addr, reset_value, true);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error reseting patch batch %08X", result.raw);
                return result;
            }
        }
        return RESULT_SUCCESS;
    }

    /// Resets all imported anonymous symbols of this module to unresolved state
    ResultCode ResetImportAnonymousSymbol() {
        u32 reset_value = SegmentTagToAddress(GetField(OnUnresolved_segment_tag));

        u32 import_num = GetField(ImportAnonymousSymbolNum);
        for (u32 i = 0; i < import_num; ++i) {
            ImportAnonymousSymbolEntry entry;
            GetEntry<ImportAnonymousSymbolTableOffset>(i, entry);
            VAddr patch_addr = entry.patch_batch_offset;
            PatchEntry patch_entry;
            Memory::ReadBlock(patch_addr, &patch_entry, sizeof(PatchEntry));

            ResultCode result = ApplyPatchBatch(patch_addr, reset_value, true);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error reseting patch batch %08X", result.raw);
                return result;
            }
        }
        return RESULT_SUCCESS;
    }

    /// Finds registered auto-link modules that this module imports, and resolve indexed and anonymous symbols exported by them
    ResultCode ApplyModuleImport() {
        u32 import_module_num = GetField(ImportModuleNum);
        for (u32 i = 0; i < import_module_num; ++i) {
            ImportModuleEntry entry;
            GetEntry<ImportModuleTableOffset>(i, entry);
            std::string want_cro_name = Memory::GetString(entry.name_offset);

            ResultCode result = ForEachAutoLinkCRO([&](CROHelper source) -> ResultVal<bool> {
                if (want_cro_name == source.ModuleName()) {
                    LOG_INFO(Service_LDR, "Resolving symbols in %s from %s", ModuleName().data(), source.ModuleName().data());
                    for (u32 j = 0; j < entry.import_indexed_symbol_num; ++j) {
                        ImportIndexedSymbolEntry im;
                        Memory::ReadBlock(entry.import_indexed_symbol_table_offset + j * sizeof(ImportIndexedSymbolEntry), &im, sizeof(ImportIndexedSymbolEntry));
                        ExportIndexedSymbolEntry ex;
                        source.GetEntry<ExportIndexedSymbolTableOffset>(im.index, ex);
                        u32 patch_value = source.SegmentTagToAddress(ex.segment_tag);
                        ResultCode result = ApplyPatchBatch(im.patch_batch_offset, patch_value);
                        if (result.IsError()) {
                            LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                            return result;
                        }
                    }
                    for (u32 j = 0; j < entry.import_anonymous_symbol_num; ++j) {
                        ImportAnonymousSymbolEntry im;
                        Memory::ReadBlock(entry.import_anonymous_symbol_table_offset + j * sizeof(ImportAnonymousSymbolEntry), &im, sizeof(ImportIndexedSymbolEntry));
                        u32 patch_value = source.SegmentTagToAddress(im.segment_tag);
                        ResultCode result = ApplyPatchBatch(im.patch_batch_offset, patch_value);
                        if (result.IsError()) {
                            LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                            return result;
                        }
                    }
                    return MakeResult<bool>(false);
                }
                return MakeResult<bool>(true);
            });
            if (result.IsError()) {
                return result;
            }
        }
        return RESULT_SUCCESS;
    }

    /// Resolves target module's imported named symbols that exported by this module
    ResultCode ApplyExportNamedSymbol(CROHelper target) {
        LOG_INFO(Service_LDR, "Try resolving named symbol in %s from %s",
            target.ModuleName().data(), ModuleName().data());

        u32 target_symbol_import_num = target.GetField(ImportNamedSymbolNum);
        for (u32 i = 0; i < target_symbol_import_num; ++i) {
            ImportNamedSymbolEntry entry;
            target.GetEntry<ImportNamedSymbolTableOffset>(i, entry);
            VAddr patch_addr = entry.patch_batch_offset;
            PatchEntry patch_entry;
            Memory::ReadBlock(patch_addr, &patch_entry, sizeof(PatchEntry));

            if (!patch_entry.batch_resolved) {
                u32 patch_value = FindExportNamedSymbol(entry.name_offset);
                if (patch_value) {
                    LOG_INFO(Service_LDR, "Resolving symbol %s",
                        Memory::GetPointer(entry.name_offset));
                    ResultCode result = target.ApplyPatchBatch(patch_addr, patch_value);
                    if (result.IsError()) {
                        LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                        return result;
                    }
                }
            }
        }
        return RESULT_SUCCESS;
    }

    /// Reset target's named symbols imported from this module to unresolved state
    ResultCode ResetExportNamedSymbol(CROHelper target) {
        LOG_INFO(Service_LDR, "Try resetying named symbol in %s from %s",
            target.ModuleName().data(), ModuleName().data());
        u32 reset_value = target.SegmentTagToAddress(target.GetField(OnUnresolved_segment_tag));

        u32 target_symbol_import_num = target.GetField(ImportNamedSymbolNum);
        for (u32 i = 0; i < target_symbol_import_num; ++i) {
            ImportNamedSymbolEntry entry;
            target.GetEntry<ImportNamedSymbolTableOffset>(i, entry);
            VAddr patch_addr = entry.patch_batch_offset;
            PatchEntry patch_entry;
            Memory::ReadBlock(patch_addr, &patch_entry, sizeof(PatchEntry));

            if (!patch_entry.batch_resolved) {
                u32 patch_value = FindExportNamedSymbol(entry.name_offset);
                if (patch_value) {
                    patch_value = reset_value;
                    LOG_INFO(Service_LDR, "Resetting symbol %s",
                        Memory::GetPointer(entry.name_offset));
                    ResultCode result = target.ApplyPatchBatch(patch_addr, patch_value, true);
                    if (result.IsError()) {
                        LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                        return result;
                    }
                }
            }
        }
        return RESULT_SUCCESS;
    }

    /// Resolves imported indexed and anonymous symbols in the target module which imported this module
    ResultCode ApplyModuleExport(CROHelper target) {
        LOG_INFO(Service_LDR, "Try resolving symbol in %s from %s",
            target.ModuleName().data(), ModuleName().data());

        std::string module_name = ModuleName();
        u32 target_import_module_num = target.GetField(ImportModuleNum);
        for (u32 i = 0; i < target_import_module_num; ++i) {
            ImportModuleEntry entry;
            target.GetEntry<ImportModuleTableOffset>(i, entry);

            if (Memory::GetString(entry.name_offset) != module_name)
                continue;

            LOG_INFO(Service_LDR, "Resolving...");
            for (u32 j = 0; j < entry.import_indexed_symbol_num; ++j) {
                ImportIndexedSymbolEntry im;
                Memory::ReadBlock(entry.import_indexed_symbol_table_offset + j * sizeof(ImportIndexedSymbolEntry), &im, sizeof(ImportIndexedSymbolEntry));
                u32 patch_value;
                ExportIndexedSymbolEntry ex;
                GetEntry<ExportIndexedSymbolTableOffset>(im.index, ex);
                patch_value = SegmentTagToAddress(ex.segment_tag);
                ResultCode result = target.ApplyPatchBatch(im.patch_batch_offset, patch_value);
                if (result.IsError()) {
                    LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                    return result;
                }
            }
            for (u32 j = 0; j < entry.import_anonymous_symbol_num; ++j) {
                ImportAnonymousSymbolEntry im;
                Memory::ReadBlock(entry.import_anonymous_symbol_table_offset + j * sizeof(ImportAnonymousSymbolEntry), &im, sizeof(ImportIndexedSymbolEntry));
                u32 patch_value = SegmentTagToAddress(im.segment_tag);
                ResultCode result = target.ApplyPatchBatch(im.patch_batch_offset, patch_value);
                if (result.IsError()) {
                    LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                    return result;
                }
            }
        }

        return RESULT_SUCCESS;
    }

    /// Reset target's indexed and anonymous symbol imported from this module to unresolved state
    ResultCode ResetModuleExport(CROHelper target) {
        LOG_INFO(Service_LDR, "Try resetting symbol in %s from %s",
            target.ModuleName().data(), ModuleName().data());
        u32 reset_value = target.SegmentTagToAddress(target.GetField(OnUnresolved_segment_tag));

        std::string module_name = ModuleName();
        u32 target_import_module_num = target.GetField(ImportModuleNum);
        for (u32 i = 0; i < target_import_module_num; ++i) {
            ImportModuleEntry entry;
            target.GetEntry<ImportModuleTableOffset>(i, entry);

            if (Memory::GetString(entry.name_offset) != module_name)
                continue;

            LOG_INFO(Service_LDR, "Resetting...");
            for (u32 j = 0; j < entry.import_indexed_symbol_num; ++j) {
                ImportIndexedSymbolEntry im;
                Memory::ReadBlock(entry.import_indexed_symbol_table_offset + j * sizeof(ImportIndexedSymbolEntry), &im, sizeof(ImportIndexedSymbolEntry));
                u32 patch_value = reset_value;
                ResultCode result = target.ApplyPatchBatch(im.patch_batch_offset, patch_value, true);
                if (result.IsError()) {
                    LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                    return result;
                }
            }
            for (u32 j = 0; j < entry.import_anonymous_symbol_num; ++j) {
                ImportAnonymousSymbolEntry im;
                Memory::ReadBlock(entry.import_anonymous_symbol_table_offset + j * sizeof(ImportAnonymousSymbolEntry), &im, sizeof(ImportIndexedSymbolEntry));
                u32 patch_value =reset_value;
                ResultCode result = target.ApplyPatchBatch(im.patch_batch_offset, patch_value, true);
                if (result.IsError()) {
                    LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                    return result;
                }
            }
        }

        return RESULT_SUCCESS;
    }

    /// Resolve the exit function in this module
    ResultCode ApplyExitPatches() {
        u32 symbol_import_num = GetField(ImportNamedSymbolNum);
        for (u32 i = 0; i < symbol_import_num; ++i) {
            ImportNamedSymbolEntry entry;
            GetEntry<ImportNamedSymbolTableOffset>(i, entry);
            VAddr patch_addr = entry.patch_batch_offset;
            PatchEntry patch_entry;
            Memory::ReadBlock(patch_addr, &patch_entry, sizeof(PatchEntry));

            if (Memory::GetString(entry.name_offset) == "__aeabi_atexit"){
                // TODO verify this code
                ResultCode result = ForEachAutoLinkCRO([&](CROHelper source) -> ResultVal<bool> {
                    u32 value = source.FindExportNamedSymbol("nnroAeabiAtexit_");
                    if (value) {
                        LOG_INFO(Service_LDR, "resolve from %s", source.ModuleName().data());
                        ResultCode result = ApplyPatchBatch(patch_addr, value);
                        if (result.IsError()) {
                            LOG_ERROR(Service_LDR, "Error applying patch batch %08X", result.raw);
                            return result;
                        }
                        return MakeResult<bool>(false);
                    }
                    return MakeResult<bool>(true);
                });
                if (result.IsError()) {
                    LOG_ERROR(Service_LDR, "Error applying exit patch %08X", result.raw);
                    return result;
                }
            }
        }
        return RESULT_SUCCESS;
    }

public:
    CROHelper(VAddr cro_address) : address(cro_address) {
    }

    std::string ModuleName() {
        return Memory::GetString(GetField(ModuleNameOffset));
    }

    u32 GetFileSize() {
        return GetField(FileSize);
    }

    /// Rebases the module according to its address
    ResultCode Rebase(u32 cro_size, VAddr data_segment_addresss, u32 data_segment_size, VAddr bss_segment_address, u32 bss_segment_size, bool is_crs = false) {
        ResultCode result = RebaseHeader(cro_size);
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing header %08X", result.raw);
            return result;
        }

        LOG_INFO(Service_LDR, "Load CRO %s", Memory::GetPointer(GetField(ModuleNameOffset)));

        // TODO verify module name

        u32 prev_data_segment_address = 0;
        if (!is_crs) {
            result = RebaseSegmentTable(cro_size,
                data_segment_addresss, data_segment_size,
                bss_segment_address, bss_segment_size,
                prev_data_segment_address);
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error rebasing segment table %08X", result.raw);
                return result;
            }
        }
        prev_data_segment_address += address;

        result = RebaseExportNamedSymbolTable();
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

        result = RebaseImportNamedSymbolTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing symbol import table %08X", result.raw);
            return result;
        }

        result = RebaseImportIndexedSymbolTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing index import table %08X", result.raw);
            return result;
        }

        result = RebaseImportAnonymousSymbolTable();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error rebasing offset import table %08X", result.raw);
            return result;
        }

        // TODO verify import strings

        if (!is_crs) {
            result = ApplyStaticAnonymousSymbolToCRS();
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

        if (!is_crs) {
            result = ApplyExitPatches();
            if (result.IsError()) {
                LOG_ERROR(Service_LDR, "Error applying exit patches %08X", result.raw);
                return result;
            }
        }

        return RESULT_SUCCESS;
    }

    /// Unrebases the module
    void Unrebase(bool is_crs = false) {
        UnrebaseImportAnonymousSymbolTable();
        UnrebaseImportIndexedSymbolTable();
        UnrebaseImportNamedSymbolTable();
        UnrebaseImportModuleTable();
        UnrebaseExportNamedSymbolTable();

        if (!is_crs)
            UnrebaseSegmentTable();

        SetNext(0);
        SetPrevious(0);

        SetField(FixedSize, 0);

        UnrebaseHeader();
    }

    /// Verifies the module by CRR
    ResultCode Verify(u32 cro_size, VAddr crr) {
        // TODO
        return RESULT_SUCCESS;
    }

    /// Links this module with all registered auto-link module
    ResultCode Link() {

        // Imports named symbols from other modules
        ResultCode result = ApplyImportNamedSymbol();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error applying symbol import %08X", result.raw);
            return result;
        }

        // Imports indexed and anonymous symbols from other modules
        result =  ApplyModuleImport();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error applying module import %08X", result.raw);
            return result;
        }

        // Exports symbols to other modules
        result = ForEachAutoLinkCRO([this](CROHelper target) -> ResultVal<bool> {
            ResultCode result = ApplyExportNamedSymbol(target);
            if (result.IsError())
                return result;
            result = ApplyModuleExport(target);
            if (result.IsError())
                return result;
            return MakeResult<bool>(true);
        });
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error applying export %08X", result.raw);
            return result;
        }

        return RESULT_SUCCESS;
    }

    /// Unlinks this module with other modules
    ResultCode Unlink() {

        // Resets all imported named symbols
        ResultCode result = ResetImportNamedSymbol();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error resetting symbol import %08X", result.raw);
            return result;
        }

        // Resets all imported indexed symbols
        result = ResetImportIndexedSymbol();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error resetting indexed import %08X", result.raw);
            return result;
        }

        // Resets all imported anonymous symbols
        result = ResetImportAnonymousSymbol();
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error resetting anonymous import %08X", result.raw);
            return result;
        }

        // Resets all symbols in other modules imported from this module
        // Note: the RO service seems only searching in auto-link modules
        result = ForEachAutoLinkCRO([this](CROHelper target) -> ResultVal<bool> {
            ResultCode result = ResetExportNamedSymbol(target);
            if (result.IsError())
                return result;
            result = ResetModuleExport(target);
            if (result.IsError())
                return result;
            return MakeResult<bool>(true);
        });
        if (result.IsError()) {
            LOG_ERROR(Service_LDR, "Error resetting export %08X", result.raw);
            return result;
        }

        return RESULT_SUCCESS;
    }

    void RegisterCRS() {
        SetNext(0);
        SetPrevious(0);
    }

    /// Registers this module and adds to the module list
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

    /// Unregisters this module and removes from the module list
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
        // TODO THIS IS BROKEN!
        u32 fix_end = GetFixEnd(fix_level);


        if (fix_level) {
            SetField(Magic, 0x44584946); // FIXD

            for (int field = FIX_BARRIERS[fix_level]; field < Fix0Barrier; field += 2) {
                SetField(static_cast<HeaderField>(field), fix_end);
                SetField(static_cast<HeaderField>(field + 1), 0);
            }
        }


        fix_end = Common::AlignUp(fix_end, 0x1000);

        u32 fixed_size = fix_end - address;
        SetField(FixedSize, fixed_size);
        return /*fixed_size*/GetField(FileSize); // HACK
    }
};

const std::array<int, 17> CROHelper::ENTRY_SIZE {{
    1, // code
    1, // data
    1, // module name
    sizeof(SegmentEntry),
    sizeof(ExportNamedSymbollEntry),
    sizeof(ExportIndexedSymbolEntry),
    1, // export strings
    sizeof(ExportTreeEntry),
    sizeof(ImportModuleEntry),
    sizeof(PatchEntry),
    sizeof(ImportNamedSymbolEntry),
    sizeof(ImportIndexedSymbolEntry),
    sizeof(ImportAnonymousSymbolEntry),
    1, // import strings
    sizeof(StaticAnonymousSymbolEntry),
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
        UNREACHABLE();//Debug
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
        UNREACHABLE();//Debug
        return;
    }

    result = cro.Link();
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error linking CRO %08X", result.raw);
        // TODO Unmap memory?
        cmd_buff[1] = result.raw;
        UNREACHABLE();//Debug
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
 *      2 : zero?
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

    // TODO "Validate Something"
    // TODO loc_140033CC

    CROHelper cro(cro_address);

    cro.Unregister();

    ResultCode result = cro.Unlink();
    if (result.IsError()) {
        LOG_ERROR(Service_LDR, "Error unlinking CRO %08X", result.raw);
        cmd_buff[1] = result.raw;
        UNREACHABLE();//Debug
        return;
    }

    // TODO if fixed {...}

    cro.Unrebase();

    Kernel::g_current_process->vm_manager.UnmapRange(cro_address, cro.GetFileSize());

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
