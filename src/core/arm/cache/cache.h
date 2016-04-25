// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <list>
#include <vector>

#include "common/assert.h"
#include "common/common_types.h"

#include "core/memory.h"

namespace Cache {

using OnClearCb = std::function<void()>;

const u32 MAX_BLOCKS = 0x40000;
const u32 INVALID_BLOCK = 0xFFFFFFFF;

struct BlockPtrCache {
    u32 addr;
    u32 addr_end;
    std::vector<u8*> data;
};

class CacheBase {
protected:
    explicit CacheBase(bool index_mode, OnClearCb clearcb);
    ~CacheBase();

public:
    /// Called when the cache needs to reset or Clear() is called
    void SetClearCallback(OnClearCb cb) { OnClearCallback = cb; }

    /// Clear and call clear callback
    void Clear();
    // returns true if block was found, false otherwise
    bool RemoveBlock(u32 pc);
    bool RemoveRange(u32 start, u32 end);

    void OnCodeLoad(u32 address, u32 size);
    void OnCodeUnload(u32 address, u32 size);

protected:
    u8* GetPtr(u32 pc) const {
        u8** ptr = page_pointers[pc >> Memory::PAGE_BITS];
        if (ptr != nullptr) {
            DEBUG_ASSERT(!index_mode || blocks_pc[pointer_to_id(ptr[pc & Memory::PAGE_MASK])] == pc);
            return ptr[pc & Memory::PAGE_MASK];
        }
        return nullptr;
    }
    u8*& GetNewPtr(u32 pc);

    std::function<u8*(u32)> id_to_pointer;
    std::function<u32(u8*)> pointer_to_id;

private:
    bool index_mode;
    OnClearCb OnClearCallback = nullptr;

    std::vector<BlockPtrCache> ptr_caches;
    std::array<u8**, (1 << (32 - Memory::PAGE_BITS))> page_pointers;

    std::vector<u32> blocks_pc;
    u32 next_block = 0;
    u32 num_blocks = 0;
};

/// Use this if you only need to store a pointer
template <typename T>
class PtrCache final : public CacheBase {
public:
    explicit PtrCache(OnClearCb clearcb = nullptr) : CacheBase(false, clearcb) {
        static_assert(std::is_pointer<T>::value, "T must be a pointer");
    }
    ~PtrCache() {}

    /// Get cached pointer for PC
    T FindPtr(u32 pc) { return reinterpret_cast<T>(GetPtr(pc)); }

    /// Get reference of pointer for PC
    T& GetNewPtr(u32 pc) { return reinterpret_cast<T&>(CacheBase::GetNewPtr(pc)); }
};

/// Index based cache
template <typename T>
class Cache final : public CacheBase {
public:
    explicit Cache(OnClearCb clearcb = nullptr) : CacheBase(true, clearcb) {
        id_to_pointer = [this](u32 id) -> u8* {
            return reinterpret_cast<u8*>(&blocks[id]);
        };
        pointer_to_id = [this](u8* ptr) -> u32 {
            return static_cast<u32>(reinterpret_cast<T*>(ptr) - &blocks[0]);
        };
    }
    ~Cache() {}

    /// Get block cached for PC
    T* FindBlock(u32 pc) { return reinterpret_cast<T*>(GetPtr(pc)); }

    /// Allocate block for PC
    T& GetNewBlock(u32 pc) { return *reinterpret_cast<T*&>(GetNewPtr(pc)); }

private:
    std::array<T, MAX_BLOCKS> blocks;
};

class CacheManager {
public:
    CacheManager() {}
    ~CacheManager() {}

    /// Loaders call these when mapping/unmapping code
    void RegisterCode(u32 address, u32 size) const;
    void UnregisterCode(u32 address, u32 size = 1) const;

    /// Clear every cache
    void ClearCache() const;

private:
    std::list<CacheBase*> caches;

public:
    void RegisterCache(CacheBase* cache);
    void UnregisterCache(CacheBase* cache);
};

extern CacheManager g_cachemanager;

}
