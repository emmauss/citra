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

/// Loaders call these when mapping/unmapping code
void RegisterCode(u32 address, u32 size);
void UnregisterCode(u32 address, u32 size = 1);

/// Clear every cache
void ClearCache();


using OnClearCb = std::function<void()>;

constexpr u32 MAX_BLOCKS = 0x40000;
constexpr u32 INVALID_BLOCK = 0xFFFFFFFF;

template <typename T>
struct BlockPtrCache {
    u32 addr;
    u32 addr_end;
    std::vector<T> data;
};

class CacheBase {
public:
    struct functions {
        std::function<void(u32, u32)> code_load;
        std::function<void(u32, u32)> code_unload;
        std::function<void()> clear;
    } const cache_fns;

protected:
    explicit CacheBase(functions fns);
    ~CacheBase();
};

template <typename T>
class CacheCommon : CacheBase {
protected:
    explicit CacheCommon(bool index_mode, OnClearCb clearcb) : index_mode(index_mode), CacheBase({
            [this](u32 start, u32 size) { this->OnCodeLoad(start, size); },
            [this](u32 start, u32 size) { this->OnCodeUnload(start, size); },
            [this]() { this->Clear(); }}) {
        static_assert(std::is_pointer<T>::value, "T must be a pointer");
        Clear();
        SetClearCallback(clearcb);
    }
    ~CacheCommon() {}

public:
    /// Called when the cache needs to reset or Clear() is called
    void SetClearCallback(OnClearCb cb) {
        OnClearCallback = cb;
    }

    /// Clear and call clear callback
    void Clear() {
        if (OnClearCallback != nullptr) {
            OnClearCallback();
        }

        for (auto& cache : ptr_caches) {
            cache.data.assign(cache.data.size(), nullptr);
        }

        if (index_mode) {
            blocks_pc.assign(MAX_BLOCKS, INVALID_BLOCK);
            next_block = num_blocks = 0;
        }
    }

    /// Returns true if block was found, false otherwise
    bool RemoveBlock(u32 pc) {
        T* ptr = page_pointers[pc >> Memory::PAGE_BITS];
        if (ptr != nullptr) {
            ptr = &ptr[pc & Memory::PAGE_MASK];

            if (*ptr == nullptr) {
                return false;
            }

            if (index_mode) {
                const u32 id = pointer_to_index(*ptr);
                ASSERT(blocks_pc[id] == pc);

                blocks_pc[id] = INVALID_BLOCK;
                next_block = std::min(id, next_block);

                while (num_blocks > 0 && blocks_pc[num_blocks - 1] == INVALID_BLOCK) {
                    --num_blocks;
                }
            }
            *ptr = nullptr;
            return true;
        }
        return false;
    }

    /// Returns true if at least one block was found, false otherwise
    bool RemoveRange(u32 start, u32 end) {
        bool result = false;
        for (auto& cache : ptr_caches) {
            for (u32 i = std::max(start, cache.addr); i < std::min(end, cache.addr_end); ++i) {
                T* ptr = &cache.data[i - cache.addr];

                if (*ptr == nullptr) {
                    continue;
                }

                if (index_mode) {
                    const u32 id = pointer_to_index(*ptr);
                    ASSERT(blocks_pc[id] == i);

                    blocks_pc[id] = INVALID_BLOCK;
                    next_block = std::min(id, next_block);

                    while (num_blocks > 0 && blocks_pc[num_blocks - 1] == INVALID_BLOCK) {
                        --num_blocks;
                    }
                }
                *ptr = nullptr;
                result = true;
            }
        }
        return result;
    }

    void OnCodeLoad(u32 address, u32 size) {
        // Check there is no overlapping
        ASSERT(std::none_of(ptr_caches.cbegin(), ptr_caches.cend(),
            [&](const auto& cache) {
            return (address < cache.addr_end) && (address + size > cache.addr);
        }));

        ASSERT((address & Memory::PAGE_MASK) == 0 && (size & Memory::PAGE_MASK) == 0);

        BlockPtrCache<T> cache{ address, address + size };
        cache.data.assign(size, nullptr);

        for (u32 i = address; i < address + size; i += Memory::PAGE_SIZE) {
            page_pointers[i >> Memory::PAGE_BITS] = &cache.data[i - address];
        }
        ptr_caches.emplace_back(std::move(cache));
    }

    void OnCodeUnload(u32 address, u32 size) {
        ptr_caches.erase(std::remove_if(ptr_caches.begin(), ptr_caches.end(),
            [&, this](const auto& cache) {
                if ((address < cache.addr_end) && (address + size > cache.addr)) {
                    this->RemoveRange(cache.addr, cache.addr_end);
                    for (u32 i = cache.addr; i < cache.addr_end; i += Memory::PAGE_SIZE) {
                        page_pointers[i >> Memory::PAGE_BITS] = nullptr;
                    }
                    return true;
                }
                return false;
            }),
            ptr_caches.cend());
    }

protected:
    T GetPtr(u32 pc) const {
        T* ptr = page_pointers[pc >> Memory::PAGE_BITS];
        if (ptr != nullptr) {
            const T value = ptr[pc & Memory::PAGE_MASK];
            DEBUG_ASSERT(!index_mode || value == nullptr || blocks_pc[pointer_to_index(ptr[pc & Memory::PAGE_MASK])] == pc);
            return value;
        }
        return nullptr;
    }

    T* GetNewPtr(u32 pc) {
        DEBUG_ASSERT(!index_mode || next_block == MAX_BLOCKS || ((next_block < MAX_BLOCKS) && blocks_pc[next_block] == INVALID_BLOCK));
        DEBUG_ASSERT(GetPtr(pc) == nullptr);

        T* page_ptr = page_pointers[pc >> Memory::PAGE_BITS];
        if (page_ptr == nullptr) {
            // pc isnt within mapped code
            OnCodeLoad(pc & ~Memory::PAGE_MASK, Memory::PAGE_SIZE);
            page_ptr = page_pointers[pc >> Memory::PAGE_BITS];
        }

        T* block_ptr = &page_ptr[pc & Memory::PAGE_MASK];

        DEBUG_ASSERT(!index_mode || *block_ptr == nullptr);

        if (index_mode) {
            if (next_block == MAX_BLOCKS) {
                Clear();
            }

            blocks_pc[next_block] = pc;
            *block_ptr = index_to_pointer(next_block);

            do {
                ++next_block;
            } while (next_block <= num_blocks && blocks_pc[next_block] != INVALID_BLOCK);

            if (next_block > num_blocks) {
                num_blocks++;
            }
        }

        return block_ptr;
    }

    std::function<T(u32)> index_to_pointer;
    std::function<u32(T)> pointer_to_index;

private:
    OnClearCb OnClearCallback = nullptr;
    const bool index_mode;

    std::vector<BlockPtrCache<T>> ptr_caches;
    std::array<T*, (1 << (32 - Memory::PAGE_BITS))> page_pointers = {};

    std::vector<u32> blocks_pc;
    u32 next_block = 0;
    u32 num_blocks = 0;
};

/// Use this if you only need to store a pointer
template <typename T>
class PtrCache final : public CacheCommon<T> {
public:
    explicit PtrCache(OnClearCb clearcb = nullptr) : CacheCommon<T>(false, clearcb) {
        static_assert(std::is_pointer<T>::value, "T must be a pointer, use IndexedCache");
    }
    ~PtrCache() {}

    /// Get cached pointer for PC
    T GetPtr(u32 pc) {
        return CacheCommon<T>::GetPtr(pc);
    }

    /// Set pointer for PC to value
    T SetPtr(u32 pc, T value) {
        return (*CacheCommon<T>::GetNewPtr(pc) = value);
    }
};

/// Index based cache
template <typename T>
class IndexedCache final : public CacheCommon<T*> {
public:
    explicit IndexedCache(OnClearCb clearcb = nullptr) : CacheCommon<T*>(true, clearcb) {

        static_assert(!std::is_pointer<T>::value, "T can't be a pointer, use PtrCache");

        CacheCommon<T*>::index_to_pointer = [this](u32 id) -> T* {
            return &blocks[id];
        };
        CacheCommon<T*>::pointer_to_index = [this](T* ptr) -> u32 {
            return static_cast<u32>(std::distance(blocks.cbegin(),
                std::find_if(blocks.cbegin(), blocks.cend(), [&](const T& block) {
                return (ptr == &block);
            })));
        };
    }
    ~IndexedCache() {}

    /// Get block cached for PC
    T* GetBlock(u32 pc) {
        return CacheCommon<T*>::GetPtr(pc);
    }

    /// Allocate block for PC
    T* GetNewBlock(u32 pc) {
        return *CacheCommon<T*>::GetNewPtr(pc);
    }

private:
    std::array<T, MAX_BLOCKS> blocks;
};

}
