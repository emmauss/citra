// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/assert.h"
#include "core/arm/cache/cache.h"

namespace Cache {

static std::list<CacheBase*> caches;

void RegisterCode(u32 address, u32 size) {
    for (auto const& cache : caches) {
        cache->OnCodeLoad(address, size);
    }
}

void UnregisterCode(u32 address, u32 size) {
    for (auto const& cache : caches) {
        cache->OnCodeUnload(address, size);
    }
}

void ClearCache() {
    for (auto const& cache : caches) {
        cache->Clear();
    }
}

static void RegisterCache(CacheBase* cache) {
    caches.push_back(cache);
}

static void UnregisterCache(CacheBase* cache) {
    caches.erase(std::remove(caches.begin(), caches.end(), cache), caches.end());
}

CacheBase::CacheBase(bool index_mode, OnClearCb clearcb) : index_mode(index_mode) {
    page_pointers.fill(nullptr);
    Clear();
    SetClearCallback(clearcb);
    RegisterCache(this);
}

CacheBase::~CacheBase() {
    UnregisterCache(this);
}

void CacheBase::Clear() {
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

bool CacheBase::RemoveBlock(u32 pc) {
    void** ptr = page_pointers[pc >> Memory::PAGE_BITS];
    if (ptr != nullptr) {
        ptr = &ptr[pc & Memory::PAGE_MASK];

        if (*ptr == nullptr) {
            return false;
        }

        if (index_mode) {
            const u32 id = pointer_to_id(*ptr);
            ASSERT(blocks_pc[id] == pc);

            blocks_pc[id] = INVALID_BLOCK;
            if (id < next_block) {
                next_block = id;
            }
            while (num_blocks > 0 && blocks_pc[num_blocks - 1] == INVALID_BLOCK) {
                --num_blocks;
            }
        }
        *ptr = nullptr;
        return true;
    }
    return false;
}

bool CacheBase::RemoveRange(u32 start, u32 end) {
    bool result = false;
    for (auto& cache : ptr_caches) {
        for (u32 i = std::max(start, cache.addr); i < std::min(end, cache.addr_end); ++i) {
            void** ptr = &cache.data[i - cache.addr];

            if (*ptr == nullptr) {
                continue;
            }

            if (index_mode) {
                const u32 id = pointer_to_id(*ptr);
                ASSERT(blocks_pc[id] == i);

                blocks_pc[id] = INVALID_BLOCK;
                if (id < next_block) {
                    next_block = id;
                }
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

void CacheBase::OnCodeLoad(u32 address, u32 size) {
    const u32 end = address + size;

    // Check there is no overlapping
    for (auto const& cache : ptr_caches) {
        ASSERT((address >= cache.addr_end) || (end <= cache.addr));
    }

    ASSERT((address & Memory::PAGE_MASK) == 0 && (size & Memory::PAGE_MASK) == 0);

    BlockPtrCache cache{ address, address + size };
    cache.data.assign(size, nullptr);

    for (u32 i = address; i < end; i += Memory::PAGE_SIZE) {
        page_pointers[i >> Memory::PAGE_BITS] = &cache.data[i - address];
    }
    ptr_caches.emplace_back(std::move(cache));
}

void CacheBase::OnCodeUnload(u32 address, u32 size) {
    ptr_caches.erase(std::remove_if(ptr_caches.begin(), ptr_caches.end(),
        [&, this](auto const& cache) {
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

void*& CacheBase::GetNewPtr(u32 pc) {
    DEBUG_ASSERT(!index_mode || next_block == MAX_BLOCKS || ((next_block < MAX_BLOCKS) && blocks_pc[next_block] == INVALID_BLOCK));
    DEBUG_ASSERT(GetPtr(pc) == nullptr);

    void** page_ptr = page_pointers[pc >> Memory::PAGE_BITS];
    if (page_ptr == nullptr) {
        // pc isnt within mapped code
        OnCodeLoad(pc & ~Memory::PAGE_MASK, Memory::PAGE_SIZE);
        page_ptr = page_pointers[pc >> Memory::PAGE_BITS];
    }

    void** block_ptr = &page_ptr[pc & Memory::PAGE_MASK];

    DEBUG_ASSERT(*block_ptr == nullptr);

    if (index_mode) {
        if (next_block == MAX_BLOCKS) {
            Clear();
        }

        blocks_pc[next_block] = pc;
        *block_ptr = id_to_pointer(next_block);

        do {
            ++next_block;
        } while (next_block <= num_blocks && blocks_pc[next_block] != INVALID_BLOCK);

        if (next_block > num_blocks) {
            num_blocks++;
        }
    }

    return *block_ptr;
}

}
