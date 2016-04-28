// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/arm/cache/cache.h"

namespace Cache {

static std::list<CacheBase*> caches;

void RegisterCode(u32 address, u32 size) {
    for (const auto& cache : caches) {
        cache->cache_fns.code_load(address, size);
    }
}

void UnregisterCode(u32 address, u32 size) {
    for (const auto& cache : caches) {
        cache->cache_fns.code_unload(address, size);
    }
}

void ClearCache() {
    for (const auto& cache : caches) {
        cache->cache_fns.clear();
    }
}

static void RegisterCache(CacheBase* cache) {
    caches.push_back(cache);
}

static void UnregisterCache(CacheBase* cache) {
    caches.erase(std::remove(caches.begin(), caches.end(), cache), caches.end());
}

CacheBase::CacheBase(functions fns) : cache_fns(fns) {
    RegisterCache(this);
}

CacheBase::~CacheBase() {
    UnregisterCache(this);
}

}
