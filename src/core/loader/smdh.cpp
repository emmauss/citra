// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <cstring>
#include <vector>

#include "common/common_types.h"

#include "core/loader/loader.h"
#include "core/loader/smdh.h"

#include "video_core/utils.h"

namespace Loader {

bool IsValidSMDH(const std::vector<u8>& smdh_data) {
    if (smdh_data.size() < sizeof(Loader::SMDH))
        return false;

    u32 magic;
    memcpy(&magic, smdh_data.data(), 4);

    return Loader::MakeMagic('S', 'M', 'D', 'H') == magic;
}

std::vector<u16> SMDH::GetIcon(bool large) const {
    u32 size;
    const u16* icon_data;

    if (large) {
        size = 48;
        icon_data = reinterpret_cast<const u16*>(large_icon.data());
    } else {
        size = 24;
        icon_data = reinterpret_cast<const u16*>(small_icon.data());
    }

    std::vector<u16> icon(size * size);
    for (u32 x = 0; x < size; ++x) {
        for (u32 y = 0; y < size; ++y) {
            u32 coarse_y = y & ~7;
            icon[x + size * y] = *(icon_data + VideoCore::GetMortonOffset(x, y, 1) + coarse_y * size);
        }
    }
    return icon;
}

std::array<u16, 0x40> SMDH::GetShortTitle(Loader::SMDH::TitleLanguage language) const {
    return titles[static_cast<int>(language)].short_title;
}

} // namespace
