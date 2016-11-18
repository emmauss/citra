

// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/string_util.h"
#include "core/hle/kernel/event.h"
#include "core/hle/service/frd/frd.h"
#include "core/hle/service/frd/frd_a.h"
#include "core/hle/service/frd/frd_u.h"
#include "core/hle/service/service.h"

namespace Service {
namespace FRD {

static Kernel::SharedPtr<Kernel::Event> event_notification;
static Kernel::SharedPtr<Kernel::Event> completion_event;
static FriendKey my_friend_key = {1, 2, 3ull};
static MyPresence my_presence = {};
static Profile my_profile = {1, 2, 3, 4, 5};
static MiiData my_mii = {};
static bool logged_in;
static std::vector<FriendKey> friends;

void HasLoggedIn(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = logged_in;
    LOG_WARNING(Service_FRD, "(STUBBED) called, return %u", logged_in);
}

void IsOnline(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = true;
    LOG_WARNING(Service_FRD, "(STUBBED) called, return %u", logged_in);
}

void Login(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 copy_handle = cmd_buff[1];
    u32 completion_handle = cmd_buff[2];
    if (completion_handle) {
        completion_event = Kernel::g_handle_table.Get<Kernel::Event>(completion_handle);
        completion_event->name = "FRD:CompletionEvent";
        completion_event->Signal(); // TODO(mailwl) : need delay it?
    }

    logged_in = true;

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called, copy_handle=0x%08X, completion_handle=0x%08X",
                copy_handle, completion_handle);
}

void Logout(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    logged_in = false;

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyPreference(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = true;               // public mode
    cmd_buff[3] = false;              // show game name
    cmd_buff[4] = false;              // show played game

    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyProfile(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    std::memcpy(&cmd_buff[2], &my_profile, sizeof(Profile));
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyPresence(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 shifted_out_size = cmd_buff[64];
    u32 my_presence_addr = cmd_buff[65];

    ASSERT(shifted_out_size == ((sizeof(MyPresence) << 14) | 2));

    Memory::WriteBlock(my_presence_addr, &my_presence, sizeof(MyPresence));

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyMii(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    std::memcpy(&cmd_buff[2], &my_mii, sizeof(MiiData));
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyPlayingGame(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = 0;
    cmd_buff[3] = 0;
    cmd_buff[4] = 0;
    cmd_buff[5] = 0;
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyFavoriteGame(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = 0;
    cmd_buff[3] = 0;
    cmd_buff[4] = 0;
    cmd_buff[5] = 0;
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyComment(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    Common::UTF8ToUTF16("Hello World!").copy(reinterpret_cast<char16_t*>(&cmd_buff[2]), 0x11);
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetFriendKeyList(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 offset = cmd_buff[1];
    u32 frd_count = cmd_buff[2];

    u32 frd_keys_size = cmd_buff[64] >> 14;
    ASSERT_MSG(frd_keys_size == sizeof(FriendKey) * frd_count, "Output buffer size not match");
    u32 frd_key_addr = cmd_buff[65];

    if( offset < friends.size() )
    for (u32 i = offset; i < frd_count; ++i) {
        if(i >= friends.size()) {
            break;
        }
        Memory::WriteBlock(frd_key_addr + i * sizeof(FriendKey), &friends[i], sizeof(FriendKey));
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = friends.size();
    LOG_WARNING(Service_FRD, "(STUBBED) called, offset=%d, frd_count=%d, frd_key_addr=0x%08X",
                offset, frd_count, frd_key_addr);
}

void GetFriendProfile(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 count = cmd_buff[1];
    u32 frd_key_addr = cmd_buff[3];
    u32 profiles_size = cmd_buff[64] >> 14;
    ASSERT_MSG(profiles_size == sizeof(Profile) * count, "Output buffer size not match");
    u32 profiles_addr = cmd_buff[65];

    Profile zero_profile = {};
    for (u32 i = 0; i < count; ++i) {
        Memory::WriteBlock(profiles_addr + i * sizeof(Profile), &zero_profile, sizeof(Profile));
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD,
                "(STUBBED) called, count=%d, frd_key_addr=0x%08X, profiles_addr=0x%08X", count,
                frd_key_addr, profiles_addr);
}

void GetFriendRelationship(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 size = cmd_buff[1];
    // VAddr friends = cmd_buff[3];

    VAddr buf = cmd_buff[65];
    Memory::ZeroBlock(buf, size);

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetFriendAttributeFlags(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 count = cmd_buff[1];
    u32 frd_key_addr = cmd_buff[3];
    u32 attr_flags_size = (cmd_buff[64] >> 14);
    ASSERT_MSG(attr_flags_size == 4 * count, "Output buffer size not match");
    u32 attr_flags_addr = cmd_buff[65];

    for (u32 i = 0; i < count; ++i) {
        Memory::Write32(attr_flags_addr + i * 4, 0);
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD,
                "(STUBBED) called, count=%d, frd_key_addr=0x%08X, attr_flags_addr=0x%08X", count,
                frd_key_addr, attr_flags_addr);
}

void GetMyFriendKey(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    std::memcpy(&cmd_buff[2], &my_friend_key, sizeof(FriendKey));
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void GetMyScreenName(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    // TODO: (mailwl) get the name from config
    Common::UTF8ToUTF16("Citra").copy(reinterpret_cast<char16_t*>(&cmd_buff[2]), 11);
    LOG_WARNING(Service_FRD, "(STUBBED) called %08X %08X %08X", cmd_buff[2], cmd_buff[3],
                cmd_buff[4]);
}

void AttachToEventNotification(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 copy_handle = cmd_buff[1];
    u32 event_handle = cmd_buff[2];

    if (event_handle) {
        event_notification = Kernel::g_handle_table.Get<Kernel::Event>(event_handle);
        event_notification->name = "FRD:EventNotification";
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called, copy_handle=0x%08X, event=0x%08X", copy_handle,
                event_handle);
}

void UpdateGameModeDescription(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 trans_size = cmd_buff[1];
    u32 addr = cmd_buff[2];

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called, trans_size=0x%08X, addr=0x%08X", trans_size, addr);
}

void GetLastResponseResult(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void PrincipalIdToFriendCode(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 principal_id = cmd_buff[1];

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = principal_id;
    cmd_buff[3] = 0;
    LOG_WARNING(Service_FRD, "(STUBBED) called, principal_id=0x%08X", principal_id);
}

void ResultToErrorCode(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 error_code = cmd_buff[1];
    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    cmd_buff[2] = error_code;
    LOG_WARNING(Service_FRD, "(STUBBED) called, error=0x%08X", error_code);
}

void SetNotificationMask(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 notif_mask = cmd_buff[1];

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called, notif_mask=0x%08X", notif_mask);
}

void GetGameAuthenticationData(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 size = cmd_buff[64] >> 14;
    VAddr addr = cmd_buff[65];

    Memory::ZeroBlock(addr, size);

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called, addr=0x%08X, size: 0x%X", addr, size);
}

void RequestGameAuthentication(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 p2 = cmd_buff[1];
    // 2 - ... -> wchat_t [0x16 bytes]
    u8 p4 = cmd_buff[8] & 0xFF;
    u8 p5 = cmd_buff[9] & 0xFF;
    // cmd_buff[10] = ProcessId [0x20]
    // cmd_buff[12] = CopyHandle
    Handle event = cmd_buff[13];
    auto evt = Kernel::g_handle_table.Get<Kernel::Event>(event);
    if (evt) {
        evt->Signal();
    }
    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called, p2=0x%08X, event: 0x%X", p2, event);
}

void AddOrUpdateFriend(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    FriendKey* key = reinterpret_cast<FriendKey*>(cmd_buff + 1);
    MiiData* mii = reinterpret_cast<MiiData*>(cmd_buff + 5);
    // FriendPersistentInfo (cmd_buff+29) size=0x48
    wchar_t* str = reinterpret_cast<wchar_t*>(cmd_buff + 47);
    std::string s = Common::UTF16ToUTF8(str);
    bool unk1 = cmd_buff[53];
    u8 unk2 = cmd_buff[54];

    friends.push_back(*key);

    if (friends.size() > 100) {
        cmd_buff[1] = -1;
        LOG_WARNING(Service_FRD, "(STUBBED) called, too many friend");
        return ;
    }

    cmd_buff[1] = RESULT_SUCCESS.raw; // No error
    LOG_WARNING(Service_FRD, "(STUBBED) called, unk1=%u, unk2=%u, %s", unk1, unk2, s.c_str());
}

void RemoveFriend(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    FriendKey* key = reinterpret_cast<FriendKey*>(cmd_buff + 1);

    for (auto frd = friends.begin(); frd != friends.end(); ++frd) {
        if( *key == *frd) {
            friends.erase(frd);
            break;
        }
    }

    LOG_WARNING(Service_FRD, "(STUBBED) called");
}

void Init() {
    using namespace Kernel;

    AddService(new FRD_A_Interface);
    AddService(new FRD_U_Interface);

    completion_event = nullptr;
    event_notification = nullptr;

    logged_in = false;
}

void Shutdown() {
    completion_event = nullptr;
    event_notification = nullptr;

    logged_in = false;
}

} // namespace FRD

} // namespace Service
