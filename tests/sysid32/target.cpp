#include <cassert>
#include <cstdio>
#include "common/common.hpp"
namespace mavlink {
const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t id) {
    for (auto &entry : common::MESSAGE_ENTRIES) if (entry.msgid == id) return &entry;
    return nullptr;
}
}
int main() {
    using namespace mavlink;
    for (uint32_t source : {42U, 0xABCDEF12U}) for (uint32_t target : {0U, 7U, 255U, 256U, 0xFFFFFFFFU}) {
        for (bool signed_frame : {false, true}) {
            auto *status = mavlink_get_channel_status(0);
            *status = {};
            mavlink_signing_t signing {};
            if (signed_frame) {
                memset(signing.secret_key, 42, 32);
                signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
                signing.link_id = 3;
                signing.timestamp = 1000;
                status->signing = &signing;
            }
            common::msg::COMMAND_LONG command {};
            command.target_system = target;
            command.target_component = 250;
            command.command = 300;
            command.confirmation = 1;
            command.param1 = 1; command.param2 = 2; command.param3 = 3;
            command.param4 = 4; command.param5 = 5; command.param6 = 6; command.param7 = 7;
            auto msg = command.pack(source, 11);
            uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
            auto length = mavlink_msg_to_send_buffer(bytes, &msg);
            for (unsigned i=0; i<length; ++i) printf("%02x", bytes[i]);
            puts("");
            mavlink_message_t rx {}, result {};
            mavlink_status_t rxstatus {}, out {};
            for (unsigned i=0; i<length; ++i) {
                auto received = mavlink_frame_char_buffer(&rx, &rxstatus, bytes[i], &result, &out);
                assert(received == (i+1 == length ? MAVLINK_FRAMING_OK : MAVLINK_FRAMING_INCOMPLETE));
            }
            MsgMap map(result);
            common::msg::COMMAND_LONG decoded {};
            decoded.deserialize(map);
            assert(result.sysid == source && decoded.target_system == target);
            assert(decoded.target_component == 250 && decoded.param7 == 7);
            decoded.target_system = 7;
            auto small = decoded.pack(source, 11);
            assert(!(small.incompat_flags & MAVLINK_IFLAG_TARGET32));
            assert(mavlink_msg_get_target_sysid(&small, mavlink_get_msg_entry(small.msgid)) == 7);
            status->flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            decoded.target_system = 256;
            bool rejected = false;
            try { decoded.pack(42, 11); } catch (const std::runtime_error &) { rejected = true; }
            assert(rejected);
        }
    }
}
