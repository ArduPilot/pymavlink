#include <assert.h>
#include "common/mavlink.h"

#if !defined(MAVLINK_HAVE_GET_TARGET_SYSTEM) || !defined(MAVLINK_HAVE_GET_TARGET_SYSID)
#error Target getter feature macros must also be visible with separate helpers
#endif

int main(void)
{
    // Preserve every ordinary destination, and evaluate the argument once.
    for (uint32_t target = 0; target <= UINT8_MAX; target++) {
        assert(mavlink_msg_target_field(target) == target);
    }
    uint32_t next_target = 255;
    assert(mavlink_msg_target_field(next_target++) == 255);
    assert(next_target == 256);
    assert(mavlink_msg_target_field(next_target++) == MAVLINK_TARGET_SYSTEM_SENTINEL);
    assert(next_target == 257);
    assert(mavlink_msg_target_field(UINT32_MAX) == MAVLINK_TARGET_SYSTEM_SENTINEL);

    const uint32_t targets[] = {0, 7, 255, 256, 0x7fffffff, 0x80000000, 0xffffffff};
    for (unsigned i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
        for (unsigned signed_frame = 0; signed_frame < 2; signed_frame++) {
            mavlink_message_t msg = {0};
            mavlink_command_long_t packet;
            mavlink_msg_command_long_pack(42, 11, &msg, targets[i], 250, 300, 1,
                                          1, 2, 3, 4, 5, 6, 7);
            mavlink_msg_command_long_decode(&msg, &packet);
            assert(mavlink_msg_command_long_get_param1(&msg) == 1.0f);
            assert(mavlink_msg_command_long_get_param7(&msg) == 7.0f);
            uint32_t target = 123;
            assert(mavlink_msg_get_target_system(&msg, &packet.target_system, &target));
            assert(target == targets[i]);

            // A received wide header overrides a conflicting decoded field.
            msg.incompat_flags |= MAVLINK_IFLAG_TARGET32;
            if (signed_frame) {
                msg.incompat_flags |= MAVLINK_IFLAG_SIGNED;
            }
            msg.target_sysid = targets[i];
            packet.target_system = 99;
            assert(mavlink_msg_get_target_system(&msg, &packet.target_system, &target));
            assert(target == targets[i]);
            assert(mavlink_msg_get_target_system(&msg, NULL, &target));
            assert(target == targets[i]);
        }
    }

    for (unsigned flags = 0; flags < MAVLINK_IFLAG_TARGET32; flags++) {
        mavlink_message_t msg = {0};
        msg.incompat_flags = flags;
        msg.target_sysid = 42; // stale metadata must not invent a header target
        uint32_t target = 0x87654321;
        assert(!mavlink_msg_get_target_system(&msg, NULL, &target));
        assert(target == 0x87654321);
        for (unsigned payload_target = 0; payload_target <= 255; payload_target++) {
            uint8_t field = payload_target;
            assert(mavlink_msg_get_target_system(&msg, &field, &target));
            assert(target == payload_target);
        }
    }

    // MAVLink1 and zero-trimmed payload targets still come from decoded fields.
    mavlink_message_t msg = {0};
    mavlink_status_t status = {0};
    status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_msg_command_long_pack_status(42, 11, &status, &msg, 255, 1, 300, 0,
                                         0, 0, 0, 0, 0, 0, 0);
    assert(msg.magic == MAVLINK_STX_MAVLINK1);
    mavlink_command_long_t packet;
    mavlink_msg_command_long_decode(&msg, &packet);
    uint32_t target;
    assert(mavlink_msg_get_target_system(&msg, &packet.target_system, &target));
    assert(target == 255);
    mavlink_msg_command_long_pack(42, 11, &msg, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    mavlink_msg_command_long_decode(&msg, &packet);
    assert(mavlink_msg_get_target_system(&msg, &packet.target_system, &target));
    assert(target == 0);
    // A wide system target does not introduce a component: use its payload byte,
    // including broadcast zero when MAVLink2 trims that byte off the wire.
    for (unsigned component = 0; component <= 255; component++) {
        mavlink_msg_command_long_pack(42, 11, &msg, UINT32_MAX, component, 300, 0,
                                      0, 0, 0, 0, 0, 0, 0);
        assert(mavlink_msg_get_target_compid(&msg, mavlink_get_msg_entry(msg.msgid)) == component);
        uint8_t wire[MAVLINK_MAX_PACKET_LEN];
        const uint16_t length = mavlink_msg_to_send_buffer(wire, &msg);
        mavlink_message_t rx = {0}, received;
        mavlink_status_t parse_status = {0}, received_status;
        for (unsigned byte = 0; byte < length; byte++) {
            const uint8_t result = mavlink_frame_char_buffer(&rx, &parse_status, wire[byte],
                                                             &received, &received_status);
            assert(result == (byte + 1 == length ? MAVLINK_FRAMING_OK : MAVLINK_FRAMING_INCOMPLETE));
        }
        assert(mavlink_msg_get_target_compid(&received, mavlink_get_msg_entry(received.msgid)) == component);
        mavlink_msg_command_long_decode(&received, &packet);
        assert(packet.target_component == component);
    }
    // Decode/edit/repack must recover the full target before leaving the message.
    // The uint8_t struct sentinel also represents a legitimate destination 255.
    for (unsigned i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
        mavlink_message_t original = {0}, relayed = {0};
        mavlink_msg_command_long_pack(42, 11, &original, targets[i], 250, 300, 1,
                                      1, 2, 3, 4, 5, 6, 7);
        mavlink_msg_command_long_decode(&original, &packet);
        const uint32_t full_target = mavlink_msg_get_target_sysid(
            &original, mavlink_get_msg_entry(original.msgid));
        assert(full_target == targets[i]);
        assert(packet.target_system == (targets[i] > UINT8_MAX ? MAVLINK_TARGET_SYSTEM_SENTINEL : targets[i]));
        assert((uint8_t)_MAV_PAYLOAD(&original)[30] == packet.target_system);
        packet.param1 = 8;
        mavlink_msg_command_long_pack(42, 11, &relayed, full_target, packet.target_component,
            packet.command, packet.confirmation, packet.param1, packet.param2, packet.param3,
            packet.param4, packet.param5, packet.param6, packet.param7);
        assert(mavlink_msg_get_target_sysid(&relayed, mavlink_get_msg_entry(relayed.msgid)) == targets[i]);
        assert(!!(relayed.incompat_flags & MAVLINK_IFLAG_TARGET32) == (targets[i] > UINT8_MAX));
        assert(mavlink_msg_command_long_get_param1(&relayed) == 8);
    }
    return 0;
}
