#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "common/mavlink.h"
int main(void) {
    const uint32_t sources[] = {42, 0xABCDEF12};
    const uint32_t targets[] = {0, 7, 255, 256, 0xFFFFFFFF};
    for (unsigned s=0; s<2; s++) for (unsigned t=0; t<5; t++) for (unsigned signed_frame=0; signed_frame<2; signed_frame++) {
        mavlink_message_t msg = {0}, received = {0}, rx = {0};
        mavlink_status_t status = {0}, rxstatus = {0}, output = {0};
        mavlink_signing_t signing = {0}, rxsigning = {0};
        mavlink_signing_streams_t streams = {0};
        status.current_tx_seq = 0;
        if (signed_frame) {
            memset(signing.secret_key, 42, 32);
            signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
            signing.link_id = 3;
            signing.timestamp = 1000;
            status.signing = &signing;
            rxsigning = signing;
            rxstatus.signing = &rxsigning;
            rxstatus.signing_streams = &streams;
        }
        const uint16_t length = mavlink_msg_command_long_pack_status(sources[s], 11, &status, &msg,
            targets[t], 250, 300, 1, 1, 2, 3, 4, 5, 6, 7);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        assert(length == mavlink_msg_to_send_buffer(buf, &msg));
        for (unsigned i=0; i<length; i++) {
            printf("%02x", buf[i]);
            uint8_t result = mavlink_frame_char_buffer(&rx, &rxstatus, buf[i], &received, &output);
            assert(result == (i+1 == length ? MAVLINK_FRAMING_OK : MAVLINK_FRAMING_INCOMPLETE));
        }
        puts("");
        assert(received.sysid == sources[s]);
        assert(mavlink_msg_get_target_sysid(&received, mavlink_get_msg_entry(received.msgid)) == targets[t]);
        assert(mavlink_msg_get_target_compid(&received, mavlink_get_msg_entry(received.msgid)) == 250);
        status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        assert(mavlink_msg_command_long_pack_status(42, 11, &status, &msg, 256, 250, 300, 1, 1, 2, 3, 4, 5, 6, 7) == 0);
    }
    return 0;
}
