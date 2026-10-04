/*
 * Getters on a locally packed MAVLink 2 message must not read the checksum.
 *
 * Packing trims trailing zero bytes from the payload and finalize then writes
 * the checksum at payload[len] and payload[len+1]. Fields at or after len are
 * zero on the wire, so the getters must return zero for them. See
 * https://github.com/ArduPilot/pymavlink/issues/1142
 */
#include <stdio.h>
#include <string.h>
#include "common/mavlink.h"

static int failures;

static void dump(const char *label, const char *buf, size_t n)
{
    printf("%s \"", label);
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)buf[i];
        if (c >= 32 && c < 127) {
            putchar(c);
        } else {
            printf("\\x%02x", c);
        }
    }
    printf("\"\n");
}

static void check_text(const mavlink_message_t *msg, const char *expected)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
    memset(text, 0x55, sizeof(text));
    mavlink_msg_statustext_get_text(msg, text);

    char want[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = {0};
    strncpy(want, expected, sizeof(want));
    if (memcmp(text, want, sizeof(text)) != 0) {
        printf("FAIL get_text (len=%u, checksum=0x%04x)\n", msg->len, msg->checksum);
        dump("  expected", want, strlen(expected) + 2);
        dump("  actual  ", text, strlen(expected) + 2);
        failures++;
    }
}

static void check_u16(const char *name, unsigned actual, unsigned expected, const mavlink_message_t *msg)
{
    if (actual != expected) {
        printf("FAIL %s: expected %u, got %u (0x%04x) (len=%u, checksum=0x%04x)\n",
               name, expected, actual, actual, msg->len, msg->checksum);
        failures++;
    }
}

static void pack_statustext(mavlink_message_t *msg, const char *s, uint16_t id, uint8_t chunk_seq)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = {0};
    strncpy(text, s, sizeof(text));
    mavlink_msg_statustext_pack(255, MAV_COMP_ID_USER1, msg, MAV_SEVERITY_INFO, text, id, chunk_seq);
}

int main(void)
{
    mavlink_message_t msg;

    /* id=0, chunk_seq=0: trim cuts into the text field (QGC StatusTextHandlerTest) */
    pack_statustext(&msg, "Hello", 0, 0);
    check_text(&msg, "Hello");
    check_u16("get_id (id=0)", mavlink_msg_statustext_get_id(&msg), 0, &msg);
    check_u16("get_chunk_seq (id=0)", mavlink_msg_statustext_get_chunk_seq(&msg), 0, &msg);

    /* id=42: high byte of id and chunk_seq are trimmed, checksum lands on them */
    pack_statustext(&msg, "Hello", 42, 0);
    check_text(&msg, "Hello");
    check_u16("get_id (id=42)", mavlink_msg_statustext_get_id(&msg), 42, &msg);
    check_u16("get_chunk_seq (id=42)", mavlink_msg_statustext_get_chunk_seq(&msg), 0, &msg);

    /* decode must agree with the getters */
    mavlink_statustext_t decoded;
    mavlink_msg_statustext_decode(&msg, &decoded);
    check_u16("decode id (id=42)", decoded.id, 42, &msg);

    /* regression check, passes before and after the fix: chunk_seq=7 is non-zero, so nothing is trimmed */
    pack_statustext(&msg, "Hello", 300, 7);
    check_text(&msg, "Hello");
    check_u16("get_id (id=300)", mavlink_msg_statustext_get_id(&msg), 300, &msg);
    check_u16("get_chunk_seq (id=300)", mavlink_msg_statustext_get_chunk_seq(&msg), 7, &msg);

    /* received messages: the parser zero-fills after len, so these passed before the fix too */
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    const uint16_t ids[] = {0, 42};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        mavlink_message_t rx;
        mavlink_status_t status;
        memset(&rx, 0, sizeof(rx));
        memset(&status, 0, sizeof(status));
        pack_statustext(&msg, "Hello", ids[i], 0);
        uint16_t n = mavlink_msg_to_send_buffer(wire, &msg);
        uint8_t got = 0;
        for (uint16_t j = 0; j < n && !got; j++) {
            got = mavlink_parse_char(MAVLINK_COMM_1, wire[j], &rx, &status);
        }
        if (!got) {
            printf("FAIL parse (id=%u): no message\n", ids[i]);
            failures++;
            continue;
        }
        printf("parsed id=%u: len=%u\n", ids[i], rx.len);
        check_text(&rx, "Hello");
        check_u16(ids[i] ? "parsed get_id (id=42)" : "parsed get_id (id=0)",
                  mavlink_msg_statustext_get_id(&rx), ids[i], &rx);
    }

    if (failures == 0) {
        printf("OK\n");
    }
    return failures == 0 ? 0 : 1;
}
