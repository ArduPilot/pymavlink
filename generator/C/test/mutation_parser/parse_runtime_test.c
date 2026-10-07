/*
 * parse_runtime_test.c
 *
 * Runtime behaviour test for the generated MAVLink parser.
 *
 * The test builds purpose-made frames and feeds them byte by byte into
 * mavlink_parse_char(). Every assertion describes an observable property of
 * the unmodified generated code; each of the ten regression cases below maps
 * to exactly one line of the generated mavlink_helpers.h that, when mutated,
 * makes the corresponding assertion fail.
 *
 * The frame bytes are built with a local, independent CRC-16/X.25
 * implementation (an alias of the checksum.h generator code, kept here so the
 * frame checksums are a golden answer and not produced by the parser under
 * test).
 *
 * The v2-only cases are compiled in when the generated headers describe the
 * MAVLink 2 wire protocol (MAVLINK_STX == 253). The same source is also
 * compiled against the 1.0 (0xFE) headers, where only the shared cases run.
 * The short-payload / message-length case is only active when built with
 * -DMAVLINK_CHECK_MESSAGE_LENGTH.
 *
 * Build: cc -Wall -Werror -O0 -Wno-address-of-packed-member \
 *            -I<output_dir> parse_runtime_test.c
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/mavlink.h"

/* ------------------------------------------------------------------ */
/* Independent CRC (MAVLink CRC-16/X.25, init 0xFFFF).                 */

static uint16_t crc_akku(uint8_t b, uint16_t crc)
{
    uint16_t tmp = (uint16_t)((b ^ (crc & 0xFF)) & 0xFF);
    tmp = (uint16_t)(tmp ^ (tmp << 4)) & 0xFF;
    return (uint16_t)(((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF);
}

static uint16_t crc_frame(const uint8_t *p, size_t n, uint8_t crc_extra)
{
    uint16_t crc = 0xFFFF;
    size_t i;
    for (i = 1; i < n; i++) {
        crc = crc_akku(p[i], crc);
    }
    crc = crc_akku(crc_extra, crc);
    return crc;
}

/* ------------------------------------------------------------------ */
/* Frame builder. Mirrors the builder in the differential analysis      */
/* (analyse_neu/gen/inputs.py) so the wire bytes are identical.        */

static uint8_t g_frame[600];
static size_t g_flen;
static uint16_t g_fcrc;

static void frame_bauen(uint8_t magic, uint8_t len, uint8_t incompat,
                        uint8_t seq, const uint8_t *sysid, size_t nsys,
                        uint8_t compid, uint32_t msgid,
                        const uint8_t *target, size_t ntarget,
                        const uint8_t *payload, size_t npayload,
                        uint8_t crc_extra, int flip_crc)
{
    size_t o = 0;
    g_frame[o++] = magic;
    g_frame[o++] = len;
    if (magic == 0xFE) {
        g_frame[o++] = seq;
        g_frame[o++] = sysid[0];
        g_frame[o++] = compid;
        g_frame[o++] = (uint8_t)msgid;
    } else {
        g_frame[o++] = incompat;
        g_frame[o++] = 0;
        g_frame[o++] = seq;
        memcpy(g_frame + o, sysid, nsys);
        o += nsys;
        g_frame[o++] = compid;
        g_frame[o++] = (uint8_t)(msgid & 0xFF);
        g_frame[o++] = (uint8_t)((msgid >> 8) & 0xFF);
        g_frame[o++] = (uint8_t)((msgid >> 16) & 0xFF);
        if (incompat & 0x04) {
            memcpy(g_frame + o, target, ntarget);
            o += ntarget;
        }
    }
    memcpy(g_frame + o, payload, npayload);
    o += npayload;
    g_fcrc = crc_frame(g_frame, o, crc_extra);
    if (flip_crc) {
        g_fcrc ^= 0x0055u;
    }
    g_frame[o++] = (uint8_t)(g_fcrc & 0xFF);
    g_frame[o++] = (uint8_t)(g_fcrc >> 8);
    g_flen = o;
}

/* Heartbeat payload of length 9 (as in the differential analysis). */
static const uint8_t g_muster9[9] = {0x11, 0x22, 0x33, 0x44, 0x01,
                                     0x01, 0x01, 0x01, 0x03};

static uint8_t crc_extra_v(uint32_t msgid)
{
#if MAVLINK_STX == 253
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msgid);
    if (e != NULL) {
        return e->crc_extra;
    }
    return 0;
#else
    /* MAVLink 1 wire: the crc_extra table is generated into the dialect
       header. Only the heartbeat (msgid 0) frames are built on this path,
       and its crc_extra is the well-known value 50. */
    if (msgid == 0) {
        return 50u;
    }
    return 0;
#endif
}

/* Parse one frame; returns the last mavlink_parse_char() result. */
static int parse_frame(const uint8_t *f, size_t n,
                       mavlink_message_t *msg, mavlink_status_t *st)
{
    size_t i;
    int rc = 0;
    memset(msg, 0, sizeof(*msg));
    memset(st, 0, sizeof(*st));
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    for (i = 0; i < n; i++) {
        rc = mavlink_parse_char(MAVLINK_COMM_0, f[i], msg, st);
    }
    return rc;
}

int main(void)
{
    mavlink_message_t msg;
    mavlink_status_t st;
    int rc;

    /* ---- Shared smoke case (runs on 1.0 and 2.0).                        */
    /* H: r_message->len must be visible right after a good frame.         */
    frame_bauen(0xFE, 9, 0, 1, (const uint8_t[]){1}, 1, 1, 0, NULL, 0,
                g_muster9, 9, crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc == MAVLINK_FRAMING_OK);
    assert(msg.msgid == 0);
    assert(msg.len == 9);

    /* The length must already be visible while the frame is still being
       parsed (mavlink_helpers.h:1036 refreshes r_message->len on every
       byte); a caller can use this during streaming. Regression
       mavlink_helpers.h:1036 (``!=`` -> ``==``) stops the refresh and the
       final value is copied into the message anyway, so the mid-frame
       visibility is the distinguishing observable. */
    {
        unsigned k;
        memset(&msg, 0, sizeof(msg));
        memset(&st, 0, sizeof(st));
        mavlink_reset_channel_status(MAVLINK_COMM_0);
        for (k = 0; k < 7; k++) {
            mavlink_parse_char(MAVLINK_COMM_0, g_frame[k], &msg, &st);
        }
        assert(msg.len == 9);
    }

    /* A good MAVLink 2 frame is accepted by the 2.0 parser too.           */
#if MAVLINK_STX == 253
    frame_bauen(0xFD, 9, 0, 4, (const uint8_t[]){1}, 1, 1, 0, NULL, 0,
                g_muster9, 9, crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc == MAVLINK_FRAMING_OK);
    assert(msg.len == 9);
#endif

    /* ---- BAD_CRC: the wire checksum must be copied into the message     */
    /*      (mavlink_helpers.h ~line 1058; shared with the 1.0 template).  */
    /*      mavlink_parse_char() swallows BAD_CRC (returns 0), so the bad   */
    /*      frame is recognised via the channel parse_error counter.       */
    frame_bauen(0xFE, 9, 0, 8, (const uint8_t[]){1}, 1, 1, 0, NULL, 0,
                g_muster9, 9, crc_extra_v(0), 1);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(mavlink_get_channel_status(MAVLINK_COMM_0)->parse_error > 0);
    assert(msg.checksum == g_fcrc);

#if MAVLINK_STX == 253
    /* ---- 1) SYSID32: 4-byte source system id.                           */
    /*      Regression mavlink_helpers.h:826  (24 -> 25).                  */
    frame_bauen(0xFD, 9, 0x02, 5, (const uint8_t[]){1, 2, 3, 4}, 4, 1, 0,
                NULL, 0, g_muster9, 9, crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc == MAVLINK_FRAMING_OK);
    assert(msg.sysid == 0x04030201u);

    /* ---- 2) TARGET32: 4-byte target system id.                          */
    /*      Regression mavlink_helpers.h:893  (8 -> 9).                    */
    frame_bauen(0xFD, 9, 0x04, 6, (const uint8_t[]){1}, 1, 1, 0,
                (const uint8_t[]){1, 2, 3, 4}, 4, g_muster9, 9,
                crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc == MAVLINK_FRAMING_OK);
    assert(msg.target_sysid == 0x04030201u);

    /* ---- 3) TARGET32 with a 1-byte payload: the message must be         */
    /*      accepted (len > 0 goes to the payload state).                  */
    /*      Regression mavlink_helpers.h:907  (0 -> 1).                    */
    /*      Only without the length check (a 1-byte heartbeat is shorter   */
    /*      than its minimum length and is rejected with the check on).    */
#ifndef MAVLINK_CHECK_MESSAGE_LENGTH
    frame_bauen(0xFD, 1, 0x04, 12, (const uint8_t[]){1}, 1, 1, 0,
                (const uint8_t[]){1, 2, 3, 4}, 4, (const uint8_t[]){0x5A}, 1,
                crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc == MAVLINK_FRAMING_OK);
    assert(msg.len == 1);
    assert(msg.target_sysid == 0x04030201u);
    frame_bauen(0xFD, 1, 0x04, 13, (const uint8_t[]){1}, 1, 1, 0,
                (const uint8_t[]){4, 3, 2, 1}, 4, (const uint8_t[]){0xC3}, 1,
                crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc == MAVLINK_FRAMING_OK);
    assert(msg.target_sysid == 0x01020304u);
#endif

    /* ---- 5) Short payload: the buffer must be zero-filled.              */
    /*      Regression mavlink_helpers.h:946  (0 -> 1).                    */
    /*      Only meaningful without the message-length check, which would  */
    /*      reject this short frame earlier.                               */
#ifndef MAVLINK_CHECK_MESSAGE_LENGTH
    frame_bauen(0xFE, 0, 0, 2, (const uint8_t[]){1}, 1, 1, 0, NULL, 0,
                NULL, 0, crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc == MAVLINK_FRAMING_OK);
    assert(msg.len == 0);
    assert(_MAV_PAYLOAD(&msg)[0] == 0);
    assert(_MAV_PAYLOAD(&msg)[15] == 0);
#endif

    /* ---- Unknown message id: BAD_CRC path. The wire checksum must be    */
    /*      copied into r_message->checksum.                               */
    /*      Regression mavlink_helpers.h:1049 and :1058.                   */
    frame_bauen(0xFD, 0, 0, 7, (const uint8_t[]){1}, 1, 1, 0x030201, NULL, 0,
                NULL, 0, 0, 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(mavlink_get_channel_status(MAVLINK_COMM_0)->parse_error > 0);
    assert(msg.checksum == g_fcrc);

    /* ---- Target system look-up: a zero-trimmed payload must report the  */
    /*      target as 0 (broadcast), not read past len.                    */
    /*      Regression mavlink_helpers.h:673  (>= -> >).                   */
    {
        uint32_t mid;
        const mavlink_msg_entry_t *e = NULL;
        int found = 0;
        for (mid = 0; mid < 0x10000u; mid++) {
            e = mavlink_get_msg_entry(mid);
            if (e != NULL &&
                (e->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) != 0 &&
                e->target_system_ofs >= 1 &&
                e->target_system_ofs < MAVLINK_MAX_PAYLOAD_LEN) {
                found = 1;
                break;
            }
        }
        if (found) {
            unsigned i;
            memset(&msg, 0, sizeof(msg));
            msg.msgid = mid;
            msg.len = (uint8_t)e->target_system_ofs;
            for (i = 0; i < MAVLINK_MAX_PAYLOAD_LEN; i++) {
                _MAV_PAYLOAD_NON_CONST(&msg)[i] = 0x55;
            }
            _MAV_PAYLOAD_NON_CONST(&msg)[e->target_system_ofs] = 0x2A;
            assert(mavlink_msg_get_target_sysid(&msg, e) == 0);
        }
    }
#endif

#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
    /* ---- Missing/rejecting length check. A MAVLink 1 short heartbeat    */
    /*      (len 0 < 9) must be rejected early and therefore never          */
    /*      produces a good frame.                                          */
    /*      Regression mavlink_helpers.h:847  (|| -> &&).                  */
    frame_bauen(0xFE, 0, 0, 2, (const uint8_t[]){1}, 1, 1, 0, NULL, 0,
                NULL, 0, crc_extra_v(0), 0);
    rc = parse_frame(g_frame, g_flen, &msg, &st);
    assert(rc != MAVLINK_FRAMING_OK);
#endif

    printf("parse_runtime_test: PASS (MAVLINK_STX=%d)\n", MAVLINK_STX);
    return 0;
}