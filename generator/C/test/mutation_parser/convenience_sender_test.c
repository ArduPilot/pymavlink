/*
 * convenience_sender_test.c
 *
 * Runtime behaviour test for the convenience sender
 * (_mav_finalize_message_chan_send_target) of the generated MAVLink 2
 * C headers.
 *
 * A target system id of 256 (> 255) must set the MAVLINK_IFLAG_TARGET32
 * flag and therefore extend the header by the four target bytes, giving a
 * frame length of 25 for a 9-byte heartbeat payload.
 *
 * Regression: mavlink_helpers.h:441 (255 -> 256) drops the flag and sends
 * a 21-byte frame instead, so the assertion below fails on the mutated
 * generated code.
 *
 * The v1 (0xFE) generated headers cannot express a 32-bit target and this
 * whole test compiles out there (gated on MAVLINK_STX == 253).
 *
 * Build: cc -Wall -Werror -O0 -Wno-address-of-packed-member \
 *            -I<output_dir> convenience_sender_test.c
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_START_UART_SEND(chan, len) (void)(chan), (void)(len)
#define MAVLINK_END_UART_SEND(chan, len) (void)(chan), (void)(len)
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) uart_bytes((buf), (len))

static uint8_t AUSGABE[600];
static uint16_t AUSGABE_LEN;

static void uart_bytes(const uint8_t *buf, uint16_t len)
{
    memcpy(AUSGABE + AUSGABE_LEN, buf, len);
    AUSGABE_LEN = (uint16_t)(AUSGABE_LEN + len);
}

#include "mavlink_types.h"

mavlink_system_t mavlink_system = {1, 1};

#include "common/mavlink.h"

int main(void)
{
#if MAVLINK_STX == 253
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    static uint8_t payload[64];
    unsigned i;

    memset(status, 0, sizeof(*status));
    status->signing = NULL;
    status->signing_streams = NULL;
    status->flags = 0;
    for (i = 0; i < sizeof(payload); i++) {
        payload[i] = (uint8_t)(i + 1);
    }

    AUSGABE_LEN = 0;
    _mav_finalize_message_chan_send_target(MAVLINK_COMM_0, 256,
                                           (const char *)payload, 9, 9, 7,
                                           256);

    assert(AUSGABE_LEN == 25);
    assert(AUSGABE[0] == 0xFD);
    assert((AUSGABE[2] & MAVLINK_IFLAG_TARGET32) != 0);
#endif
    printf("convenience_sender_test: PASS (MAVLINK_STX=%d)\n", MAVLINK_STX);
    return 0;
}