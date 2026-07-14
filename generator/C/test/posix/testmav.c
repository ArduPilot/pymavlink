/*
  simple MAVLink testsuite for C
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#include <stdbool.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_COMM_NUM_BUFFERS 2

// this trick allows us to make mavlink_message_t as small as possible
// for this dialect, which saves some memory
#include <version.h>
#define MAVLINK_MAX_PAYLOAD_LEN MAVLINK_MAX_DIALECT_PAYLOAD_SIZE

#include <mavlink_types.h>
static mavlink_system_t mavlink_system = {42,11,};

#define MAVLINK_ASSERT(x) assert(x)
static void comm_send_ch(mavlink_channel_t chan, uint8_t c);

static mavlink_message_t last_msg;

#include <mavlink.h>
#include <testsuite.h>

static unsigned chan_counts[MAVLINK_COMM_NUM_BUFFERS];

#ifndef MAVLINK_HAVE_MIN_MESSAGE_LENGTH
static const uint8_t message_lengths[] = MAVLINK_MESSAGE_LENGTHS;
#define mavlink_min_message_length(msg) message_lengths[(msg)->msgid]
#endif

#ifndef MAVLINK_HAVE_GET_MESSAGE_INFO
static const mavlink_message_info_t message_info[] = MAVLINK_MESSAGE_INFO;
#define mavlink_get_message_info(msg) &message_info[(msg)->msgid]
#endif

static unsigned error_count;


static void print_one_field(mavlink_message_t *msg, const mavlink_field_info_t *f, int idx)
{
#define PRINT_FORMAT(f, def) (f->print_format?f->print_format:def)
	switch (f->type) {
	case MAVLINK_TYPE_CHAR:
		printf(PRINT_FORMAT(f, "%c"), _MAV_RETURN_char(msg, f->wire_offset+idx*1));
		break;
	case MAVLINK_TYPE_UINT8_T:
		printf(PRINT_FORMAT(f, "%u"), _MAV_RETURN_uint8_t(msg, f->wire_offset+idx*1));
		break;
	case MAVLINK_TYPE_INT8_T:
		printf(PRINT_FORMAT(f, "%d"), _MAV_RETURN_int8_t(msg, f->wire_offset+idx*1));
		break;
	case MAVLINK_TYPE_UINT16_T:
		printf(PRINT_FORMAT(f, "%u"), _MAV_RETURN_uint16_t(msg, f->wire_offset+idx*2));
		break;
	case MAVLINK_TYPE_INT16_T:
		printf(PRINT_FORMAT(f, "%d"), _MAV_RETURN_int16_t(msg, f->wire_offset+idx*2));
		break;
	case MAVLINK_TYPE_UINT32_T:
		printf(PRINT_FORMAT(f, "%lu"), (unsigned long)_MAV_RETURN_uint32_t(msg, f->wire_offset+idx*4));
		break;
	case MAVLINK_TYPE_INT32_T:
		printf(PRINT_FORMAT(f, "%ld"), (long)_MAV_RETURN_int32_t(msg, f->wire_offset+idx*4));
		break;
	case MAVLINK_TYPE_UINT64_T:
		printf(PRINT_FORMAT(f, "%llu"), (unsigned long long)_MAV_RETURN_uint64_t(msg, f->wire_offset+idx*8));
		break;
	case MAVLINK_TYPE_INT64_T:
		printf(PRINT_FORMAT(f, "%lld"), (long long)_MAV_RETURN_int64_t(msg, f->wire_offset+idx*8));
		break;
	case MAVLINK_TYPE_FLOAT:
		printf(PRINT_FORMAT(f, "%f"), (double)_MAV_RETURN_float(msg, f->wire_offset+idx*4));
		break;
	case MAVLINK_TYPE_DOUBLE:
		printf(PRINT_FORMAT(f, "%f"), _MAV_RETURN_double(msg, f->wire_offset+idx*8));
		break;
	}
}

static void print_field(mavlink_message_t *msg, const mavlink_field_info_t *f)
{
	printf("%s: ", f->name);
	if (f->array_length == 0) {
		print_one_field(msg, f, 0);
		printf(" ");
	} else {
		unsigned i;
		/* print an array */
		if (f->type == MAVLINK_TYPE_CHAR) {
			printf("'%.*s'", f->array_length,
			       f->wire_offset+(const char *)_MAV_PAYLOAD(msg));
			
		} else {
			printf("[ ");
			for (i=0; i<f->array_length; i++) {
				print_one_field(msg, f, i);
				if (i < f->array_length) {
					printf(", ");
				}
			}
			printf("]");
		}
	}
	printf(" ");
}


mavlink_status_t *statusp;

static void print_message(mavlink_message_t *msg,mavlink_channel_t chan)
{
	const mavlink_message_info_t *m = mavlink_get_message_info(msg);
	if (m == NULL) {
		printf("ERROR: no message info for %u\n", msg->msgid);
		error_count++;
		return;
	}
	const mavlink_field_info_t *f = m->fields;
	unsigned i;
	printf("sysid:%d ", msg->sysid);
	printf("compid:%d ", msg->compid);
	printf("seq:%d ", msg->seq);
#ifdef MAVLINK_SIGNING_FLAG_SIGN_OUTGOING
    // only print if links has a statusp, and it's not null, and we are channel 1, where signing is active
    if ( statusp && (statusp->signing != NULL) && (chan == MAVLINK_COMM_1) )  
        printf("sign_ts:%ld ", statusp->signing->timestamp-1); // subtract 1 from ts as api increments it before here
#endif
	printf("%s { ", m->name);
	for (i=0; i<m->num_fields; i++) {
		print_field(msg, &f[i]);
	}
	printf("}\n");
}

#ifdef MAVLINK_SIGNING_FLAG_SIGN_OUTGOING
static mavlink_signing_t signing_in[MAVLINK_COMM_NUM_BUFFERS];
static mavlink_signing_streams_t signing_streams_in;
#endif

// when capture_buf is set comm_send_ch appends bytes to it instead of
// parsing, allowing byte-exact tests of the convenience send functions
static uint8_t *capture_buf;
static uint16_t capture_len;

static void comm_send_ch(mavlink_channel_t chan, uint8_t c)
{
	if (capture_buf != NULL) {
		capture_buf[capture_len++] = c;
		return;
	}
	mavlink_status_t status;
	memset(&status, 0, sizeof(status));
#ifdef MAVLINK_SIGNING_FLAG_SIGN_OUTGOING
	status.signing = &signing_in[chan];
        status.signing_streams = &signing_streams_in;
#endif
#define SHOW_AS_HEX 1
#ifdef SHOW_AS_HEX
    printf("%02x ",c);
#endif
	if (mavlink_parse_char(chan, c, &last_msg, &status)) {
#ifdef SHOW_AS_HEX
    printf("\n");
#endif
		print_message(&last_msg,chan);
		chan_counts[chan]++;
		/* channel 0 gets 3 messages per message, because of
		   the channel defaults for _pack() and _encode() */
		if (chan == MAVLINK_COMM_0 && status.current_rx_seq != (uint8_t)(chan_counts[chan]*3)) {
			printf("Channel 0 sequence mismatch error at packet %u (rx_seq=%u)\n", 
			       chan_counts[chan], status.current_rx_seq);
			error_count++;
		} else if (chan > MAVLINK_COMM_0 && status.current_rx_seq != (uint8_t)chan_counts[chan]) {
			printf("Channel %u sequence mismatch error at packet %u (rx_seq=%u)\n", 
			       (unsigned)chan, chan_counts[chan], status.current_rx_seq);
			error_count++;
		}
                // we only check the lengtth for MAVLink1. In MAVLink2 packets are zero trimmed
                if (mavlink_min_message_length(&last_msg) > last_msg.len && last_msg.magic == 254) {
			printf("Incorrect message length %u for message %u - expected %u\n", 
			       (unsigned)last_msg.len, (unsigned)last_msg.msgid,
                               mavlink_min_message_length(&last_msg));
			error_count++;
		}
	}
	if (status.packet_rx_drop_count != 0) {
		printf("Parse error at packet %u\n", chan_counts[chan]);
		error_count++;
	}
}

#if defined(MAVLINK_IFLAG_SYSID32) && defined(MAVLINK_MSG_ID_COMMAND_LONG)
/*
  parse a byte buffer with a private parser, returning the last framing
  status and the decoded message in *out
 */
static uint8_t parse_buffer(const uint8_t *buf, uint16_t len, mavlink_message_t *out,
                            mavlink_signing_t *signing, mavlink_signing_streams_t *streams)
{
    static mavlink_message_t rxmsg;
    static mavlink_status_t rstatus;
    mavlink_status_t status_out;
    uint8_t ret = MAVLINK_FRAMING_INCOMPLETE;
    memset(&rxmsg, 0, sizeof(rxmsg));
    memset(&rstatus, 0, sizeof(rstatus));
    rstatus.signing = signing;
    rstatus.signing_streams = streams;
    for (uint16_t i=0; i<len; i++) {
        uint8_t r = mavlink_frame_char_buffer(&rxmsg, &rstatus, buf[i], out, &status_out);
        if (r != MAVLINK_FRAMING_INCOMPLETE) {
            ret = r;
        }
    }
    return ret;
}

static void test_sysid32(void)
{
    const uint32_t sysids[2] = { 42, 0x0A000001UL };   // 10.0.0.1
    const uint32_t targets[2] = { 7, 0x0A000002UL };   // 10.0.0.2
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(MAVLINK_MSG_ID_COMMAND_LONG);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    printf("Testing 32 bit sysid/target combinations\n");
    assert(entry != NULL);

    // earlier tests may have left the default channel in MAVLink1 mode
    mavlink_status_t *chan_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    chan_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    chan_status->signing = NULL;

    for (uint8_t i=0; i<2; i++) {
        for (uint8_t j=0; j<2; j++) {
            const uint32_t sysid = sysids[i];
            const uint32_t target = targets[j];
            const bool sysid32 = sysid > 255;
            const bool targetted = target > 255;
            mavlink_message_t msg;
            memset(&msg, 0, sizeof(msg));

            mavlink_msg_command_long_pack(sysid, 11, &msg, target, 250, 300, 1,
                                          1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);

            assert(((msg.incompat_flags & MAVLINK_IFLAG_SYSID32) != 0) == sysid32);
            assert(((msg.incompat_flags & MAVLINK_IFLAG_TARGETTED) != 0) == targetted);
            assert(msg.compat_flags == MAVLINK_CFLAG_SYSID32);
            assert(msg.sysid == sysid);

            // getters must return the full target, payload byte must be
            // zero when the target is in the extended header
            assert(mavlink_msg_command_long_get_target_system(&msg) == target);
            assert(mavlink_msg_get_target_sysid(&msg, entry) == target);
            assert(mavlink_msg_get_target_compid(&msg, entry) == 250);
            assert((uint8_t)_MAV_PAYLOAD(&msg)[entry->target_system_ofs] == (targetted?0:target));

            mavlink_command_long_t pkt;
            mavlink_msg_command_long_decode(&msg, &pkt);
            assert(pkt.target_system == (targetted?0:target));
            assert(pkt.target_component == 250);

            // round trip through the wire format
            const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            const uint16_t header_len = MAVLINK_NUM_HEADER_BYTES +
                (sysid32?MAVLINK_SYSID32_HEADER_EXTRA:0) +
                (targetted?MAVLINK_TARGETTED_HEADER_EXTRA:0);
            assert(len == header_len + msg.len + 2);
            assert(len == mavlink_msg_get_send_buffer_length(&msg));

            mavlink_message_t rmsg;
            assert(parse_buffer(buf, len, &rmsg, NULL, NULL) == MAVLINK_FRAMING_OK);
            assert(rmsg.sysid == sysid);
            assert(rmsg.compid == 11);
            assert(rmsg.msgid == MAVLINK_MSG_ID_COMMAND_LONG);
            assert(rmsg.incompat_flags == msg.incompat_flags);
            assert(mavlink_msg_command_long_get_target_system(&rmsg) == target);
            assert(mavlink_msg_command_long_get_target_component(&rmsg) == 250);
            assert(mavlink_msg_get_target_sysid(&rmsg, entry) == target);
            assert(mavlink_msg_command_long_get_command(&rmsg) == 300);
            assert(mavlink_msg_command_long_get_param7(&rmsg) == 7.0f);

            // re-serialization for forwarding must be byte identical
            uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
            const uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &rmsg);
            assert(len2 == len);
            assert(memcmp(buf, buf2, len) == 0);
        }
    }

    // a frame with small IDs must be wire-identical to a MAVLink2.0
    // frame except for the compat_flags byte and the CRC
    {
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(msg));
        mavlink_msg_command_long_pack(42, 11, &msg, 7, 250, 300, 1,
                                      1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        assert(len == MAVLINK_NUM_HEADER_BYTES + msg.len + 2);
        assert(buf[2] == 0);                     // incompat_flags
        assert(buf[3] == MAVLINK_CFLAG_SYSID32); // compat_flags

        // build the equivalent legacy frame (compat_flags 0) and check
        // an old-style sender still parses OK with the new parser
        uint8_t legacy[MAVLINK_MAX_PACKET_LEN];
        memcpy(legacy, buf, len);
        legacy[3] = 0;
        uint16_t crc = crc_calculate(&legacy[1], MAVLINK_CORE_HEADER_LEN);
        crc_accumulate_buffer(&crc, (const char *)&legacy[MAVLINK_NUM_HEADER_BYTES], msg.len);
        crc_accumulate(entry->crc_extra, &crc);
        legacy[len-2] = crc & 0xFF;
        legacy[len-1] = crc >> 8;

        mavlink_message_t rmsg;
        assert(parse_buffer(legacy, len, &rmsg, NULL, NULL) == MAVLINK_FRAMING_OK);
        assert(rmsg.sysid == 42);
        assert(mavlink_msg_get_target_sysid(&rmsg, entry) == 7);

        // the two frames differ only in compat_flags and CRC
        for (uint16_t k=0; k<len-2; k++) {
            if (k != 3) {
                assert(buf[k] == legacy[k]);
            }
        }
    }

    // MAVLink1 output with a 32 bit ID must error out, not truncate
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        memset(&msg, 0, sizeof(msg));
        memset(&status, 0, sizeof(status));
        status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        assert(mavlink_msg_command_long_pack_status(0x0A000001UL, 11, &status, &msg, 7, 250, 300, 1,
                                                    1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f) == 0);
        assert(status.parse_error == 1);
        assert(mavlink_msg_command_long_pack_status(42, 11, &status, &msg, 0x0A000002UL, 250, 300, 1,
                                                    1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f) == 0);
        assert(status.parse_error == 2);
    }

    // truncated extended header must not produce a false OK and the
    // parser must recover
    {
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(msg));
        mavlink_msg_command_long_pack(0x0A000001UL, 11, &msg, 0x0A000002UL, 250, 300, 1,
                                      1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        static mavlink_message_t rxmsg;
        static mavlink_status_t rstatus;
        mavlink_message_t out;
        mavlink_status_t status_out;
        memset(&rxmsg, 0, sizeof(rxmsg));
        memset(&rstatus, 0, sizeof(rstatus));
        unsigned ok_count = 0, bad_count = 0;
        // feed a frame truncated in the middle of the target block ...
        for (uint16_t k=0; k<14; k++) {
            assert(mavlink_frame_char_buffer(&rxmsg, &rstatus, buf[k], &out, &status_out) == MAVLINK_FRAMING_INCOMPLETE);
        }
        // ... then complete frames until one parses; the parser must
        // recover within a few frames and never return a false OK
        for (uint8_t attempt=0; attempt<4 && ok_count == 0; attempt++) {
            for (uint16_t k=0; k<len; k++) {
                uint8_t r = mavlink_frame_char_buffer(&rxmsg, &rstatus, buf[k], &out, &status_out);
                if (r == MAVLINK_FRAMING_OK) {
                    ok_count++;
                } else if (r == MAVLINK_FRAMING_BAD_CRC) {
                    bad_count++;
                }
            }
        }
        assert(ok_count > 0);
        assert(out.sysid == 0x0A000001UL);
    }

    printf("32 bit sysid tests OK\n");
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
static void test_sysid32_send(void)
{
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(MAVLINK_MSG_ID_COMMAND_LONG);
    static uint8_t sendbuf[MAVLINK_MAX_PACKET_LEN];
    const uint32_t old_sysid = mavlink_system.sysid;

    printf("Testing 32 bit sysid convenience send\n");

    mavlink_status_t *chan_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    chan_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    chan_status->signing = NULL;
    mavlink_system.sysid = 0x0A000001UL;

    capture_buf = sendbuf;
    capture_len = 0;
    mavlink_msg_command_long_send(MAVLINK_COMM_0, 0x0A000002UL, 250, 300, 1,
                                  1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
    capture_buf = NULL;
    assert(capture_len > 0);

    mavlink_message_t rmsg;
    assert(parse_buffer(sendbuf, capture_len, &rmsg, NULL, NULL) == MAVLINK_FRAMING_OK);
    assert(rmsg.sysid == 0x0A000001UL);
    assert((rmsg.incompat_flags & MAVLINK_IFLAG_SYSID32) != 0);
    assert((rmsg.incompat_flags & MAVLINK_IFLAG_TARGETTED) != 0);
    assert(mavlink_msg_get_target_sysid(&rmsg, entry) == 0x0A000002UL);
    assert(mavlink_msg_get_target_compid(&rmsg, entry) == 250);
    assert(mavlink_msg_command_long_get_command(&rmsg) == 300);

    // _mavlink_resend_uart must reproduce the frame byte for byte
    static uint8_t resendbuf[MAVLINK_MAX_PACKET_LEN];
    capture_buf = resendbuf;
    capture_len = 0;
    _mavlink_resend_uart(MAVLINK_COMM_0, &rmsg);
    capture_buf = NULL;
    assert(capture_len > 0);
    // seq/CRC are preserved by resend, so compare against the original
    uint16_t origlen = mavlink_msg_to_send_buffer(sendbuf, &rmsg);
    assert(capture_len == origlen);
    assert(memcmp(sendbuf, resendbuf, origlen) == 0);

    // MAVLink1 convenience send with 32 bit IDs must send nothing
    chan_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    const uint8_t parse_errors = chan_status->parse_error;
    capture_buf = sendbuf;
    capture_len = 0;
    mavlink_msg_command_long_send(MAVLINK_COMM_0, 7, 250, 300, 1,
                                  1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
    capture_buf = NULL;
    assert(capture_len == 0);
    assert(chan_status->parse_error == parse_errors+1);
    chan_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    mavlink_system.sysid = old_sysid;
    printf("32 bit sysid convenience send OK\n");
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

#ifdef MAVLINK_SIGNING_FLAG_SIGN_OUTGOING
static void test_sysid32_signing(void)
{
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(MAVLINK_MSG_ID_COMMAND_LONG);
    mavlink_signing_t tx_signing;
    mavlink_signing_t rx_signing;
    mavlink_signing_streams_t rx_streams;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    printf("Testing 32 bit sysid signing\n");

    memset(&tx_signing, 0, sizeof(tx_signing));
    tx_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    tx_signing.link_id = 0;
    tx_signing.timestamp = 1000;
    memset(tx_signing.secret_key, 42, sizeof(tx_signing.secret_key));

    memset(&rx_signing, 0, sizeof(rx_signing));
    rx_signing.timestamp = 1000;
    memset(rx_signing.secret_key, 42, sizeof(rx_signing.secret_key));

    mavlink_status_t status;
    memset(&status, 0, sizeof(status));
    status.signing = &tx_signing;

    mavlink_message_t msg;
    memset(&msg, 0, sizeof(msg));
    mavlink_msg_command_long_pack_status(0x0A000001UL, 11, &status, &msg, 0x0A000002UL, 250, 300, 1,
                                         1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
    assert(msg.incompat_flags == (MAVLINK_IFLAG_SIGNED|MAVLINK_IFLAG_SYSID32|MAVLINK_IFLAG_TARGETTED));
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    assert(len == MAVLINK_MAX_HEADER_LEN + msg.len + 2 + MAVLINK_SIGNATURE_BLOCK_LEN);

    mavlink_message_t rmsg;
    memset(&rx_streams, 0, sizeof(rx_streams));
    assert(parse_buffer(buf, len, &rmsg, &rx_signing, &rx_streams) == MAVLINK_FRAMING_OK);
    assert(rmsg.sysid == 0x0A000001UL);
    assert(mavlink_msg_get_target_sysid(&rmsg, entry) == 0x0A000002UL);

    // corrupting an extended header byte must give a CRC failure
    buf[7] ^= 0x40; // high byte of the 32 bit sysid
    memset(&rx_streams, 0, sizeof(rx_streams));
    assert(parse_buffer(buf, len, &rmsg, &rx_signing, &rx_streams) == MAVLINK_FRAMING_BAD_CRC);
    buf[7] ^= 0x40;

    // corrupting the signature must give a signature failure
    buf[len-1] ^= 0x40;
    memset(&rx_streams, 0, sizeof(rx_streams));
    assert(parse_buffer(buf, len, &rmsg, &rx_signing, &rx_streams) == MAVLINK_FRAMING_BAD_SIGNATURE);
    buf[len-1] ^= 0x40;

    printf("32 bit sysid signing OK\n");
}
#endif // MAVLINK_SIGNING_FLAG_SIGN_OUTGOING
#endif // MAVLINK_IFLAG_SYSID32

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
static const mavlink_message_info_t *dumb_search_info(const mavlink_message_info_t *msgs, uint32_t num_ids, uint32_t id)
{
    for (uint32_t i=0; i<num_ids; i++) {
        if (msgs[i].msgid == id) {
            return &msgs[i];
        }
    }
    return NULL;
}

static void test_get_message_info_by_id()
{
    const mavlink_message_info_t *msgs = mavlink_get_message_info_by_id(0);
    static const mavlink_msg_entry_t crcs[] = MAVLINK_MESSAGE_CRCS;
    const uint32_t num_msgs = sizeof(crcs)/sizeof(crcs[0]);
    for (uint32_t i=0; i<70000; i++) {
        const mavlink_message_info_t *m1 = mavlink_get_message_info_by_id(i);
        const mavlink_message_info_t *m2 = dumb_search_info(msgs, num_msgs, i);
        if (m1 != m2) {
            printf("Search error for id %u\n", (unsigned)i);
            error_count++;
        }
    }
}

static const mavlink_message_info_t *dumb_search_name(const mavlink_message_info_t *msgs, uint32_t num_ids, const char *name)
{
    for (uint32_t i=0; i<num_ids; i++) {
        if (strcmp(msgs[i].name, name) == 0) {
            return &msgs[i];
        }
    }
    return NULL;
}

static void test_get_message_info_by_name()
{
    static const char *test_names[] = { "HEARTBEAT", "STATUS_TEXT", "ATTITUDE", "FOOBLAH", "SILLY_NAME" };
    const uint8_t num_names = sizeof(test_names)/sizeof(test_names[0]);
    const mavlink_message_info_t *msgs = mavlink_get_message_info_by_id(0);
    static const mavlink_msg_entry_t crcs[] = MAVLINK_MESSAGE_CRCS;
    const uint32_t num_msgs = sizeof(crcs)/sizeof(crcs[0]);
    for (uint32_t i=0; i<num_names; i++) {
        const mavlink_message_info_t *m1 = mavlink_get_message_info_by_name(test_names[i]);
        const mavlink_message_info_t *m2 = dumb_search_name(msgs, num_msgs, test_names[i]);
        if (m1 != m2) {
            printf("Search error for id %s\n", test_names[i]);
            error_count++;
        }
    }
}


static const mavlink_msg_entry_t *dumb_search_entry(const mavlink_msg_entry_t *msgs, uint32_t num_ids, uint32_t id)
{
    for (uint32_t i=0; i<num_ids; i++) {
        if (msgs[i].msgid == id) {
            return &msgs[i];
        }
    }
    return NULL;
}

static void test_get_msg_entry()
{
    const mavlink_msg_entry_t *msgs = mavlink_get_msg_entry(0);
    static const mavlink_msg_entry_t crcs[] = MAVLINK_MESSAGE_CRCS;
    const uint32_t num_msgs = sizeof(crcs)/sizeof(crcs[0]);
    for (uint32_t i=0; i<70000; i++) {
        const mavlink_msg_entry_t *m1 = mavlink_get_msg_entry(i);
        const mavlink_msg_entry_t *m2 = dumb_search_entry(msgs, num_msgs, i);
        if (m1 != m2) {
            printf("Search error for entry id %u\n", (unsigned)i);
            error_count++;
        }
    }
}
#endif // MAVLINK_HAVE_GET_MESSAGE_INFO

int main(void)
{
	mavlink_channel_t chan;

        printf("Running mavlink_test_all\n");
	mavlink_test_all(11, 10, &last_msg);
	for (chan=MAVLINK_COMM_0; chan<=MAVLINK_COMM_1; chan++) {
		printf("Received %u messages on channel %u OK\n", 
		       chan_counts[chan], (unsigned)chan);
	}
	if (error_count != 0) {
		printf("Error count %u\n", error_count);
		exit(1);
	}
	printf("No errors detected\n");

        printf("Running mavlink_test_minimal\n");
        mavlink_test_minimal(11, 10, &last_msg);
	for (chan=MAVLINK_COMM_0; chan<=MAVLINK_COMM_1; chan++) {
		printf("Received %u messages on channel %u OK\n", 
		       chan_counts[chan], (unsigned)chan);
	}
	if (error_count != 0) {
		printf("Error count %u\n", error_count);
		exit(1);
	}
	printf("No errors detected\n");
        
#ifdef MAVLINK_SIGNING_FLAG_SIGN_OUTGOING

	printf("Testing signing\n");
	mavlink_signing_t signing;
	mavlink_signing_streams_t signing_streams;
        memset(&signing, 0, sizeof(signing));
        memset(&signing_streams, 0, sizeof(signing_streams));
	signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
	signing.link_id = 0;
	signing.timestamp = 1;
	memset(signing.secret_key, 42, sizeof(signing.secret_key));

  // 32 length uint8 signing.secret_key
	printf("signing_key = [ ");
	for (unsigned s=0; s<sizeof(signing.secret_key); s++) {
		if (s < 31) printf("%u, ", signing.secret_key[s]);
		if (s == 31) printf("%u ", signing.secret_key[s]);
	}
	printf("]\n");

    // we enable signing on channel 1 only, so the below loop alternately puts out non-signed and signed.
	statusp = mavlink_get_channel_status(MAVLINK_COMM_1);
	statusp->signing = &signing;
	statusp->signing_streams = &signing_streams;

	mavlink_test_all(11, 10, &last_msg);
	for (chan=MAVLINK_COMM_0; chan<=MAVLINK_COMM_1; chan++) {
		printf("Received %u messages on channel %u OK\n", 
		       chan_counts[chan], (unsigned)chan);
	}
	if (error_count != 0) {
		printf("Error count %u\n", error_count);
		exit(1);
	}
	printf("No errors detected\n");	
#endif

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        statusp = mavlink_get_channel_status(MAVLINK_COMM_0);
        statusp->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        statusp->signing = NULL;
        statusp = mavlink_get_channel_status(MAVLINK_COMM_1);
        statusp->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        statusp->signing = NULL;
        printf("Testing sending as MAVLink1\n");
        
	mavlink_test_all(11, 10, &last_msg);
	for (chan=MAVLINK_COMM_0; chan<=MAVLINK_COMM_1; chan++) {
		printf("Received %u messages on channel %u OK\n", 
		       chan_counts[chan], (unsigned)chan);
	}
	if (error_count != 0) {
		printf("Error count %u\n", error_count);
		exit(1);
	}
	printf("No errors detected\n");
#endif

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
        test_get_message_info_by_id();
        test_get_message_info_by_name();
        test_get_msg_entry();
#endif

#if defined(MAVLINK_IFLAG_SYSID32) && defined(MAVLINK_MSG_ID_COMMAND_LONG)
        test_sysid32();
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
        test_sysid32_send();
#endif
#ifdef MAVLINK_SIGNING_FLAG_SIGN_OUTGOING
        test_sysid32_signing();
#endif
	if (error_count != 0) {
		printf("Error count %u\n", error_count);
		exit(1);
	}
	printf("No errors detected\n");
#endif

	return 0;
}

