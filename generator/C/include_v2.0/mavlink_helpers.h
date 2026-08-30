#pragma once

#include "string.h"
#include "checksum.h"
#include "mavlink_types.h"
#include "mavlink_conversions.h"
#include <stdio.h>

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

#include "mavlink_sha256.h"

#ifndef MAVLINK_SIGNING_TIMESTAMP_LIMIT
#define MAVLINK_SIGNING_TIMESTAMP_LIMIT 60
#endif

#ifdef MAVLINK_USE_CXX_NAMESPACE
namespace mavlink {
#endif

/*
 * Internal function to give access to the channel status for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_STATUS
MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
#ifdef MAVLINK_EXTERNAL_RX_STATUS
	// No m_mavlink_status array defined in function,
	// has to be defined externally
#else
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
#endif
	return &m_mavlink_status[chan];
}
#endif

/*
 * Internal function to give access to the channel buffer for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_BUFFER
MAVLINK_HELPER mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
	
#ifdef MAVLINK_EXTERNAL_RX_BUFFER
	// No m_mavlink_buffer array defined in function,
	// has to be defined externally
#else
	static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
	return &m_mavlink_buffer[chan];
}
#endif // MAVLINK_GET_CHANNEL_BUFFER

/* Enable this option to check the length of each message.
    This allows invalid messages to be caught much sooner. Use if the transmission
    medium is prone to missing (or extra) characters (e.g. a radio that fades in
    and out). Only use if the channel will only contain messages types listed in
    the headers.
*/
//#define MAVLINK_CHECK_MESSAGE_LENGTH

/**
 * @brief Reset the status of a channel.
 */
MAVLINK_HELPER void mavlink_reset_channel_status(uint8_t chan)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	status->parse_state = MAVLINK_PARSE_STATE_IDLE;
}

#ifndef MAVLINK_NO_SIGN_PACKET
/**
 * @brief create a signature block for a packet
 */
MAVLINK_HELPER uint8_t mavlink_sign_packet(mavlink_signing_t *signing,
					   uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN],
					   const uint8_t *header, uint8_t header_len,
					   const uint8_t *packet, uint8_t packet_len,
					   const uint8_t crc[2])
{
	mavlink_sha256_ctx ctx;
	union {
	    uint64_t t64;
	    uint8_t t8[8];
	} tstamp;
	if (signing == NULL || !(signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING)) {
	    return 0;
	}
	signature[0] = signing->link_id;
	tstamp.t64 = signing->timestamp;
	memcpy(&signature[1], tstamp.t8, 6);
	signing->timestamp++;
	
	mavlink_sha256_init(&ctx);
	mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
	mavlink_sha256_update(&ctx, header, header_len);
	mavlink_sha256_update(&ctx, packet, packet_len);
	mavlink_sha256_update(&ctx, crc, 2);
	mavlink_sha256_update(&ctx, signature, 7);
	mavlink_sha256_final_48(&ctx, &signature[7]);
	
	return MAVLINK_SIGNATURE_BLOCK_LEN;
}
#endif

/**
 * @brief Trim payload of any trailing zero-populated bytes (MAVLink 2 only).
 *
 * @param payload Serialised payload buffer.
 * @param length Length of full-width payload buffer.
 * @return Length of payload after zero-filled bytes are trimmed.
 */
MAVLINK_HELPER uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
	while (length > 1 && payload[length-1] == 0) {
		length--;
	}
	return length;
}

/**
 * @brief pack a wire header from discrete header fields
 *
 * buf must be at least MAVLINK_MAX_HEADER_LEN bytes
 * @return header length in bytes, including the magic byte
 */
MAVLINK_HELPER uint8_t _mav_header_pack(uint8_t *buf, uint8_t magic, uint8_t len,
					uint8_t incompat_flags, uint8_t compat_flags,
					uint8_t seq, uint32_t sysid, uint8_t compid, uint32_t msgid,
					uint32_t target_sysid, uint8_t target_compid)
{
	buf[0] = magic;
	buf[1] = len;
	if (magic == MAVLINK_STX_MAVLINK1) {
		if (sysid > 255 || target_sysid > 255 || msgid > 255) {
			// 32 bit IDs cannot be represented in MAVLink1; never truncate
			return 0;
		}
		buf[2] = seq;
		buf[3] = sysid & 0xFF;
		buf[4] = compid;
		buf[5] = msgid & 0xFF;
		return MAVLINK_CORE_HEADER_MAVLINK1_LEN+1;
	}
	uint8_t ofs = 2;
	buf[ofs++] = incompat_flags;
	buf[ofs++] = compat_flags;
	buf[ofs++] = seq;
	buf[ofs++] = sysid & 0xFF;
	if (incompat_flags & MAVLINK_IFLAG_SYSID32) {
		buf[ofs++] = (sysid >> 8) & 0xFF;
		buf[ofs++] = (sysid >> 16) & 0xFF;
		buf[ofs++] = (sysid >> 24) & 0xFF;
	}
	buf[ofs++] = compid;
	buf[ofs++] = msgid & 0xFF;
	buf[ofs++] = (msgid >> 8) & 0xFF;
	buf[ofs++] = (msgid >> 16) & 0xFF;
	if (incompat_flags & MAVLINK_IFLAG_TARGETTED_SYSID32) {
		buf[ofs++] = target_sysid & 0xFF;
		buf[ofs++] = (target_sysid >> 8) & 0xFF;
		buf[ofs++] = (target_sysid >> 16) & 0xFF;
		buf[ofs++] = (target_sysid >> 24) & 0xFF;
		buf[ofs++] = target_compid;
	}
	return ofs;
}

/**
 * @brief serialize the wire header of a message, handling the
 * SYSID32 and TARGETTED extended headers
 *
 * buf must be at least MAVLINK_MAX_HEADER_LEN bytes
 * @return header length in bytes, including the magic byte
 */
MAVLINK_HELPER uint8_t mavlink_header_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
	return _mav_header_pack(buf, msg->magic, msg->len,
				msg->incompat_flags, msg->compat_flags,
				msg->seq, msg->sysid, msg->compid, msg->msgid,
				msg->target_sysid, msg->target_compid);
}

#ifndef MAVLINK_NO_SIGNATURE_CHECK
/**
 * @brief check a signature block for a packet
 */
MAVLINK_HELPER bool mavlink_signature_check(mavlink_signing_t *signing,
					    mavlink_signing_streams_t *signing_streams,
					    const mavlink_message_t *msg)
{
	if (signing == NULL) {
		return true;
	}
	// the message struct layout no longer matches the wire header, so
	// re-serialize the (possibly extended) header for the hash
	uint8_t header[MAVLINK_MAX_HEADER_LEN];
	const uint8_t header_len = mavlink_header_to_send_buffer(header, msg);
	const uint8_t *psig = msg->signature;
        const uint8_t *incoming_signature = psig+7;
	mavlink_sha256_ctx ctx;
	uint8_t signature[6];
	uint16_t i;

	mavlink_sha256_init(&ctx);
	mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
	mavlink_sha256_update(&ctx, header, header_len);
	mavlink_sha256_update(&ctx, _MAV_PAYLOAD(msg), msg->len);
	mavlink_sha256_update(&ctx, msg->ck, 2);
	mavlink_sha256_update(&ctx, psig, 1+6);
	mavlink_sha256_final_48(&ctx, signature);
        if (memcmp(signature, incoming_signature, 6) != 0) {
                signing->last_status = MAVLINK_SIGNING_STATUS_BAD_SIGNATURE;
		return false;
	}

	// now check timestamp
	union tstamp {
	    uint64_t t64;
	    uint8_t t8[8];
	} tstamp;
	uint8_t link_id = psig[0];
	tstamp.t64 = 0;
	memcpy(tstamp.t8, psig+1, 6);

	if (signing_streams == NULL) {
                signing->last_status = MAVLINK_SIGNING_STATUS_NO_STREAMS;
                return false;
	}
	
	// find stream
	for (i=0; i<signing_streams->num_signing_streams; i++) {
		if (msg->sysid == signing_streams->stream[i].sysid &&
		    msg->compid == signing_streams->stream[i].compid &&
		    link_id == signing_streams->stream[i].link_id) {
			break;
		}
	}
	if (i == signing_streams->num_signing_streams) {
		if (signing_streams->num_signing_streams >= MAVLINK_MAX_SIGNING_STREAMS) {
			// over max number of streams
                        signing->last_status = MAVLINK_SIGNING_STATUS_TOO_MANY_STREAMS;
                        return false;
		}
        // new stream. Only accept if timestamp is not more than 1 minute old by default
        if (tstamp.t64 + MAVLINK_SIGNING_TIMESTAMP_LIMIT*100000UL < signing->timestamp) {
                        signing->last_status = MAVLINK_SIGNING_STATUS_OLD_TIMESTAMP;
                        return false;
		}
		// add new stream
		signing_streams->stream[i].sysid = msg->sysid;
		signing_streams->stream[i].compid = msg->compid;
		signing_streams->stream[i].link_id = link_id;
		signing_streams->num_signing_streams++;
	} else {
		union tstamp last_tstamp;
		last_tstamp.t64 = 0;
		memcpy(last_tstamp.t8, signing_streams->stream[i].timestamp_bytes, 6);
		if (tstamp.t64 <= last_tstamp.t64) {
			// repeating old timestamp
                        signing->last_status = MAVLINK_SIGNING_STATUS_REPLAY;
                        return false;
		}
	}

	// remember last timestamp
	memcpy(signing_streams->stream[i].timestamp_bytes, psig+1, 6);

	// our next timestamp must be at least this timestamp
	if (tstamp.t64 > signing->timestamp) {
		signing->timestamp = tstamp.t64;
	}
        signing->last_status = MAVLINK_SIGNING_STATUS_OK;
        return true;
}
#endif


/** Record a parse or output framing error. */
static inline void _mav_parse_error(mavlink_status_t *status)
{
    status->parse_error++;
}

/**
 * @brief Finalize a MAVLink message with channel assignment
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. This function
 * can also be used if the message header has already been written before (as in mavlink_msg_xxx_pack
 * instead of mavlink_msg_xxx_pack_headerless), it just introduces little extra overhead.
 *
 * @param msg Message to finalize
 * @param system_id Id of the sending (this) system, 1-127
 * @param length Message length
 */
MAVLINK_HELPER uint16_t mavlink_finalize_message_buffer_target(mavlink_message_t* msg, uint32_t system_id, uint8_t component_id,
						      mavlink_status_t* status, uint8_t min_length, uint8_t length, uint8_t crc_extra,
						      uint32_t target_sysid, uint8_t target_compid)
{
	bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
#ifndef MAVLINK_NO_SIGN_PACKET
	bool signing = 	(!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);
#else
	bool signing = false;
#endif
	uint8_t signature_len = signing? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
	uint8_t buf[MAVLINK_MAX_HEADER_LEN];
	if (mavlink1) {
		if (system_id > 255 || target_sysid > 255 || msg->msgid > 255) {
			// 32 bit IDs and extended message IDs cannot be represented in MAVLink1. Never
			// truncate: a masked sysid would alias another system
			// and a zeroed target would become a broadcast
			_mav_parse_error(status);
			return 0;
		}
		msg->magic = MAVLINK_STX_MAVLINK1;
	} else {
		msg->magic = MAVLINK_STX;
	}
	msg->len = mavlink1?min_length:_mav_trim_payload(_MAV_PAYLOAD(msg), length);
	msg->sysid = system_id;
	msg->compid = component_id;
	msg->incompat_flags = 0;
	msg->target_sysid = 0;
	msg->target_compid = 0;
	if (signing) {
		msg->incompat_flags |= MAVLINK_IFLAG_SIGNED;
	}
	if (!mavlink1) {
		if (system_id > 255) {
			msg->incompat_flags |= MAVLINK_IFLAG_SYSID32;
		}
		if (target_sysid > 255) {
			msg->incompat_flags |= MAVLINK_IFLAG_TARGETTED_SYSID32;
			msg->target_sysid = target_sysid;
			msg->target_compid = target_compid;
		}
	}
	// advertise that we understand 32 bit system IDs
	msg->compat_flags = mavlink1?0:MAVLINK_CFLAG_SYSID32;
	msg->seq = status->current_tx_seq;
	status->current_tx_seq = status->current_tx_seq + 1;

	// form the header as a byte array for the crc
	const uint8_t header_len = mavlink_header_to_send_buffer(buf, msg);

	uint16_t checksum = crc_calculate(&buf[1], header_len-1);
	crc_accumulate_buffer(&checksum, _MAV_PAYLOAD(msg), msg->len);
	crc_accumulate(crc_extra, &checksum);
	mavlink_ck_a(msg) = (uint8_t)(checksum & 0xFF);
	mavlink_ck_b(msg) = (uint8_t)(checksum >> 8);

	msg->checksum = checksum;

#ifndef MAVLINK_NO_SIGN_PACKET
	if (signing) {
		mavlink_sign_packet(status->signing,
				    msg->signature,
				    (const uint8_t *)buf, header_len,
				    (const uint8_t *)_MAV_PAYLOAD(msg), msg->len,
				    (const uint8_t *)_MAV_PAYLOAD(msg)+(uint16_t)msg->len);
	}
#endif

	return msg->len + header_len + 2 + signature_len;
}

MAVLINK_HELPER uint16_t mavlink_finalize_message_buffer(mavlink_message_t* msg, uint32_t system_id, uint8_t component_id,
						      mavlink_status_t* status, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
	return mavlink_finalize_message_buffer_target(msg, system_id, component_id, status, min_length, length, crc_extra, 0, 0);
}

MAVLINK_HELPER uint16_t mavlink_finalize_message_chan_target(mavlink_message_t* msg, uint32_t system_id, uint8_t component_id,
						      uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra,
						      uint32_t target_sysid, uint8_t target_compid)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	return mavlink_finalize_message_buffer_target(msg, system_id, component_id, status, min_length, length, crc_extra,
						      target_sysid, target_compid);
}

MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint32_t system_id, uint8_t component_id,
						      uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
	return mavlink_finalize_message_chan_target(msg, system_id, component_id, chan, min_length, length, crc_extra, 0, 0);
}

/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 */
MAVLINK_HELPER uint16_t mavlink_finalize_message_target(mavlink_message_t* msg, uint32_t system_id, uint8_t component_id,
						 uint8_t min_length, uint8_t length, uint8_t crc_extra,
						 uint32_t target_sysid, uint8_t target_compid)
{
    return mavlink_finalize_message_chan_target(msg, system_id, component_id, MAVLINK_COMM_0, min_length, length, crc_extra,
						target_sysid, target_compid);
}

MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint32_t system_id, uint8_t component_id,
						 uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, min_length, length, crc_extra);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);

/**
 * @brief Finalize a MAVLink message with channel assignment and send
 */
MAVLINK_HELPER void _mav_finalize_message_chan_send_target(mavlink_channel_t chan, uint32_t msgid,
                                                    const char *packet,
						    uint8_t min_length, uint8_t length, uint8_t crc_extra,
						    uint32_t target_sysid, uint8_t target_compid)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_MAX_HEADER_LEN];
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	uint8_t header_len;
	uint8_t signature_len = 0;
	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
	bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
	bool signing = 	(!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);
	uint8_t incompat_flags = 0;

        if (mavlink1) {
            length = min_length;
            if (msgid > 255 || mavlink_system.sysid > 255 || target_sysid > 255) {
                // can't send 16 bit msgids or 32 bit IDs in MAVLink1
                _mav_parse_error(status);
                return;
            }
        } else {
	    if (signing) {
		incompat_flags |= MAVLINK_IFLAG_SIGNED;
	    }
	    if (mavlink_system.sysid > 255) {
		incompat_flags |= MAVLINK_IFLAG_SYSID32;
	    }
	    if (target_sysid > 255) {
		incompat_flags |= MAVLINK_IFLAG_TARGETTED_SYSID32;
	    }
            length = _mav_trim_payload(packet, length);
        }
	header_len = _mav_header_pack(buf,
				      mavlink1?MAVLINK_STX_MAVLINK1:MAVLINK_STX, length,
				      incompat_flags, mavlink1?0:MAVLINK_CFLAG_SYSID32,
				      status->current_tx_seq,
				      mavlink_system.sysid, mavlink_system.compid, msgid,
				      target_sysid, target_compid);
	status->current_tx_seq++;
	checksum = crc_calculate((const uint8_t*)&buf[1], header_len-1);
	crc_accumulate_buffer(&checksum, packet, length);
	crc_accumulate(crc_extra, &checksum);
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

#ifndef MAVLINK_NO_SIGN_PACKET
	if (signing) {
		// possibly add a signature
		signature_len = mavlink_sign_packet(status->signing, signature, buf, header_len,
						    (const uint8_t *)packet, length, ck);
	}
#endif

	MAVLINK_START_UART_SEND(chan, header_len + 2 + (uint16_t)length + (uint16_t)signature_len);
	_mavlink_send_uart(chan, (const char *)buf, header_len);
	_mavlink_send_uart(chan, packet, length);
	_mavlink_send_uart(chan, (const char *)ck, 2);
	if (signature_len != 0) {
		_mavlink_send_uart(chan, (const char *)signature, signature_len);
	}
	MAVLINK_END_UART_SEND(chan, header_len + 2 + (uint16_t)length + (uint16_t)signature_len);
}

MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint32_t msgid,
                                                    const char *packet,
						    uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
	_mav_finalize_message_chan_send_target(chan, msgid, packet, min_length, length, crc_extra, 0, 0);
}

/**
 * @brief re-send a message over a uart channel
 * this is more stack efficient than re-marshalling the message
 * If the message is signed then the original signature is also sent
 */
MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg)
{
	uint8_t ck[2];

	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);
	// XXX use the right sequence here

        // we can't send the structure directly as its layout does not match the wire header
        uint8_t buf[MAVLINK_MAX_HEADER_LEN];
        const uint8_t header_len = mavlink_header_to_send_buffer(buf, msg);
        if (header_len == 0) {
                // unrepresentable (32 bit IDs in a MAVLink1 message)
                return;
        }
        uint8_t signature_len;

        if (msg->magic == MAVLINK_STX_MAVLINK1) {
            signature_len = 0;
        } else {
            signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
        }
        MAVLINK_START_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
        _mavlink_send_uart(chan, (const char *)buf, header_len);
	_mavlink_send_uart(chan, _MAV_PAYLOAD(msg), msg->len);
	_mavlink_send_uart(chan, (const char *)ck, 2);
        if (signature_len != 0) {
	    _mavlink_send_uart(chan, (const char *)msg->signature, MAVLINK_SIGNATURE_BLOCK_LEN);
        }
        MAVLINK_END_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a message to send it over a serial byte stream
 */
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
	uint8_t signature_len;
	uint8_t *ck;
        uint8_t length = msg->len;
        const uint8_t header_len = mavlink_header_to_send_buffer(buf, msg);
        if (header_len == 0) {
                // unrepresentable (32 bit IDs in a MAVLink1 message)
                return 0;
        }

	if (msg->magic == MAVLINK_STX_MAVLINK1) {
		signature_len = 0;
	} else {
		length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);
		buf[1] = length;
		signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
	}
	memcpy(&buf[header_len], _MAV_PAYLOAD(msg), length);
	ck = buf + header_len + (uint16_t)length;
	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);
	if (signature_len > 0) {
		memcpy(&ck[2], msg->signature, signature_len);
	}

	return header_len + 2 + (uint16_t)length + (uint16_t)signature_len;
}

union __mavlink_bitfield {
	uint8_t uint8;
	int8_t int8;
	uint16_t uint16;
	int16_t int16;
	uint32_t uint32;
	int32_t int32;
};


MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg)
{
	uint16_t crcTmp = 0;
	crc_init(&crcTmp);
	msg->checksum = crcTmp;
}

MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
	uint16_t checksum = msg->checksum;
	crc_accumulate(c, &checksum);
	msg->checksum = checksum;
}

/*
  return the crc_entry value for a msgid
*/
#ifndef MAVLINK_GET_MSG_ENTRY
MAVLINK_HELPER const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid)
{
	static const mavlink_msg_entry_t mavlink_message_crcs[] = MAVLINK_MESSAGE_CRCS;
        /*
	  use a bisection search to find the right entry. A perfect hash may be better
	  Note that this assumes the table is sorted by msgid
	*/
        uint32_t low=0, high=sizeof(mavlink_message_crcs)/sizeof(mavlink_message_crcs[0]) - 1;
        while (low < high) {
            uint32_t mid = (low+1+high)/2;
            if (msgid < mavlink_message_crcs[mid].msgid) {
                high = mid-1;
                continue;
            }
            if (msgid > mavlink_message_crcs[mid].msgid) {
                low = mid;
                continue;
            }
            low = mid;
            break;
        }
        if (mavlink_message_crcs[low].msgid != msgid) {
            // msgid is not in the table
            return NULL;
        }
        return &mavlink_message_crcs[low];
}
#endif // MAVLINK_GET_MSG_ENTRY

/*
  return the crc_extra value for a message
*/
MAVLINK_HELPER uint8_t mavlink_get_crc_extra(const mavlink_message_t *msg)
{
	const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
	return e?e->crc_extra:0;
}

/*
  return the min message length
*/
#define MAVLINK_HAVE_MIN_MESSAGE_LENGTH
MAVLINK_HELPER uint8_t mavlink_min_message_length(const mavlink_message_t *msg)
{
	const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
        return e?e->min_msg_len:0;
}

/*
  return the max message length (including extensions)
*/
#define MAVLINK_HAVE_MAX_MESSAGE_LENGTH
MAVLINK_HELPER uint8_t mavlink_max_message_length(const mavlink_message_t *msg)
{
	const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
        return e?e->max_msg_len:0;
}

/*
  return the target system ID of a message, taking the TARGETTED
  extended header into account. Returns 0 (broadcast) if the message
  has no target. The entry may be NULL
*/
#define MAVLINK_HAVE_GET_TARGET_SYSID
MAVLINK_HELPER uint32_t mavlink_msg_get_target_sysid(const mavlink_message_t *msg, const mavlink_msg_entry_t *entry)
{
	if (msg->incompat_flags & MAVLINK_IFLAG_TARGETTED_SYSID32) {
		return msg->target_sysid;
	}
	if (entry == NULL || !(entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)) {
		return 0;
	}
	if (entry->target_system_ofs >= msg->len) {
		// zero-trimmed
		return 0;
	}
	return (uint8_t)_MAV_PAYLOAD(msg)[entry->target_system_ofs];
}

/*
  return the target component ID of a message, taking the TARGETTED
  extended header into account. Returns 0 (broadcast) if the message
  has no target component. The entry may be NULL
*/
MAVLINK_HELPER uint8_t mavlink_msg_get_target_compid(const mavlink_message_t *msg, const mavlink_msg_entry_t *entry)
{
	if (msg->incompat_flags & MAVLINK_IFLAG_TARGETTED_SYSID32) {
		return msg->target_compid;
	}
	if (entry == NULL || !(entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)) {
		return 0;
	}
	if (entry->target_component_ofs >= msg->len) {
		// zero-trimmed
		return 0;
	}
	return (uint8_t)_MAV_PAYLOAD(msg)[entry->target_component_ofs];
}

/**
 * This is a variant of mavlink_frame_char() but with caller supplied
 * parsing buffers. It is useful when you want to create a MAVLink
 * parser in a library that doesn't use any global variables
 *
 * @param rxmsg    parsing message buffer
 * @param status   parsing status buffer
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 */
MAVLINK_HELPER uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg, 
                                                 mavlink_status_t* status,
                                                 uint8_t c, 
                                                 mavlink_message_t* r_message, 
                                                 mavlink_status_t* r_mavlink_status)
{

	status->msg_received = MAVLINK_FRAMING_INCOMPLETE;

	switch (status->parse_state)
	{
	case MAVLINK_PARSE_STATE_UNINIT:
	case MAVLINK_PARSE_STATE_IDLE:
		if (c == MAVLINK_STX)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;
                        status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
			mavlink_start_checksum(rxmsg);
		} else if (c == MAVLINK_STX_MAVLINK1)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;
                        status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
			mavlink_start_checksum(rxmsg);
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_STX:
			if (status->msg_received 
/* Support shorter buffers than the
   default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
				|| c > MAVLINK_MAX_PAYLOAD_LEN
#endif
				)
		{
			status->buffer_overrun++;
			_mav_parse_error(status);
			status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
		}
		else
		{
			// NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
			rxmsg->len = c;
			status->packet_idx = 0;
			mavlink_update_checksum(rxmsg, c);
                        if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
                            rxmsg->incompat_flags = 0;
                            rxmsg->compat_flags = 0;
                            // MAVLink1 skips the GOT_LENGTH state, so clear the
                            // target IDs here too; a stale target from a previous
                            // frame would make this one unrepresentable on send
                            rxmsg->target_sysid = 0;
                            rxmsg->target_compid = 0;
                            status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
                        } else {
                            status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
                        }
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_LENGTH:
		rxmsg->incompat_flags = c;
		if ((rxmsg->incompat_flags & ~MAVLINK_IFLAG_MASK) != 0) {
			// message includes an incompatible feature flag
			_mav_parse_error(status);
			status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			break;
		}
		if (!(rxmsg->incompat_flags & MAVLINK_IFLAG_TARGETTED_SYSID32)) {
			// don't leave stale target IDs in re-used buffers
			rxmsg->target_sysid = 0;
			rxmsg->target_compid = 0;
		}
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
		break;

	case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
		rxmsg->compat_flags = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
		rxmsg->seq = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
		break;
                
	case MAVLINK_PARSE_STATE_GOT_SEQ:
		rxmsg->sysid = c;
		mavlink_update_checksum(rxmsg, c);
		if (rxmsg->incompat_flags & MAVLINK_IFLAG_SYSID32) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID1;
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID1:
		rxmsg->sysid |= ((uint32_t)c) << 8;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID2;
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID2:
		rxmsg->sysid |= ((uint32_t)c) << 16;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID3;
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID3:
		rxmsg->sysid |= ((uint32_t)c) << 24;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID:
		rxmsg->compid = c;
		mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPID:
		rxmsg->msgid = c;
		mavlink_update_checksum(rxmsg, c);
		if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
			if(rxmsg->len > 0) {
				status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
			} else {
				status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
			}
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
			if (rxmsg->len < mavlink_min_message_length(rxmsg) ||
				rxmsg->len > mavlink_max_message_length(rxmsg)) {
				_mav_parse_error(status);
				status->parse_state = MAVLINK_PARSE_STATE_IDLE;
				break;
			}
#endif
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID1:
		rxmsg->msgid |= ((uint32_t)c)<<8;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID2:
		rxmsg->msgid |= ((uint32_t)c)<<16;
		mavlink_update_checksum(rxmsg, c);
		if (rxmsg->incompat_flags & MAVLINK_IFLAG_TARGETTED_SYSID32) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3_TARGETTED;
		} else if(rxmsg->len > 0){
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
        if (rxmsg->len < mavlink_min_message_length(rxmsg) ||
            rxmsg->len > mavlink_max_message_length(rxmsg))
        {
			_mav_parse_error(status);
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			break;
        }
#endif
		break;
                
	case MAVLINK_PARSE_STATE_GOT_MSGID3_TARGETTED:
		rxmsg->target_sysid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_TARGET_SYSID0;
		break;

	case MAVLINK_PARSE_STATE_GOT_TARGET_SYSID0:
		rxmsg->target_sysid |= ((uint32_t)c) << 8;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_TARGET_SYSID1;
		break;

	case MAVLINK_PARSE_STATE_GOT_TARGET_SYSID1:
		rxmsg->target_sysid |= ((uint32_t)c) << 16;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_TARGET_SYSID2;
		break;

	case MAVLINK_PARSE_STATE_GOT_TARGET_SYSID2:
		rxmsg->target_sysid |= ((uint32_t)c) << 24;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_TARGET_SYSID3;
		break;

	case MAVLINK_PARSE_STATE_GOT_TARGET_SYSID3:
		rxmsg->target_compid = c;
		mavlink_update_checksum(rxmsg, c);
		if (rxmsg->len > 0) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID3:
		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
		mavlink_update_checksum(rxmsg, c);
		if (status->packet_idx == rxmsg->len)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_PAYLOAD: {
		const mavlink_msg_entry_t *e = mavlink_get_msg_entry(rxmsg->msgid);
		if (e == NULL) {
			// Message not found in CRC_EXTRA table.
			status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
			rxmsg->ck[0] = c;
		} else {
			uint8_t crc_extra = e->crc_extra;
			mavlink_update_checksum(rxmsg, crc_extra);
			if (c != (rxmsg->checksum & 0xFF)) {
				status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
			} else {
				status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
			}
			rxmsg->ck[0] = c;

			// zero-fill the packet to cope with short incoming packets.
			// Clamp to MAVLINK_MAX_PAYLOAD_LEN: e->max_msg_len comes from the
			// message table and can exceed the payload buffer size when the
			// buffer has been shrunk via MAVLINK_MAX_PAYLOAD_LEN, which would
			// otherwise overflow the buffer.
				uint8_t fill_len = e->max_msg_len < MAVLINK_MAX_PAYLOAD_LEN ? e->max_msg_len : MAVLINK_MAX_PAYLOAD_LEN;
				if (e && status->packet_idx < fill_len) {
					memset(&_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx], 0, fill_len - status->packet_idx);
			}
		}
		break;
        }

	case MAVLINK_PARSE_STATE_GOT_CRC1:
	case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
		if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8)) {
			// got a bad CRC message
			status->msg_received = MAVLINK_FRAMING_BAD_CRC;
		} else {
			// Successfully got message
			status->msg_received = MAVLINK_FRAMING_OK;
		}
		rxmsg->ck[1] = c;

		if (rxmsg->incompat_flags & MAVLINK_IFLAG_SIGNED) {
			if (status->msg_received == MAVLINK_FRAMING_BAD_CRC) {
			    // If the CRC is already wrong, don't overwrite msg_received,
			    // otherwise we can end up with garbage flagged as valid.
			    status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT_BAD_CRC;
			} else {
			    status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;
			    status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
			}
			status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;
		} else {
			if (status->signing &&
			   	(status->signing->accept_unsigned_callback == NULL ||
			   	 !status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {

				// If the CRC is already wrong, don't overwrite msg_received.
				if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) {
					status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
				}
			}
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (r_message != NULL) {
				memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
			}
		}
		break;
	case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
	case MAVLINK_PARSE_STATE_SIGNATURE_WAIT_BAD_CRC:
		rxmsg->signature[MAVLINK_SIGNATURE_BLOCK_LEN-status->signature_wait] = c;
		status->signature_wait--;
		if (status->signature_wait == 0) {
			// we have the whole signature, check it is OK
#ifndef MAVLINK_NO_SIGNATURE_CHECK
			bool sig_ok = mavlink_signature_check(status->signing, status->signing_streams, rxmsg);
#else
			bool sig_ok = true;
#endif
			if (!sig_ok &&
			   	(status->signing->accept_unsigned_callback &&
			   	 status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {
				// accepted via application level override
				sig_ok = true;
			}
			if (status->parse_state == MAVLINK_PARSE_STATE_SIGNATURE_WAIT_BAD_CRC) {
			    status->msg_received = MAVLINK_FRAMING_BAD_CRC;
			} else if (sig_ok) {
			    status->msg_received = MAVLINK_FRAMING_OK;
			} else {
			    status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
			}
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (r_message !=NULL) {
				memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
			}
		}
		break;
	}

	// If a message has been successfully decoded, check index
	if (status->msg_received == MAVLINK_FRAMING_OK)
	{
		//while(status->current_seq != rxmsg->seq)
		//{
		//	status->packet_rx_drop_count++;
		//               status->current_seq++;
		//}
		status->current_rx_seq = rxmsg->seq;
		// Initial condition: If no packet has been received so far, drop count is undefined
		if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;
		// Count this packet as received
		status->packet_rx_success_count++;
	}

       if (r_message != NULL) {
           r_message->len = rxmsg->len; // Provide visibility on how far we are into current msg
       }
       if (r_mavlink_status != NULL) {	
           r_mavlink_status->parse_state = status->parse_state;
           r_mavlink_status->packet_idx = status->packet_idx;
           r_mavlink_status->current_rx_seq = status->current_rx_seq+1;
           r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
           r_mavlink_status->packet_rx_drop_count = status->parse_error;
           r_mavlink_status->flags = status->flags;
       }
       status->parse_error = 0;

	if (status->msg_received == MAVLINK_FRAMING_BAD_CRC) {
		/*
		  the CRC came out wrong. We now need to overwrite the
		  msg CRC with the one on the wire so that if the
		  caller decides to forward the message anyway that
		  mavlink_msg_to_send_buffer() won't overwrite the
		  checksum
		 */
            if (r_message != NULL) {
                r_message->checksum = rxmsg->ck[0] | (rxmsg->ck[1]<<8);
            }
	}

	return status->msg_received;
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0, 1 or
 * 2 (MAVLINK_FRAMING_INCOMPLETE, MAVLINK_FRAMING_OK or MAVLINK_FRAMING_BAD_CRC)
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_frame_char(chan, byte, &msg, &status) != MAVLINK_FRAMING_INCOMPLETE)
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
MAVLINK_HELPER uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
	return mavlink_frame_char_buffer(mavlink_get_channel_buffer(chan),
					 mavlink_get_channel_status(chan),
					 c,
					 r_message,
					 r_mavlink_status);
}

/**
 * Set the protocol version
 */
MAVLINK_HELPER void mavlink_set_proto_version(uint8_t chan, unsigned int version)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	if (version > 1) {
		status->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
	} else {
		status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
	}
}

/**
 * Get the protocol version
 *
 * @return 1 for v1, 2 for v2
 */
MAVLINK_HELPER unsigned int mavlink_get_proto_version(uint8_t chan)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) > 0) {
		return 1;
	} else {
		return 2;
	}
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0 or 1.
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded or bad CRC, 1 on good message and CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_parse_char(chan, byte, &msg, &status))
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
MAVLINK_HELPER uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char(chan, c, r_message, r_mavlink_status);
    if (msg_received == MAVLINK_FRAMING_BAD_CRC ||
	msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
	    // we got a bad CRC. Treat as a parse failure
	    mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan);
	    mavlink_status_t* status = mavlink_get_channel_status(chan);
	    _mav_parse_error(status);
	    status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
	    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
	    if (c == MAVLINK_STX)
	    {
		    status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
		    rxmsg->len = 0;
		    mavlink_start_checksum(rxmsg);
	    }
	    return 0;
    }
    return msg_received;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

// To make MAVLink work on your MCU, define comm_send_ch() if you wish
// to send 1 byte at a time, or MAVLINK_SEND_UART_BYTES() to send a
// whole packet at a time

/*

#include "mavlink_types.h"

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
        uart0_transmit(ch);
    }
    if (chan == MAVLINK_COMM_1)
    {
    	uart1_transmit(ch);
    }
}
 */

MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len)
{
#ifdef MAVLINK_SEND_UART_BYTES
	/* this is the more efficient approach, if the platform
	   defines it */
	MAVLINK_SEND_UART_BYTES(chan, (const uint8_t *)buf, len);
#else
	/* fallback to one byte at a time */
	uint16_t i;
	for (i = 0; i < len; i++) {
		comm_send_ch(chan, (uint8_t)buf[i]);
	}
#endif
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

#ifdef MAVLINK_USE_CXX_NAMESPACE
} // namespace mavlink
#endif
