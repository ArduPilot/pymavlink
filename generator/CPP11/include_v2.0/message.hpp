
#pragma once

#include <array>
#include <cassert>
#include <cstring>
#include <sstream>
#include <string>
#include <iostream>

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER static inline
#endif

/*
  Opt-in compile-time diagnostics for MAVLink messages and MAV_CMD/enum
  entries flagged in the XML as work-in-progress, deprecated or superseded.
  Kept in sync with the equivalent block in protocol.h (not #include-d here,
  since its typed declarations assume the un-namespaced C build). See
  protocol.h for usage/severity guidance. These are function-only macros:
  they never annotate the C++11 message struct itself (see
  MAVLINK_MSG_TYPE_* below for that), but they do still apply to the plain
  C pack/encode/send/decode helper functions that TEST_INTEROP builds pull
  in from mavlink.h alongside this header.
*/
#ifndef MAVLINK_WIP
#define MAVLINK_WIP
#endif
#ifndef MAVLINK_DEPRECATED
#define MAVLINK_DEPRECATED
#endif
#ifndef MAVLINK_SUPERSEDED
#define MAVLINK_SUPERSEDED
#endif
#ifndef MAVLINK_ENUM_WIP
#define MAVLINK_ENUM_WIP
#endif
#ifndef MAVLINK_ENUM_DEPRECATED
#define MAVLINK_ENUM_DEPRECATED
#endif
#ifndef MAVLINK_ENUM_SUPERSEDED
#define MAVLINK_ENUM_SUPERSEDED
#endif

/*
  Type-only counterparts of the three MAVLINK_* macros above, applied to
  the C++11 message struct itself (e.g. "struct MAVLINK_MSG_TYPE_WIP Ping
  : mavlink::Message"). A struct/class is not a function, so GCC's
  function-only warning()/error() attributes cannot be used here -- as
  with MAVLINK_ENUM_WIP/DEPRECATED/SUPERSEDED, only deprecated/unavailable
  (or the standard [[deprecated("...")]]) are valid, e.g.:

    // unavailable() requires clang or GCC >= 12 (see protocol.h): guard it,
    // or just use deprecated() unconditionally if you don't need the harder
    // failure and want to support older GCC too.
    #if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
    #define MAVLINK_MSG_TYPE_WIP         __attribute__((unavailable("MAVLink WIP message used")))
    #else
    #define MAVLINK_MSG_TYPE_WIP         __attribute__((deprecated("MAVLink WIP message used")))
    #endif
    #define MAVLINK_MSG_TYPE_DEPRECATED  __attribute__((deprecated("MAVLink deprecated message used")))
    #define MAVLINK_MSG_TYPE_SUPERSEDED  // leave undefined/empty: still fully supported
*/
#ifndef MAVLINK_MSG_TYPE_WIP
#define MAVLINK_MSG_TYPE_WIP
#endif
#ifndef MAVLINK_MSG_TYPE_DEPRECATED
#define MAVLINK_MSG_TYPE_DEPRECATED
#endif
#ifndef MAVLINK_MSG_TYPE_SUPERSEDED
#define MAVLINK_MSG_TYPE_SUPERSEDED
#endif

/*
  Generated gtestsuite.hpp instantiates every message type, including
  flagged ones, to round-trip test it - that self-reference must not
  itself warn/error under MAVLINK_MSG_TYPE_DEPRECATED/_SUPERSEDED, the
  same reasoning and mechanism as MAVLINK_DEPRECATED_CALL_BEGIN/END in
  protocol.h. This only silences the diagnostic for that one internal
  use; a real caller instantiating the type directly still gets it.
*/
#ifndef MAVLINK_DEPRECATED_CALL_BEGIN
# if defined(__GNUC__) || defined(__clang__)
#  define MAVLINK_DEPRECATED_CALL_BEGIN \
    _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
# else
#  define MAVLINK_DEPRECATED_CALL_BEGIN
# endif
#endif
#ifndef MAVLINK_DEPRECATED_CALL_END
# if defined(__GNUC__) || defined(__clang__)
#  define MAVLINK_DEPRECATED_CALL_END _Pragma("GCC diagnostic pop")
# else
#  define MAVLINK_DEPRECATED_CALL_END
# endif
#endif

#define MAVLINK_USE_CXX_NAMESPACE	// put C-lib into namespace
#include "mavlink_types.h"

#define  _MAVLINK_CONVERSIONS_H_	// do not include mavlink_conversions.h
#define MAVLINK_GET_MSG_ENTRY		// user should provide mavlink_get_msg_entry()
namespace mavlink {
/**
 * Return message entry data for msgid.
 *
 * @note user of MAVLink library should provide
 *       implementation for this function.
 *       Use mavlink::<dialect-name>::MESSAGE_ENTRIES array to make hash map.
 *
 * @returns nullptr  if message is unknown
 */
const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid);
} // namespace mavlink

#include "mavlink_helpers.h"

#include "msgmap.hpp"

namespace mavlink {

//! Message ID type
using msgid_t = uint32_t;

/**
 * MAVLink Message base class.
 */
struct Message {
	static constexpr msgid_t MSG_ID = UINT32_MAX;
	static constexpr size_t LENGTH = 0;
	static constexpr size_t MIN_LENGTH = 0;
	static constexpr uint8_t CRC_EXTRA = 0;
	static constexpr auto NAME = "BASE";

	struct Info {
		msgid_t id;
		size_t length;
		size_t min_length;
		uint8_t crc_extra;
	};

	/**
	 * Get NAME constant. Helper for overloaded class access.
	 */
	virtual std::string get_name(void) const = 0;

	/**
	 * Get info needed for mavlink_finalize_message_xxx()
	 */
	virtual Info get_message_info(void) const = 0;

	/**
	 * Make YAML-string from message content.
	 */
	virtual std::string to_yaml(void) const = 0;

	/**
	 * Serialize message.
	 *
	 * @param[out] map
	 */
	virtual void serialize(MsgMap &map) const = 0;

	/**
	 * Deserialize message.
	 *
	 * @param[in] map
	 */
	virtual void deserialize(MsgMap &msp) = 0;
};

/**
 * Converts std::array<char, N> to std::string.
 *
 * Array treated as null-terminated string up to _N chars.
 */
template<size_t _N>
std::string to_string(const std::array<char, _N> &a)
{
	return std::string(a.data(), strnlen(a.data(), a.size()));
}

/**
 * Convert std::array to comma separated string
 */
template<typename _T, size_t _N>
std::string to_string(const std::array<_T, _N> &a)
{
	std::stringstream ss;
	bool first = true;

	for (auto const &v : a) {
		if (first) {
			first = false;
		} else {
			ss << ", ";
		}

		// +v treated as 0+v, it's safe for all types,
		// but force int8_t/uint8_t to be print as number.
		ss << +v;
	}

	return ss.str();
}

/**
 * Set std::string value to std::array<char, N> (may be not null-terminated)
 *
 * @param[out] a
 * @param[in]  s
 */
template<size_t _N>
void set_string(std::array<char, _N> &a, const std::string &s)
{
	strncpy(a.data(), s.c_str(), a.size());
}

/**
 * Set std::string value to std::array<char, N> (always null-terminated)
 *
 * @param[out] a
 * @param[in]  s
 */
template<size_t _N>
void set_string_z(std::array<char, _N> &a, const std::string &s)
{
	strncpy(a.data(), s.c_str(), a.size() - 1);
	a[a.size() - 1] = '\0';
}

} // namespace mavlink
