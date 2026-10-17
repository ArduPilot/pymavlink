/** @file
 *    @brief MAVLink comm protocol testsuite generated from test.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef TEST_TESTSUITE_H
#define TEST_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_test(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_test(system_id, component_id, last_msg);
}
#endif





static void mavlink_test_test(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // TEST_TESTSUITE_H
