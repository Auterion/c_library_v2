/** @file
 *    @brief MAVLink comm protocol testsuite generated from zipline.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef ZIPLINE_TESTSUITE_H
#define ZIPLINE_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_zipline(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_zipline(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_carrier_guidance(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CARRIER_GUIDANCE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_carrier_guidance_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
    mavlink_carrier_guidance_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.ax = packet_in.ax;
        packet1.ay = packet_in.ay;
        packet1.az = packet_in.az;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_carrier_guidance_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_carrier_guidance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_carrier_guidance_pack(system_id, component_id, &msg , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.ax , packet1.ay , packet1.az );
    mavlink_msg_carrier_guidance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_carrier_guidance_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.ax , packet1.ay , packet1.az );
    mavlink_msg_carrier_guidance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_carrier_guidance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_carrier_guidance_send(MAVLINK_COMM_1 , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.ax , packet1.ay , packet1.az );
    mavlink_msg_carrier_guidance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_winch_debug_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WINCH_DEBUG_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_winch_debug_data_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,149
    };
    mavlink_winch_debug_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.spool_x = packet_in.spool_x;
        packet1.spool_v = packet_in.spool_v;
        packet1.spool_x_setpoint = packet_in.spool_x_setpoint;
        packet1.spool_v_setpoint = packet_in.spool_v_setpoint;
        packet1.payout_x = packet_in.payout_x;
        packet1.payout_v = packet_in.payout_v;
        packet1.payout_x_setpoint = packet_in.payout_x_setpoint;
        packet1.payout_v_setpoint = packet_in.payout_v_setpoint;
        packet1.correction = packet_in.correction;
        packet1.datum = packet_in.datum;
        packet1.force = packet_in.force;
        packet1.disparity = packet_in.disparity;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_debug_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_winch_debug_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_debug_data_pack(system_id, component_id, &msg , packet1.spool_x , packet1.spool_v , packet1.spool_x_setpoint , packet1.spool_v_setpoint , packet1.payout_x , packet1.payout_v , packet1.payout_x_setpoint , packet1.payout_v_setpoint , packet1.correction , packet1.datum , packet1.force , packet1.disparity , packet1.mode );
    mavlink_msg_winch_debug_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_debug_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.spool_x , packet1.spool_v , packet1.spool_x_setpoint , packet1.spool_v_setpoint , packet1.payout_x , packet1.payout_v , packet1.payout_x_setpoint , packet1.payout_v_setpoint , packet1.correction , packet1.datum , packet1.force , packet1.disparity , packet1.mode );
    mavlink_msg_winch_debug_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_winch_debug_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_debug_data_send(MAVLINK_COMM_1 , packet1.spool_x , packet1.spool_v , packet1.spool_x_setpoint , packet1.spool_v_setpoint , packet1.payout_x , packet1.payout_v , packet1.payout_x_setpoint , packet1.payout_v_setpoint , packet1.correction , packet1.datum , packet1.force , packet1.disparity , packet1.mode );
    mavlink_msg_winch_debug_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_zipline(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_carrier_guidance(system_id, component_id, last_msg);
    mavlink_test_winch_debug_data(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ZIPLINE_TESTSUITE_H
