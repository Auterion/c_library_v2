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
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,173,240,51,118,185
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
        packet1.current = packet_in.current;
        packet1.voltage = packet_in.voltage;
        packet1.mode = packet_in.mode;
        packet1.err = packet_in.err;
        packet1.motor_err = packet_in.motor_err;
        packet1.encoder_err = packet_in.encoder_err;
        packet1.state = packet_in.state;
        
        
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
    mavlink_msg_winch_debug_data_pack(system_id, component_id, &msg , packet1.spool_x , packet1.spool_v , packet1.spool_x_setpoint , packet1.spool_v_setpoint , packet1.payout_x , packet1.payout_v , packet1.payout_x_setpoint , packet1.payout_v_setpoint , packet1.correction , packet1.datum , packet1.force , packet1.disparity , packet1.mode , packet1.current , packet1.voltage , packet1.err , packet1.motor_err , packet1.encoder_err , packet1.state );
    mavlink_msg_winch_debug_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_debug_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.spool_x , packet1.spool_v , packet1.spool_x_setpoint , packet1.spool_v_setpoint , packet1.payout_x , packet1.payout_v , packet1.payout_x_setpoint , packet1.payout_v_setpoint , packet1.correction , packet1.datum , packet1.force , packet1.disparity , packet1.mode , packet1.current , packet1.voltage , packet1.err , packet1.motor_err , packet1.encoder_err , packet1.state );
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
    mavlink_msg_winch_debug_data_send(MAVLINK_COMM_1 , packet1.spool_x , packet1.spool_v , packet1.spool_x_setpoint , packet1.spool_v_setpoint , packet1.payout_x , packet1.payout_v , packet1.payout_x_setpoint , packet1.payout_v_setpoint , packet1.correction , packet1.datum , packet1.force , packet1.disparity , packet1.mode , packet1.current , packet1.voltage , packet1.err , packet1.motor_err , packet1.encoder_err , packet1.state );
    mavlink_msg_winch_debug_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_reaction_wheel_telemetry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_reaction_wheel_telemetry_t packet_in = {
        17.0,45.0,73.0,101.0,53,120,187,254
    };
    mavlink_reaction_wheel_telemetry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.velocity = packet_in.velocity;
        packet1.torque = packet_in.torque;
        packet1.current = packet_in.current;
        packet1.voltage = packet_in.voltage;
        packet1.err = packet_in.err;
        packet1.motor_err = packet_in.motor_err;
        packet1.encoder_err = packet_in.encoder_err;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reaction_wheel_telemetry_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_reaction_wheel_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reaction_wheel_telemetry_pack(system_id, component_id, &msg , packet1.velocity , packet1.torque , packet1.current , packet1.voltage , packet1.err , packet1.motor_err , packet1.encoder_err , packet1.state );
    mavlink_msg_reaction_wheel_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reaction_wheel_telemetry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.velocity , packet1.torque , packet1.current , packet1.voltage , packet1.err , packet1.motor_err , packet1.encoder_err , packet1.state );
    mavlink_msg_reaction_wheel_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_reaction_wheel_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reaction_wheel_telemetry_send(MAVLINK_COMM_1 , packet1.velocity , packet1.torque , packet1.current , packet1.voltage , packet1.err , packet1.motor_err , packet1.encoder_err , packet1.state );
    mavlink_msg_reaction_wheel_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_alarm_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ALARM_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_alarm_report_t packet_in = {
        { 93372036854775807, 93372036854775808, 93372036854775809, 93372036854775810, 93372036854775811, 93372036854775812, 93372036854775813, 93372036854775814, 93372036854775815, 93372036854775816 },245,56,123
    };
    mavlink_alarm_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.blocking_preflight = packet_in.blocking_preflight;
        packet1.should_abort = packet_in.should_abort;
        packet1.should_terminate = packet_in.should_terminate;
        
        mav_array_memcpy(packet1.alarm_mask, packet_in.alarm_mask, sizeof(uint64_t)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_alarm_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_alarm_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_alarm_report_pack(system_id, component_id, &msg , packet1.alarm_mask , packet1.blocking_preflight , packet1.should_abort , packet1.should_terminate );
    mavlink_msg_alarm_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_alarm_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.alarm_mask , packet1.blocking_preflight , packet1.should_abort , packet1.should_terminate );
    mavlink_msg_alarm_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_alarm_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_alarm_report_send(MAVLINK_COMM_1 , packet1.alarm_mask , packet1.blocking_preflight , packet1.should_abort , packet1.should_terminate );
    mavlink_msg_alarm_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_alarm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ALARM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_alarm_t packet_in = {
        963497464,17
    };
    mavlink_set_alarm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.alarm_id = packet_in.alarm_id;
        packet1.set = packet_in.set;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ALARM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ALARM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_alarm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_alarm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_alarm_pack(system_id, component_id, &msg , packet1.alarm_id , packet1.set );
    mavlink_msg_set_alarm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_alarm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.alarm_id , packet1.set );
    mavlink_msg_set_alarm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_alarm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_alarm_send(MAVLINK_COMM_1 , packet1.alarm_id , packet1.set );
    mavlink_msg_set_alarm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_zipline(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_carrier_guidance(system_id, component_id, last_msg);
    mavlink_test_winch_debug_data(system_id, component_id, last_msg);
    mavlink_test_reaction_wheel_telemetry(system_id, component_id, last_msg);
    mavlink_test_alarm_report(system_id, component_id, last_msg);
    mavlink_test_set_alarm(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ZIPLINE_TESTSUITE_H
