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
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,20147,20251,20355,20459,197
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
        packet1.err = packet_in.err;
        packet1.motor_err = packet_in.motor_err;
        packet1.encoder_err = packet_in.encoder_err;
        packet1.state = packet_in.state;
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
        17.0,45.0,73.0,101.0,18067,18171,18275,18379
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
        { 93372036854775807, 93372036854775808, 93372036854775809, 93372036854775810, 93372036854775811, 93372036854775812, 93372036854775813, 93372036854775814, 93372036854775815, 93372036854775816 },245
    };
    mavlink_alarm_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.failure_level = packet_in.failure_level;
        
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
    mavlink_msg_alarm_report_pack(system_id, component_id, &msg , packet1.alarm_mask , packet1.failure_level );
    mavlink_msg_alarm_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_alarm_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.alarm_mask , packet1.failure_level );
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
    mavlink_msg_alarm_report_send(MAVLINK_COMM_1 , packet1.alarm_mask , packet1.failure_level );
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

static void mavlink_test_wind_sensor_ground(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIND_SENSOR_GROUND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wind_sensor_ground_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_wind_sensor_ground_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.wind_n = packet_in.wind_n;
        packet1.wind_e = packet_in.wind_e;
        packet1.wind_d = packet_in.wind_d;
        packet1.temperature = packet_in.temperature;
        packet1.humidity = packet_in.humidity;
        packet1.pressure = packet_in.pressure;
        packet1.wind_speed = packet_in.wind_speed;
        packet1.wind_heading = packet_in.wind_heading;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIND_SENSOR_GROUND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIND_SENSOR_GROUND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_ground_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wind_sensor_ground_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_ground_pack(system_id, component_id, &msg , packet1.wind_n , packet1.wind_e , packet1.wind_d , packet1.temperature , packet1.humidity , packet1.pressure , packet1.wind_speed , packet1.wind_heading );
    mavlink_msg_wind_sensor_ground_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_ground_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.wind_n , packet1.wind_e , packet1.wind_d , packet1.temperature , packet1.humidity , packet1.pressure , packet1.wind_speed , packet1.wind_heading );
    mavlink_msg_wind_sensor_ground_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wind_sensor_ground_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_ground_send(MAVLINK_COMM_1 , packet1.wind_n , packet1.wind_e , packet1.wind_d , packet1.temperature , packet1.humidity , packet1.pressure , packet1.wind_speed , packet1.wind_heading );
    mavlink_msg_wind_sensor_ground_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_wind_sensor_body(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIND_SENSOR_BODY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wind_sensor_body_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_wind_sensor_body_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.wind_n = packet_in.wind_n;
        packet1.wind_e = packet_in.wind_e;
        packet1.wind_d = packet_in.wind_d;
        packet1.temperature = packet_in.temperature;
        packet1.humidity = packet_in.humidity;
        packet1.pressure = packet_in.pressure;
        packet1.wind_speed = packet_in.wind_speed;
        packet1.wind_heading = packet_in.wind_heading;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIND_SENSOR_BODY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIND_SENSOR_BODY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_body_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wind_sensor_body_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_body_pack(system_id, component_id, &msg , packet1.wind_n , packet1.wind_e , packet1.wind_d , packet1.temperature , packet1.humidity , packet1.pressure , packet1.wind_speed , packet1.wind_heading );
    mavlink_msg_wind_sensor_body_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_body_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.wind_n , packet1.wind_e , packet1.wind_d , packet1.temperature , packet1.humidity , packet1.pressure , packet1.wind_speed , packet1.wind_heading );
    mavlink_msg_wind_sensor_body_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wind_sensor_body_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_sensor_body_send(MAVLINK_COMM_1 , packet1.wind_n , packet1.wind_e , packet1.wind_d , packet1.temperature , packet1.humidity , packet1.pressure , packet1.wind_speed , packet1.wind_heading );
    mavlink_msg_wind_sensor_body_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_package_commander_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_package_commander_status_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,149,216,27,94,161
    };
    mavlink_package_commander_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.autoretract_time_utc = packet_in.autoretract_time_utc;
        packet1.impact_pos_x = packet_in.impact_pos_x;
        packet1.impact_pos_y = packet_in.impact_pos_y;
        packet1.impact_pos_z = packet_in.impact_pos_z;
        packet1.target_impact_pos_x = packet_in.target_impact_pos_x;
        packet1.target_impact_pos_y = packet_in.target_impact_pos_y;
        packet1.target_impact_pos_z = packet_in.target_impact_pos_z;
        packet1.xy_error = packet_in.xy_error;
        packet1.z_error = packet_in.z_error;
        packet1.xy_error_touchdown = packet_in.xy_error_touchdown;
        packet1.z_error_touchdown = packet_in.z_error_touchdown;
        packet1.is_active = packet_in.is_active;
        packet1.on_ground = packet_in.on_ground;
        packet1.autoretract_allowed = packet_in.autoretract_allowed;
        packet1.autoretract_enabed = packet_in.autoretract_enabed;
        packet1.ready_for_autoretract = packet_in.ready_for_autoretract;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_package_commander_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_package_commander_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_package_commander_status_pack(system_id, component_id, &msg , packet1.is_active , packet1.on_ground , packet1.autoretract_allowed , packet1.autoretract_enabed , packet1.autoretract_time_utc , packet1.ready_for_autoretract , packet1.impact_pos_x , packet1.impact_pos_y , packet1.impact_pos_z , packet1.target_impact_pos_x , packet1.target_impact_pos_y , packet1.target_impact_pos_z , packet1.xy_error , packet1.z_error , packet1.xy_error_touchdown , packet1.z_error_touchdown );
    mavlink_msg_package_commander_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_package_commander_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.is_active , packet1.on_ground , packet1.autoretract_allowed , packet1.autoretract_enabed , packet1.autoretract_time_utc , packet1.ready_for_autoretract , packet1.impact_pos_x , packet1.impact_pos_y , packet1.impact_pos_z , packet1.target_impact_pos_x , packet1.target_impact_pos_y , packet1.target_impact_pos_z , packet1.xy_error , packet1.z_error , packet1.xy_error_touchdown , packet1.z_error_touchdown );
    mavlink_msg_package_commander_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_package_commander_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_package_commander_status_send(MAVLINK_COMM_1 , packet1.is_active , packet1.on_ground , packet1.autoretract_allowed , packet1.autoretract_enabed , packet1.autoretract_time_utc , packet1.ready_for_autoretract , packet1.impact_pos_x , packet1.impact_pos_y , packet1.impact_pos_z , packet1.target_impact_pos_x , packet1.target_impact_pos_y , packet1.target_impact_pos_z , packet1.xy_error , packet1.z_error , packet1.xy_error_touchdown , packet1.z_error_touchdown );
    mavlink_msg_package_commander_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_file_log_chunk(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FILE_LOG_CHUNK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_file_log_chunk_t packet_in = {
        17235,17339,{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36 },77,{ 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87 }
    };
    mavlink_file_log_chunk_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.chunks = packet_in.chunks;
        packet1.chunk_index = packet_in.chunk_index;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.filename, packet_in.filename, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*200);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_log_chunk_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_file_log_chunk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_log_chunk_pack(system_id, component_id, &msg , packet1.filename , packet1.chunks , packet1.chunk_index , packet1.len , packet1.data );
    mavlink_msg_file_log_chunk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_log_chunk_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.filename , packet1.chunks , packet1.chunk_index , packet1.len , packet1.data );
    mavlink_msg_file_log_chunk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_file_log_chunk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_log_chunk_send(MAVLINK_COMM_1 , packet1.filename , packet1.chunks , packet1.chunk_index , packet1.len , packet1.data );
    mavlink_msg_file_log_chunk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_zipline(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_carrier_guidance(system_id, component_id, last_msg);
    mavlink_test_winch_debug_data(system_id, component_id, last_msg);
    mavlink_test_reaction_wheel_telemetry(system_id, component_id, last_msg);
    mavlink_test_alarm_report(system_id, component_id, last_msg);
    mavlink_test_set_alarm(system_id, component_id, last_msg);
    mavlink_test_wind_sensor_ground(system_id, component_id, last_msg);
    mavlink_test_wind_sensor_body(system_id, component_id, last_msg);
    mavlink_test_package_commander_status(system_id, component_id, last_msg);
    mavlink_test_file_log_chunk(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ZIPLINE_TESTSUITE_H
