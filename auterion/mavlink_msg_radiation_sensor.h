#pragma once
// MESSAGE RADIATION_SENSOR PACKING

#define MAVLINK_MSG_ID_RADIATION_SENSOR 450


typedef struct __mavlink_radiation_sensor_t {
 uint32_t detector_time; /*< [ms]  The current uptime of a detector, counting upwards since the first time it came up.*/
 uint32_t delta_ts; /*< [s]  The time that has passed since the last measurement has been sent.*/
 uint32_t counts[2]; /*<   An array of detector counts (accumulated since start). First element stands for all the counts from low to mid cut, second is for the values  that are in the bins from mid to high cut. The values between mid and high cut are of interest for integrators.*/
 float rates[2]; /*< [cps]  An array containing the rate below and above the mid cut. The rate unit is counts per second normalized from the counted neutrons in the time difference delta_t_s.*/
} mavlink_radiation_sensor_t;

#define MAVLINK_MSG_ID_RADIATION_SENSOR_LEN 24
#define MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN 24
#define MAVLINK_MSG_ID_450_LEN 24
#define MAVLINK_MSG_ID_450_MIN_LEN 24

#define MAVLINK_MSG_ID_RADIATION_SENSOR_CRC 69
#define MAVLINK_MSG_ID_450_CRC 69

#define MAVLINK_MSG_RADIATION_SENSOR_FIELD_COUNTS_LEN 2
#define MAVLINK_MSG_RADIATION_SENSOR_FIELD_RATES_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADIATION_SENSOR { \
    450, \
    "RADIATION_SENSOR", \
    4, \
    {  { "detector_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radiation_sensor_t, detector_time) }, \
         { "delta_ts", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_radiation_sensor_t, delta_ts) }, \
         { "counts", NULL, MAVLINK_TYPE_UINT32_T, 2, 8, offsetof(mavlink_radiation_sensor_t, counts) }, \
         { "rates", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_radiation_sensor_t, rates) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADIATION_SENSOR { \
    "RADIATION_SENSOR", \
    4, \
    {  { "detector_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radiation_sensor_t, detector_time) }, \
         { "delta_ts", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_radiation_sensor_t, delta_ts) }, \
         { "counts", NULL, MAVLINK_TYPE_UINT32_T, 2, 8, offsetof(mavlink_radiation_sensor_t, counts) }, \
         { "rates", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_radiation_sensor_t, rates) }, \
         } \
}
#endif

/**
 * @brief Pack a radiation_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param detector_time [ms]  The current uptime of a detector, counting upwards since the first time it came up.
 * @param delta_ts [s]  The time that has passed since the last measurement has been sent.
 * @param counts   An array of detector counts (accumulated since start). First element stands for all the counts from low to mid cut, second is for the values  that are in the bins from mid to high cut. The values between mid and high cut are of interest for integrators.
 * @param rates [cps]  An array containing the rate below and above the mid cut. The rate unit is counts per second normalized from the counted neutrons in the time difference delta_t_s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radiation_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t detector_time, uint32_t delta_ts, const uint32_t *counts, const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIATION_SENSOR_LEN];
    _mav_put_uint32_t(buf, 0, detector_time);
    _mav_put_uint32_t(buf, 4, delta_ts);
    _mav_put_uint32_t_array(buf, 8, counts, 2);
    _mav_put_float_array(buf, 16, rates, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN);
#else
    mavlink_radiation_sensor_t packet;
    packet.detector_time = detector_time;
    packet.delta_ts = delta_ts;
    mav_array_memcpy(packet.counts, counts, sizeof(uint32_t)*2);
    mav_array_memcpy(packet.rates, rates, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIATION_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_CRC);
}

/**
 * @brief Pack a radiation_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param detector_time [ms]  The current uptime of a detector, counting upwards since the first time it came up.
 * @param delta_ts [s]  The time that has passed since the last measurement has been sent.
 * @param counts   An array of detector counts (accumulated since start). First element stands for all the counts from low to mid cut, second is for the values  that are in the bins from mid to high cut. The values between mid and high cut are of interest for integrators.
 * @param rates [cps]  An array containing the rate below and above the mid cut. The rate unit is counts per second normalized from the counted neutrons in the time difference delta_t_s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radiation_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t detector_time,uint32_t delta_ts,const uint32_t *counts,const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIATION_SENSOR_LEN];
    _mav_put_uint32_t(buf, 0, detector_time);
    _mav_put_uint32_t(buf, 4, delta_ts);
    _mav_put_uint32_t_array(buf, 8, counts, 2);
    _mav_put_float_array(buf, 16, rates, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN);
#else
    mavlink_radiation_sensor_t packet;
    packet.detector_time = detector_time;
    packet.delta_ts = delta_ts;
    mav_array_memcpy(packet.counts, counts, sizeof(uint32_t)*2);
    mav_array_memcpy(packet.rates, rates, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIATION_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_CRC);
}

/**
 * @brief Encode a radiation_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radiation_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radiation_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radiation_sensor_t* radiation_sensor)
{
    return mavlink_msg_radiation_sensor_pack(system_id, component_id, msg, radiation_sensor->detector_time, radiation_sensor->delta_ts, radiation_sensor->counts, radiation_sensor->rates);
}

/**
 * @brief Encode a radiation_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radiation_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radiation_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radiation_sensor_t* radiation_sensor)
{
    return mavlink_msg_radiation_sensor_pack_chan(system_id, component_id, chan, msg, radiation_sensor->detector_time, radiation_sensor->delta_ts, radiation_sensor->counts, radiation_sensor->rates);
}

/**
 * @brief Send a radiation_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param detector_time [ms]  The current uptime of a detector, counting upwards since the first time it came up.
 * @param delta_ts [s]  The time that has passed since the last measurement has been sent.
 * @param counts   An array of detector counts (accumulated since start). First element stands for all the counts from low to mid cut, second is for the values  that are in the bins from mid to high cut. The values between mid and high cut are of interest for integrators.
 * @param rates [cps]  An array containing the rate below and above the mid cut. The rate unit is counts per second normalized from the counted neutrons in the time difference delta_t_s.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radiation_sensor_send(mavlink_channel_t chan, uint32_t detector_time, uint32_t delta_ts, const uint32_t *counts, const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIATION_SENSOR_LEN];
    _mav_put_uint32_t(buf, 0, detector_time);
    _mav_put_uint32_t(buf, 4, delta_ts);
    _mav_put_uint32_t_array(buf, 8, counts, 2);
    _mav_put_float_array(buf, 16, rates, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_SENSOR, buf, MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_CRC);
#else
    mavlink_radiation_sensor_t packet;
    packet.detector_time = detector_time;
    packet.delta_ts = delta_ts;
    mav_array_memcpy(packet.counts, counts, sizeof(uint32_t)*2);
    mav_array_memcpy(packet.rates, rates, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_CRC);
#endif
}

/**
 * @brief Send a radiation_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radiation_sensor_send_struct(mavlink_channel_t chan, const mavlink_radiation_sensor_t* radiation_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radiation_sensor_send(chan, radiation_sensor->detector_time, radiation_sensor->delta_ts, radiation_sensor->counts, radiation_sensor->rates);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_SENSOR, (const char *)radiation_sensor, MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADIATION_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radiation_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t detector_time, uint32_t delta_ts, const uint32_t *counts, const float *rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, detector_time);
    _mav_put_uint32_t(buf, 4, delta_ts);
    _mav_put_uint32_t_array(buf, 8, counts, 2);
    _mav_put_float_array(buf, 16, rates, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_SENSOR, buf, MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_CRC);
#else
    mavlink_radiation_sensor_t *packet = (mavlink_radiation_sensor_t *)msgbuf;
    packet->detector_time = detector_time;
    packet->delta_ts = delta_ts;
    mav_array_memcpy(packet->counts, counts, sizeof(uint32_t)*2);
    mav_array_memcpy(packet->rates, rates, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_SENSOR, (const char *)packet, MAVLINK_MSG_ID_RADIATION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN, MAVLINK_MSG_ID_RADIATION_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE RADIATION_SENSOR UNPACKING


/**
 * @brief Get field detector_time from radiation_sensor message
 *
 * @return [ms]  The current uptime of a detector, counting upwards since the first time it came up.
 */
static inline uint32_t mavlink_msg_radiation_sensor_get_detector_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field delta_ts from radiation_sensor message
 *
 * @return [s]  The time that has passed since the last measurement has been sent.
 */
static inline uint32_t mavlink_msg_radiation_sensor_get_delta_ts(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field counts from radiation_sensor message
 *
 * @return   An array of detector counts (accumulated since start). First element stands for all the counts from low to mid cut, second is for the values  that are in the bins from mid to high cut. The values between mid and high cut are of interest for integrators.
 */
static inline uint16_t mavlink_msg_radiation_sensor_get_counts(const mavlink_message_t* msg, uint32_t *counts)
{
    return _MAV_RETURN_uint32_t_array(msg, counts, 2,  8);
}

/**
 * @brief Get field rates from radiation_sensor message
 *
 * @return [cps]  An array containing the rate below and above the mid cut. The rate unit is counts per second normalized from the counted neutrons in the time difference delta_t_s.
 */
static inline uint16_t mavlink_msg_radiation_sensor_get_rates(const mavlink_message_t* msg, float *rates)
{
    return _MAV_RETURN_float_array(msg, rates, 2,  16);
}

/**
 * @brief Decode a radiation_sensor message into a struct
 *
 * @param msg The message to decode
 * @param radiation_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_radiation_sensor_decode(const mavlink_message_t* msg, mavlink_radiation_sensor_t* radiation_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radiation_sensor->detector_time = mavlink_msg_radiation_sensor_get_detector_time(msg);
    radiation_sensor->delta_ts = mavlink_msg_radiation_sensor_get_delta_ts(msg);
    mavlink_msg_radiation_sensor_get_counts(msg, radiation_sensor->counts);
    mavlink_msg_radiation_sensor_get_rates(msg, radiation_sensor->rates);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADIATION_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_RADIATION_SENSOR_LEN;
        memset(radiation_sensor, 0, MAVLINK_MSG_ID_RADIATION_SENSOR_LEN);
    memcpy(radiation_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
