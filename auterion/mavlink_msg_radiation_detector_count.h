#pragma once
// MESSAGE RADIATION_DETECTOR_COUNT PACKING

#define MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT 450


typedef struct __mavlink_radiation_detector_count_t {
 double timestamp; /*< [s]  Timestamp (UNIX Epoch time or time since detector boot).*/
 float dt; /*< [s]  Delta-t integration period.*/
 uint32_t count; /*<   Detector count since start of measurement.*/
 uint32_t rate; /*<   Detector count in the current dt integration period.*/
} mavlink_radiation_detector_count_t;

#define MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN 20
#define MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN 20
#define MAVLINK_MSG_ID_450_LEN 20
#define MAVLINK_MSG_ID_450_MIN_LEN 20

#define MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC 26
#define MAVLINK_MSG_ID_450_CRC 26



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADIATION_DETECTOR_COUNT { \
    450, \
    "RADIATION_DETECTOR_COUNT", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_radiation_detector_count_t, timestamp) }, \
         { "dt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_radiation_detector_count_t, dt) }, \
         { "count", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_radiation_detector_count_t, count) }, \
         { "rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_radiation_detector_count_t, rate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADIATION_DETECTOR_COUNT { \
    "RADIATION_DETECTOR_COUNT", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_radiation_detector_count_t, timestamp) }, \
         { "dt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_radiation_detector_count_t, dt) }, \
         { "count", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_radiation_detector_count_t, count) }, \
         { "rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_radiation_detector_count_t, rate) }, \
         } \
}
#endif

/**
 * @brief Pack a radiation_detector_count message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [s]  Timestamp (UNIX Epoch time or time since detector boot).
 * @param dt [s]  Delta-t integration period.
 * @param count   Detector count since start of measurement.
 * @param rate   Detector count in the current dt integration period.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radiation_detector_count_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double timestamp, float dt, uint32_t count, uint32_t rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN];
    _mav_put_double(buf, 0, timestamp);
    _mav_put_float(buf, 8, dt);
    _mav_put_uint32_t(buf, 12, count);
    _mav_put_uint32_t(buf, 16, rate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN);
#else
    mavlink_radiation_detector_count_t packet;
    packet.timestamp = timestamp;
    packet.dt = dt;
    packet.count = count;
    packet.rate = rate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC);
}

/**
 * @brief Pack a radiation_detector_count message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [s]  Timestamp (UNIX Epoch time or time since detector boot).
 * @param dt [s]  Delta-t integration period.
 * @param count   Detector count since start of measurement.
 * @param rate   Detector count in the current dt integration period.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radiation_detector_count_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double timestamp,float dt,uint32_t count,uint32_t rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN];
    _mav_put_double(buf, 0, timestamp);
    _mav_put_float(buf, 8, dt);
    _mav_put_uint32_t(buf, 12, count);
    _mav_put_uint32_t(buf, 16, rate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN);
#else
    mavlink_radiation_detector_count_t packet;
    packet.timestamp = timestamp;
    packet.dt = dt;
    packet.count = count;
    packet.rate = rate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC);
}

/**
 * @brief Encode a radiation_detector_count struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radiation_detector_count C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radiation_detector_count_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radiation_detector_count_t* radiation_detector_count)
{
    return mavlink_msg_radiation_detector_count_pack(system_id, component_id, msg, radiation_detector_count->timestamp, radiation_detector_count->dt, radiation_detector_count->count, radiation_detector_count->rate);
}

/**
 * @brief Encode a radiation_detector_count struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radiation_detector_count C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radiation_detector_count_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radiation_detector_count_t* radiation_detector_count)
{
    return mavlink_msg_radiation_detector_count_pack_chan(system_id, component_id, chan, msg, radiation_detector_count->timestamp, radiation_detector_count->dt, radiation_detector_count->count, radiation_detector_count->rate);
}

/**
 * @brief Send a radiation_detector_count message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [s]  Timestamp (UNIX Epoch time or time since detector boot).
 * @param dt [s]  Delta-t integration period.
 * @param count   Detector count since start of measurement.
 * @param rate   Detector count in the current dt integration period.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radiation_detector_count_send(mavlink_channel_t chan, double timestamp, float dt, uint32_t count, uint32_t rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN];
    _mav_put_double(buf, 0, timestamp);
    _mav_put_float(buf, 8, dt);
    _mav_put_uint32_t(buf, 12, count);
    _mav_put_uint32_t(buf, 16, rate);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT, buf, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC);
#else
    mavlink_radiation_detector_count_t packet;
    packet.timestamp = timestamp;
    packet.dt = dt;
    packet.count = count;
    packet.rate = rate;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT, (const char *)&packet, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC);
#endif
}

/**
 * @brief Send a radiation_detector_count message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radiation_detector_count_send_struct(mavlink_channel_t chan, const mavlink_radiation_detector_count_t* radiation_detector_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radiation_detector_count_send(chan, radiation_detector_count->timestamp, radiation_detector_count->dt, radiation_detector_count->count, radiation_detector_count->rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT, (const char *)radiation_detector_count, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radiation_detector_count_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double timestamp, float dt, uint32_t count, uint32_t rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, timestamp);
    _mav_put_float(buf, 8, dt);
    _mav_put_uint32_t(buf, 12, count);
    _mav_put_uint32_t(buf, 16, rate);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT, buf, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC);
#else
    mavlink_radiation_detector_count_t *packet = (mavlink_radiation_detector_count_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->dt = dt;
    packet->count = count;
    packet->rate = rate;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT, (const char *)packet, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_MIN_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_CRC);
#endif
}
#endif

#endif

// MESSAGE RADIATION_DETECTOR_COUNT UNPACKING


/**
 * @brief Get field timestamp from radiation_detector_count message
 *
 * @return [s]  Timestamp (UNIX Epoch time or time since detector boot).
 */
static inline double mavlink_msg_radiation_detector_count_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field dt from radiation_detector_count message
 *
 * @return [s]  Delta-t integration period.
 */
static inline float mavlink_msg_radiation_detector_count_get_dt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field count from radiation_detector_count message
 *
 * @return   Detector count since start of measurement.
 */
static inline uint32_t mavlink_msg_radiation_detector_count_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field rate from radiation_detector_count message
 *
 * @return   Detector count in the current dt integration period.
 */
static inline uint32_t mavlink_msg_radiation_detector_count_get_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Decode a radiation_detector_count message into a struct
 *
 * @param msg The message to decode
 * @param radiation_detector_count C-struct to decode the message contents into
 */
static inline void mavlink_msg_radiation_detector_count_decode(const mavlink_message_t* msg, mavlink_radiation_detector_count_t* radiation_detector_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radiation_detector_count->timestamp = mavlink_msg_radiation_detector_count_get_timestamp(msg);
    radiation_detector_count->dt = mavlink_msg_radiation_detector_count_get_dt(msg);
    radiation_detector_count->count = mavlink_msg_radiation_detector_count_get_count(msg);
    radiation_detector_count->rate = mavlink_msg_radiation_detector_count_get_rate(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN? msg->len : MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN;
        memset(radiation_detector_count, 0, MAVLINK_MSG_ID_RADIATION_DETECTOR_COUNT_LEN);
    memcpy(radiation_detector_count, _MAV_PAYLOAD(msg), len);
#endif
}
