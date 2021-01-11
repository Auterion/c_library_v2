#pragma once
// MESSAGE WINCH_CORRECTION PACKING

#define MAVLINK_MSG_ID_WINCH_CORRECTION 7002


typedef struct __mavlink_winch_correction_t {
 float position; /*< [m] Position*/
 float velocity; /*< [m] Velocity (unused)*/
 float acceleration; /*< [m] Acceleration (unused)*/
} mavlink_winch_correction_t;

#define MAVLINK_MSG_ID_WINCH_CORRECTION_LEN 12
#define MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN 12
#define MAVLINK_MSG_ID_7002_LEN 12
#define MAVLINK_MSG_ID_7002_MIN_LEN 12

#define MAVLINK_MSG_ID_WINCH_CORRECTION_CRC 172
#define MAVLINK_MSG_ID_7002_CRC 172



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WINCH_CORRECTION { \
    7002, \
    "WINCH_CORRECTION", \
    3, \
    {  { "position", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_winch_correction_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_winch_correction_t, velocity) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_winch_correction_t, acceleration) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WINCH_CORRECTION { \
    "WINCH_CORRECTION", \
    3, \
    {  { "position", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_winch_correction_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_winch_correction_t, velocity) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_winch_correction_t, acceleration) }, \
         } \
}
#endif

/**
 * @brief Pack a winch_correction message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param position [m] Position
 * @param velocity [m] Velocity (unused)
 * @param acceleration [m] Acceleration (unused)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_winch_correction_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float position, float velocity, float acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_CORRECTION_LEN];
    _mav_put_float(buf, 0, position);
    _mav_put_float(buf, 4, velocity);
    _mav_put_float(buf, 8, acceleration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN);
#else
    mavlink_winch_correction_t packet;
    packet.position = position;
    packet.velocity = velocity;
    packet.acceleration = acceleration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINCH_CORRECTION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_CRC);
}

/**
 * @brief Pack a winch_correction message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param position [m] Position
 * @param velocity [m] Velocity (unused)
 * @param acceleration [m] Acceleration (unused)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_winch_correction_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float position,float velocity,float acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_CORRECTION_LEN];
    _mav_put_float(buf, 0, position);
    _mav_put_float(buf, 4, velocity);
    _mav_put_float(buf, 8, acceleration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN);
#else
    mavlink_winch_correction_t packet;
    packet.position = position;
    packet.velocity = velocity;
    packet.acceleration = acceleration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINCH_CORRECTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_CRC);
}

/**
 * @brief Encode a winch_correction struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param winch_correction C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_winch_correction_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_winch_correction_t* winch_correction)
{
    return mavlink_msg_winch_correction_pack(system_id, component_id, msg, winch_correction->position, winch_correction->velocity, winch_correction->acceleration);
}

/**
 * @brief Encode a winch_correction struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param winch_correction C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_winch_correction_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_winch_correction_t* winch_correction)
{
    return mavlink_msg_winch_correction_pack_chan(system_id, component_id, chan, msg, winch_correction->position, winch_correction->velocity, winch_correction->acceleration);
}

/**
 * @brief Send a winch_correction message
 * @param chan MAVLink channel to send the message
 *
 * @param position [m] Position
 * @param velocity [m] Velocity (unused)
 * @param acceleration [m] Acceleration (unused)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_winch_correction_send(mavlink_channel_t chan, float position, float velocity, float acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_CORRECTION_LEN];
    _mav_put_float(buf, 0, position);
    _mav_put_float(buf, 4, velocity);
    _mav_put_float(buf, 8, acceleration);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_CORRECTION, buf, MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_CRC);
#else
    mavlink_winch_correction_t packet;
    packet.position = position;
    packet.velocity = velocity;
    packet.acceleration = acceleration;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_CORRECTION, (const char *)&packet, MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_CRC);
#endif
}

/**
 * @brief Send a winch_correction message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_winch_correction_send_struct(mavlink_channel_t chan, const mavlink_winch_correction_t* winch_correction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_winch_correction_send(chan, winch_correction->position, winch_correction->velocity, winch_correction->acceleration);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_CORRECTION, (const char *)winch_correction, MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_CRC);
#endif
}

#if MAVLINK_MSG_ID_WINCH_CORRECTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_winch_correction_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float position, float velocity, float acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, position);
    _mav_put_float(buf, 4, velocity);
    _mav_put_float(buf, 8, acceleration);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_CORRECTION, buf, MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_CRC);
#else
    mavlink_winch_correction_t *packet = (mavlink_winch_correction_t *)msgbuf;
    packet->position = position;
    packet->velocity = velocity;
    packet->acceleration = acceleration;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_CORRECTION, (const char *)packet, MAVLINK_MSG_ID_WINCH_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN, MAVLINK_MSG_ID_WINCH_CORRECTION_CRC);
#endif
}
#endif

#endif

// MESSAGE WINCH_CORRECTION UNPACKING


/**
 * @brief Get field position from winch_correction message
 *
 * @return [m] Position
 */
static inline float mavlink_msg_winch_correction_get_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field velocity from winch_correction message
 *
 * @return [m] Velocity (unused)
 */
static inline float mavlink_msg_winch_correction_get_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field acceleration from winch_correction message
 *
 * @return [m] Acceleration (unused)
 */
static inline float mavlink_msg_winch_correction_get_acceleration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a winch_correction message into a struct
 *
 * @param msg The message to decode
 * @param winch_correction C-struct to decode the message contents into
 */
static inline void mavlink_msg_winch_correction_decode(const mavlink_message_t* msg, mavlink_winch_correction_t* winch_correction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    winch_correction->position = mavlink_msg_winch_correction_get_position(msg);
    winch_correction->velocity = mavlink_msg_winch_correction_get_velocity(msg);
    winch_correction->acceleration = mavlink_msg_winch_correction_get_acceleration(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WINCH_CORRECTION_LEN? msg->len : MAVLINK_MSG_ID_WINCH_CORRECTION_LEN;
        memset(winch_correction, 0, MAVLINK_MSG_ID_WINCH_CORRECTION_LEN);
    memcpy(winch_correction, _MAV_PAYLOAD(msg), len);
#endif
}
