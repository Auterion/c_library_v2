#pragma once
// MESSAGE CARRIER_GUIDANCE PACKING

#define MAVLINK_MSG_ID_CARRIER_GUIDANCE 7001


typedef struct __mavlink_carrier_guidance_t {
 float x; /*< [m] X Position in NED Frame*/
 float y; /*< [m] Y Position in NED Frame*/
 float z; /*< [m] Z Position in NED Frame*/
 float vx; /*< [m/s] X Velocity in NED Frame*/
 float vy; /*< [m/s] X Velocity in NED Frame*/
 float vz; /*< [m/s] X Velocity in NED Frame*/
 float ax; /*< [m/s^2] X Acceleration in NED Frame*/
 float ay; /*< [m/s^2] X Acceleration in NED Frame*/
 float az; /*< [m/s^2] X Acceleration in NED Frame*/
} mavlink_carrier_guidance_t;

#define MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN 36
#define MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN 36
#define MAVLINK_MSG_ID_7001_LEN 36
#define MAVLINK_MSG_ID_7001_MIN_LEN 36

#define MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC 244
#define MAVLINK_MSG_ID_7001_CRC 244



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CARRIER_GUIDANCE { \
    7001, \
    "CARRIER_GUIDANCE", \
    9, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_carrier_guidance_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_carrier_guidance_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_carrier_guidance_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_carrier_guidance_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_carrier_guidance_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_carrier_guidance_t, vz) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_carrier_guidance_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_carrier_guidance_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_carrier_guidance_t, az) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CARRIER_GUIDANCE { \
    "CARRIER_GUIDANCE", \
    9, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_carrier_guidance_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_carrier_guidance_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_carrier_guidance_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_carrier_guidance_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_carrier_guidance_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_carrier_guidance_t, vz) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_carrier_guidance_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_carrier_guidance_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_carrier_guidance_t, az) }, \
         } \
}
#endif

/**
 * @brief Pack a carrier_guidance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x [m] X Position in NED Frame
 * @param y [m] Y Position in NED Frame
 * @param z [m] Z Position in NED Frame
 * @param vx [m/s] X Velocity in NED Frame
 * @param vy [m/s] X Velocity in NED Frame
 * @param vz [m/s] X Velocity in NED Frame
 * @param ax [m/s^2] X Acceleration in NED Frame
 * @param ay [m/s^2] X Acceleration in NED Frame
 * @param az [m/s^2] X Acceleration in NED Frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_carrier_guidance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);
    _mav_put_float(buf, 24, ax);
    _mav_put_float(buf, 28, ay);
    _mav_put_float(buf, 32, az);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN);
#else
    mavlink_carrier_guidance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CARRIER_GUIDANCE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC);
}

/**
 * @brief Pack a carrier_guidance message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x [m] X Position in NED Frame
 * @param y [m] Y Position in NED Frame
 * @param z [m] Z Position in NED Frame
 * @param vx [m/s] X Velocity in NED Frame
 * @param vy [m/s] X Velocity in NED Frame
 * @param vz [m/s] X Velocity in NED Frame
 * @param ax [m/s^2] X Acceleration in NED Frame
 * @param ay [m/s^2] X Acceleration in NED Frame
 * @param az [m/s^2] X Acceleration in NED Frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_carrier_guidance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float vx,float vy,float vz,float ax,float ay,float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);
    _mav_put_float(buf, 24, ax);
    _mav_put_float(buf, 28, ay);
    _mav_put_float(buf, 32, az);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN);
#else
    mavlink_carrier_guidance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CARRIER_GUIDANCE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC);
}

/**
 * @brief Encode a carrier_guidance struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param carrier_guidance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_carrier_guidance_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_carrier_guidance_t* carrier_guidance)
{
    return mavlink_msg_carrier_guidance_pack(system_id, component_id, msg, carrier_guidance->x, carrier_guidance->y, carrier_guidance->z, carrier_guidance->vx, carrier_guidance->vy, carrier_guidance->vz, carrier_guidance->ax, carrier_guidance->ay, carrier_guidance->az);
}

/**
 * @brief Encode a carrier_guidance struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param carrier_guidance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_carrier_guidance_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_carrier_guidance_t* carrier_guidance)
{
    return mavlink_msg_carrier_guidance_pack_chan(system_id, component_id, chan, msg, carrier_guidance->x, carrier_guidance->y, carrier_guidance->z, carrier_guidance->vx, carrier_guidance->vy, carrier_guidance->vz, carrier_guidance->ax, carrier_guidance->ay, carrier_guidance->az);
}

/**
 * @brief Send a carrier_guidance message
 * @param chan MAVLink channel to send the message
 *
 * @param x [m] X Position in NED Frame
 * @param y [m] Y Position in NED Frame
 * @param z [m] Z Position in NED Frame
 * @param vx [m/s] X Velocity in NED Frame
 * @param vy [m/s] X Velocity in NED Frame
 * @param vz [m/s] X Velocity in NED Frame
 * @param ax [m/s^2] X Acceleration in NED Frame
 * @param ay [m/s^2] X Acceleration in NED Frame
 * @param az [m/s^2] X Acceleration in NED Frame
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_carrier_guidance_send(mavlink_channel_t chan, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);
    _mav_put_float(buf, 24, ax);
    _mav_put_float(buf, 28, ay);
    _mav_put_float(buf, 32, az);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CARRIER_GUIDANCE, buf, MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC);
#else
    mavlink_carrier_guidance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CARRIER_GUIDANCE, (const char *)&packet, MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC);
#endif
}

/**
 * @brief Send a carrier_guidance message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_carrier_guidance_send_struct(mavlink_channel_t chan, const mavlink_carrier_guidance_t* carrier_guidance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_carrier_guidance_send(chan, carrier_guidance->x, carrier_guidance->y, carrier_guidance->z, carrier_guidance->vx, carrier_guidance->vy, carrier_guidance->vz, carrier_guidance->ax, carrier_guidance->ay, carrier_guidance->az);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CARRIER_GUIDANCE, (const char *)carrier_guidance, MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_carrier_guidance_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);
    _mav_put_float(buf, 24, ax);
    _mav_put_float(buf, 28, ay);
    _mav_put_float(buf, 32, az);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CARRIER_GUIDANCE, buf, MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC);
#else
    mavlink_carrier_guidance_t *packet = (mavlink_carrier_guidance_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->ax = ax;
    packet->ay = ay;
    packet->az = az;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CARRIER_GUIDANCE, (const char *)packet, MAVLINK_MSG_ID_CARRIER_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN, MAVLINK_MSG_ID_CARRIER_GUIDANCE_CRC);
#endif
}
#endif

#endif

// MESSAGE CARRIER_GUIDANCE UNPACKING


/**
 * @brief Get field x from carrier_guidance message
 *
 * @return [m] X Position in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from carrier_guidance message
 *
 * @return [m] Y Position in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from carrier_guidance message
 *
 * @return [m] Z Position in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vx from carrier_guidance message
 *
 * @return [m/s] X Velocity in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vy from carrier_guidance message
 *
 * @return [m/s] X Velocity in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vz from carrier_guidance message
 *
 * @return [m/s] X Velocity in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ax from carrier_guidance message
 *
 * @return [m/s^2] X Acceleration in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_ax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ay from carrier_guidance message
 *
 * @return [m/s^2] X Acceleration in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_ay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field az from carrier_guidance message
 *
 * @return [m/s^2] X Acceleration in NED Frame
 */
static inline float mavlink_msg_carrier_guidance_get_az(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a carrier_guidance message into a struct
 *
 * @param msg The message to decode
 * @param carrier_guidance C-struct to decode the message contents into
 */
static inline void mavlink_msg_carrier_guidance_decode(const mavlink_message_t* msg, mavlink_carrier_guidance_t* carrier_guidance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    carrier_guidance->x = mavlink_msg_carrier_guidance_get_x(msg);
    carrier_guidance->y = mavlink_msg_carrier_guidance_get_y(msg);
    carrier_guidance->z = mavlink_msg_carrier_guidance_get_z(msg);
    carrier_guidance->vx = mavlink_msg_carrier_guidance_get_vx(msg);
    carrier_guidance->vy = mavlink_msg_carrier_guidance_get_vy(msg);
    carrier_guidance->vz = mavlink_msg_carrier_guidance_get_vz(msg);
    carrier_guidance->ax = mavlink_msg_carrier_guidance_get_ax(msg);
    carrier_guidance->ay = mavlink_msg_carrier_guidance_get_ay(msg);
    carrier_guidance->az = mavlink_msg_carrier_guidance_get_az(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN? msg->len : MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN;
        memset(carrier_guidance, 0, MAVLINK_MSG_ID_CARRIER_GUIDANCE_LEN);
    memcpy(carrier_guidance, _MAV_PAYLOAD(msg), len);
#endif
}
