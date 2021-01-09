#pragma once
// MESSAGE ZIP_GUIDANCE PACKING

#define MAVLINK_MSG_ID_ZIP_GUIDANCE 7001


typedef struct __mavlink_zip_guidance_t {
 float x; /*< [m] X Position in NED Frame*/
 float y; /*< [m] Y Position in NED Frame*/
 float z; /*< [m] Z Position in NED Frame*/
 float vx; /*< [m/s] X Velocity in NED Frame*/
 float vy; /*< [m/s] X Velocity in NED Frame*/
 float vz; /*< [m/s] X Velocity in NED Frame*/
} mavlink_zip_guidance_t;

#define MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN 24
#define MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN 24
#define MAVLINK_MSG_ID_7001_LEN 24
#define MAVLINK_MSG_ID_7001_MIN_LEN 24

#define MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC 183
#define MAVLINK_MSG_ID_7001_CRC 183



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZIP_GUIDANCE { \
    7001, \
    "ZIP_GUIDANCE", \
    6, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_zip_guidance_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_zip_guidance_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zip_guidance_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zip_guidance_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zip_guidance_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_zip_guidance_t, vz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZIP_GUIDANCE { \
    "ZIP_GUIDANCE", \
    6, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_zip_guidance_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_zip_guidance_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_zip_guidance_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_zip_guidance_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_zip_guidance_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_zip_guidance_t, vz) }, \
         } \
}
#endif

/**
 * @brief Pack a zip_guidance message
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zip_guidance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN);
#else
    mavlink_zip_guidance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZIP_GUIDANCE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC);
}

/**
 * @brief Pack a zip_guidance message on a channel
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zip_guidance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float vx,float vy,float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN);
#else
    mavlink_zip_guidance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZIP_GUIDANCE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC);
}

/**
 * @brief Encode a zip_guidance struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zip_guidance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zip_guidance_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zip_guidance_t* zip_guidance)
{
    return mavlink_msg_zip_guidance_pack(system_id, component_id, msg, zip_guidance->x, zip_guidance->y, zip_guidance->z, zip_guidance->vx, zip_guidance->vy, zip_guidance->vz);
}

/**
 * @brief Encode a zip_guidance struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zip_guidance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zip_guidance_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zip_guidance_t* zip_guidance)
{
    return mavlink_msg_zip_guidance_pack_chan(system_id, component_id, chan, msg, zip_guidance->x, zip_guidance->y, zip_guidance->z, zip_guidance->vx, zip_guidance->vy, zip_guidance->vz);
}

/**
 * @brief Send a zip_guidance message
 * @param chan MAVLink channel to send the message
 *
 * @param x [m] X Position in NED Frame
 * @param y [m] Y Position in NED Frame
 * @param z [m] Z Position in NED Frame
 * @param vx [m/s] X Velocity in NED Frame
 * @param vy [m/s] X Velocity in NED Frame
 * @param vz [m/s] X Velocity in NED Frame
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zip_guidance_send(mavlink_channel_t chan, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZIP_GUIDANCE, buf, MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC);
#else
    mavlink_zip_guidance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZIP_GUIDANCE, (const char *)&packet, MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC);
#endif
}

/**
 * @brief Send a zip_guidance message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zip_guidance_send_struct(mavlink_channel_t chan, const mavlink_zip_guidance_t* zip_guidance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zip_guidance_send(chan, zip_guidance->x, zip_guidance->y, zip_guidance->z, zip_guidance->vx, zip_guidance->vy, zip_guidance->vz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZIP_GUIDANCE, (const char *)zip_guidance, MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zip_guidance_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, vx);
    _mav_put_float(buf, 16, vy);
    _mav_put_float(buf, 20, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZIP_GUIDANCE, buf, MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC);
#else
    mavlink_zip_guidance_t *packet = (mavlink_zip_guidance_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZIP_GUIDANCE, (const char *)packet, MAVLINK_MSG_ID_ZIP_GUIDANCE_MIN_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN, MAVLINK_MSG_ID_ZIP_GUIDANCE_CRC);
#endif
}
#endif

#endif

// MESSAGE ZIP_GUIDANCE UNPACKING


/**
 * @brief Get field x from zip_guidance message
 *
 * @return [m] X Position in NED Frame
 */
static inline float mavlink_msg_zip_guidance_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from zip_guidance message
 *
 * @return [m] Y Position in NED Frame
 */
static inline float mavlink_msg_zip_guidance_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from zip_guidance message
 *
 * @return [m] Z Position in NED Frame
 */
static inline float mavlink_msg_zip_guidance_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vx from zip_guidance message
 *
 * @return [m/s] X Velocity in NED Frame
 */
static inline float mavlink_msg_zip_guidance_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vy from zip_guidance message
 *
 * @return [m/s] X Velocity in NED Frame
 */
static inline float mavlink_msg_zip_guidance_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vz from zip_guidance message
 *
 * @return [m/s] X Velocity in NED Frame
 */
static inline float mavlink_msg_zip_guidance_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a zip_guidance message into a struct
 *
 * @param msg The message to decode
 * @param zip_guidance C-struct to decode the message contents into
 */
static inline void mavlink_msg_zip_guidance_decode(const mavlink_message_t* msg, mavlink_zip_guidance_t* zip_guidance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zip_guidance->x = mavlink_msg_zip_guidance_get_x(msg);
    zip_guidance->y = mavlink_msg_zip_guidance_get_y(msg);
    zip_guidance->z = mavlink_msg_zip_guidance_get_z(msg);
    zip_guidance->vx = mavlink_msg_zip_guidance_get_vx(msg);
    zip_guidance->vy = mavlink_msg_zip_guidance_get_vy(msg);
    zip_guidance->vz = mavlink_msg_zip_guidance_get_vz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN? msg->len : MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN;
        memset(zip_guidance, 0, MAVLINK_MSG_ID_ZIP_GUIDANCE_LEN);
    memcpy(zip_guidance, _MAV_PAYLOAD(msg), len);
#endif
}
