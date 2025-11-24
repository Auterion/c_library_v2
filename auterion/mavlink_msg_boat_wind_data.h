#pragma once
// MESSAGE BOAT_WIND_DATA PACKING

#define MAVLINK_MSG_ID_BOAT_WIND_DATA 672


typedef struct __mavlink_boat_wind_data_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float wind_speed; /*< [m/s] Wind speed.*/
 float wind_angle; /*< [rad] Wing_angle.*/
 uint8_t reference; /*<  Wind reference.*/
} mavlink_boat_wind_data_t;

#define MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN 17
#define MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN 17
#define MAVLINK_MSG_ID_672_LEN 17
#define MAVLINK_MSG_ID_672_MIN_LEN 17

#define MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC 99
#define MAVLINK_MSG_ID_672_CRC 99



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BOAT_WIND_DATA { \
    672, \
    "BOAT_WIND_DATA", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_boat_wind_data_t, time_usec) }, \
         { "reference", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_boat_wind_data_t, reference) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_boat_wind_data_t, wind_speed) }, \
         { "wind_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_boat_wind_data_t, wind_angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BOAT_WIND_DATA { \
    "BOAT_WIND_DATA", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_boat_wind_data_t, time_usec) }, \
         { "reference", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_boat_wind_data_t, reference) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_boat_wind_data_t, wind_speed) }, \
         { "wind_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_boat_wind_data_t, wind_angle) }, \
         } \
}
#endif

/**
 * @brief Pack a boat_wind_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param reference  Wind reference.
 * @param wind_speed [m/s] Wind speed.
 * @param wind_angle [rad] Wing_angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_wind_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t reference, float wind_speed, float wind_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_speed);
    _mav_put_float(buf, 12, wind_angle);
    _mav_put_uint8_t(buf, 16, reference);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
#else
    mavlink_boat_wind_data_t packet;
    packet.time_usec = time_usec;
    packet.wind_speed = wind_speed;
    packet.wind_angle = wind_angle;
    packet.reference = reference;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_WIND_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
}

/**
 * @brief Pack a boat_wind_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param reference  Wind reference.
 * @param wind_speed [m/s] Wind speed.
 * @param wind_angle [rad] Wing_angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boat_wind_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t reference, float wind_speed, float wind_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_speed);
    _mav_put_float(buf, 12, wind_angle);
    _mav_put_uint8_t(buf, 16, reference);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
#else
    mavlink_boat_wind_data_t packet;
    packet.time_usec = time_usec;
    packet.wind_speed = wind_speed;
    packet.wind_angle = wind_angle;
    packet.reference = reference;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_WIND_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
#endif
}

/**
 * @brief Pack a boat_wind_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param reference  Wind reference.
 * @param wind_speed [m/s] Wind speed.
 * @param wind_angle [rad] Wing_angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_wind_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t reference,float wind_speed,float wind_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_speed);
    _mav_put_float(buf, 12, wind_angle);
    _mav_put_uint8_t(buf, 16, reference);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
#else
    mavlink_boat_wind_data_t packet;
    packet.time_usec = time_usec;
    packet.wind_speed = wind_speed;
    packet.wind_angle = wind_angle;
    packet.reference = reference;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_WIND_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
}

/**
 * @brief Encode a boat_wind_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boat_wind_data C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_wind_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boat_wind_data_t* boat_wind_data)
{
    return mavlink_msg_boat_wind_data_pack(system_id, component_id, msg, boat_wind_data->time_usec, boat_wind_data->reference, boat_wind_data->wind_speed, boat_wind_data->wind_angle);
}

/**
 * @brief Encode a boat_wind_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boat_wind_data C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_wind_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_boat_wind_data_t* boat_wind_data)
{
    return mavlink_msg_boat_wind_data_pack_chan(system_id, component_id, chan, msg, boat_wind_data->time_usec, boat_wind_data->reference, boat_wind_data->wind_speed, boat_wind_data->wind_angle);
}

/**
 * @brief Encode a boat_wind_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param boat_wind_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boat_wind_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_boat_wind_data_t* boat_wind_data)
{
    return mavlink_msg_boat_wind_data_pack_status(system_id, component_id, _status, msg,  boat_wind_data->time_usec, boat_wind_data->reference, boat_wind_data->wind_speed, boat_wind_data->wind_angle);
}

/**
 * @brief Send a boat_wind_data message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param reference  Wind reference.
 * @param wind_speed [m/s] Wind speed.
 * @param wind_angle [rad] Wing_angle.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_boat_wind_data_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t reference, float wind_speed, float wind_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_speed);
    _mav_put_float(buf, 12, wind_angle);
    _mav_put_uint8_t(buf, 16, reference);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_WIND_DATA, buf, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
#else
    mavlink_boat_wind_data_t packet;
    packet.time_usec = time_usec;
    packet.wind_speed = wind_speed;
    packet.wind_angle = wind_angle;
    packet.reference = reference;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_WIND_DATA, (const char *)&packet, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
#endif
}

/**
 * @brief Send a boat_wind_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_wind_data_send_struct(mavlink_channel_t chan, const mavlink_boat_wind_data_t* boat_wind_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_boat_wind_data_send(chan, boat_wind_data->time_usec, boat_wind_data->reference, boat_wind_data->wind_speed, boat_wind_data->wind_angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_WIND_DATA, (const char *)boat_wind_data, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_wind_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t reference, float wind_speed, float wind_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_speed);
    _mav_put_float(buf, 12, wind_angle);
    _mav_put_uint8_t(buf, 16, reference);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_WIND_DATA, buf, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
#else
    mavlink_boat_wind_data_t *packet = (mavlink_boat_wind_data_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->wind_speed = wind_speed;
    packet->wind_angle = wind_angle;
    packet->reference = reference;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_WIND_DATA, (const char *)packet, MAVLINK_MSG_ID_BOAT_WIND_DATA_MIN_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN, MAVLINK_MSG_ID_BOAT_WIND_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE BOAT_WIND_DATA UNPACKING


/**
 * @brief Get field time_usec from boat_wind_data message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
MAVLINK_WIP
static inline uint64_t mavlink_msg_boat_wind_data_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field reference from boat_wind_data message
 *
 * @return  Wind reference.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_boat_wind_data_get_reference(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field wind_speed from boat_wind_data message
 *
 * @return [m/s] Wind speed.
 */
MAVLINK_WIP
static inline float mavlink_msg_boat_wind_data_get_wind_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field wind_angle from boat_wind_data message
 *
 * @return [rad] Wing_angle.
 */
MAVLINK_WIP
static inline float mavlink_msg_boat_wind_data_get_wind_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a boat_wind_data message into a struct
 *
 * @param msg The message to decode
 * @param boat_wind_data C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_wind_data_decode(const mavlink_message_t* msg, mavlink_boat_wind_data_t* boat_wind_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    boat_wind_data->time_usec = mavlink_msg_boat_wind_data_get_time_usec(msg);
    boat_wind_data->wind_speed = mavlink_msg_boat_wind_data_get_wind_speed(msg);
    boat_wind_data->wind_angle = mavlink_msg_boat_wind_data_get_wind_angle(msg);
    boat_wind_data->reference = mavlink_msg_boat_wind_data_get_reference(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN? msg->len : MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN;
        memset(boat_wind_data, 0, MAVLINK_MSG_ID_BOAT_WIND_DATA_LEN);
    memcpy(boat_wind_data, _MAV_PAYLOAD(msg), len);
#endif
}
