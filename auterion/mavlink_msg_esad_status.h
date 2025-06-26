#pragma once
// MESSAGE ESAD_STATUS PACKING

#define MAVLINK_MSG_ID_ESAD_STATUS 13674


typedef struct __mavlink_esad_status_t {
 char status[200]; /*<  Status, for human-friendly display in a Ground Control Station*/
} mavlink_esad_status_t;

#define MAVLINK_MSG_ID_ESAD_STATUS_LEN 200
#define MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN 200
#define MAVLINK_MSG_ID_13674_LEN 200
#define MAVLINK_MSG_ID_13674_MIN_LEN 200

#define MAVLINK_MSG_ID_ESAD_STATUS_CRC 16
#define MAVLINK_MSG_ID_13674_CRC 16

#define MAVLINK_MSG_ESAD_STATUS_FIELD_STATUS_LEN 200

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ESAD_STATUS { \
    13674, \
    "ESAD_STATUS", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_CHAR, 200, 0, offsetof(mavlink_esad_status_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ESAD_STATUS { \
    "ESAD_STATUS", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_CHAR, 200, 0, offsetof(mavlink_esad_status_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a esad_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status, for human-friendly display in a Ground Control Station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esad_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESAD_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
#else
    mavlink_esad_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESAD_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
}

/**
 * @brief Pack a esad_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status, for human-friendly display in a Ground Control Station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esad_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESAD_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
#else
    mavlink_esad_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESAD_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
#endif
}

/**
 * @brief Pack a esad_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  Status, for human-friendly display in a Ground Control Station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esad_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESAD_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
#else
    mavlink_esad_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESAD_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
}

/**
 * @brief Encode a esad_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param esad_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esad_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_esad_status_t* esad_status)
{
    return mavlink_msg_esad_status_pack(system_id, component_id, msg, esad_status->status);
}

/**
 * @brief Encode a esad_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esad_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esad_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_esad_status_t* esad_status)
{
    return mavlink_msg_esad_status_pack_chan(system_id, component_id, chan, msg, esad_status->status);
}

/**
 * @brief Encode a esad_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param esad_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esad_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_esad_status_t* esad_status)
{
    return mavlink_msg_esad_status_pack_status(system_id, component_id, _status, msg,  esad_status->status);
}

/**
 * @brief Send a esad_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status  Status, for human-friendly display in a Ground Control Station
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_esad_status_send(mavlink_channel_t chan, const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESAD_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESAD_STATUS, buf, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
#else
    mavlink_esad_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESAD_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
#endif
}

/**
 * @brief Send a esad_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_esad_status_send_struct(mavlink_channel_t chan, const mavlink_esad_status_t* esad_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esad_status_send(chan, esad_status->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESAD_STATUS, (const char *)esad_status, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ESAD_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_esad_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, status, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESAD_STATUS, buf, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
#else
    mavlink_esad_status_t *packet = (mavlink_esad_status_t *)msgbuf;

    mav_array_memcpy(packet->status, status, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESAD_STATUS, (const char *)packet, MAVLINK_MSG_ID_ESAD_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESAD_STATUS_LEN, MAVLINK_MSG_ID_ESAD_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ESAD_STATUS UNPACKING


/**
 * @brief Get field status from esad_status message
 *
 * @return  Status, for human-friendly display in a Ground Control Station
 */
static inline uint16_t mavlink_msg_esad_status_get_status(const mavlink_message_t* msg, char *status)
{
    return _MAV_RETURN_char_array(msg, status, 200,  0);
}

/**
 * @brief Decode a esad_status message into a struct
 *
 * @param msg The message to decode
 * @param esad_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_esad_status_decode(const mavlink_message_t* msg, mavlink_esad_status_t* esad_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esad_status_get_status(msg, esad_status->status);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ESAD_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ESAD_STATUS_LEN;
        memset(esad_status, 0, MAVLINK_MSG_ID_ESAD_STATUS_LEN);
    memcpy(esad_status, _MAV_PAYLOAD(msg), len);
#endif
}
