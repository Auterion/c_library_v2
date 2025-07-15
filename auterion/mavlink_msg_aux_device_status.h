#pragma once
// MESSAGE AUX_DEVICE_STATUS PACKING

#define MAVLINK_MSG_ID_AUX_DEVICE_STATUS 13674


typedef struct __mavlink_aux_device_status_t {
 char status[200]; /*<  Status, for human-friendly display in a Ground Control Station*/
} mavlink_aux_device_status_t;

#define MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN 200
#define MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN 200
#define MAVLINK_MSG_ID_13674_LEN 200
#define MAVLINK_MSG_ID_13674_MIN_LEN 200

#define MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC 230
#define MAVLINK_MSG_ID_13674_CRC 230

#define MAVLINK_MSG_AUX_DEVICE_STATUS_FIELD_STATUS_LEN 200

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AUX_DEVICE_STATUS { \
    13674, \
    "AUX_DEVICE_STATUS", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_CHAR, 200, 0, offsetof(mavlink_aux_device_status_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AUX_DEVICE_STATUS { \
    "AUX_DEVICE_STATUS", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_CHAR, 200, 0, offsetof(mavlink_aux_device_status_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a aux_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status, for human-friendly display in a Ground Control Station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aux_device_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
#else
    mavlink_aux_device_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUX_DEVICE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
}

/**
 * @brief Pack a aux_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status, for human-friendly display in a Ground Control Station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aux_device_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
#else
    mavlink_aux_device_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUX_DEVICE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a aux_device_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  Status, for human-friendly display in a Ground Control Station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aux_device_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
#else
    mavlink_aux_device_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUX_DEVICE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
}

/**
 * @brief Encode a aux_device_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aux_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aux_device_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aux_device_status_t* aux_device_status)
{
    return mavlink_msg_aux_device_status_pack(system_id, component_id, msg, aux_device_status->status);
}

/**
 * @brief Encode a aux_device_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aux_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aux_device_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aux_device_status_t* aux_device_status)
{
    return mavlink_msg_aux_device_status_pack_chan(system_id, component_id, chan, msg, aux_device_status->status);
}

/**
 * @brief Encode a aux_device_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param aux_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aux_device_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_aux_device_status_t* aux_device_status)
{
    return mavlink_msg_aux_device_status_pack_status(system_id, component_id, _status, msg,  aux_device_status->status);
}

/**
 * @brief Send a aux_device_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status  Status, for human-friendly display in a Ground Control Station
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aux_device_status_send(mavlink_channel_t chan, const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN];

    _mav_put_char_array(buf, 0, status, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUX_DEVICE_STATUS, buf, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
#else
    mavlink_aux_device_status_t packet;

    mav_array_memcpy(packet.status, status, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUX_DEVICE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
#endif
}

/**
 * @brief Send a aux_device_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_aux_device_status_send_struct(mavlink_channel_t chan, const mavlink_aux_device_status_t* aux_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aux_device_status_send(chan, aux_device_status->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUX_DEVICE_STATUS, (const char *)aux_device_status, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_aux_device_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, status, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUX_DEVICE_STATUS, buf, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
#else
    mavlink_aux_device_status_t *packet = (mavlink_aux_device_status_t *)msgbuf;

    mav_array_memcpy(packet->status, status, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUX_DEVICE_STATUS, (const char *)packet, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE AUX_DEVICE_STATUS UNPACKING


/**
 * @brief Get field status from aux_device_status message
 *
 * @return  Status, for human-friendly display in a Ground Control Station
 */
static inline uint16_t mavlink_msg_aux_device_status_get_status(const mavlink_message_t* msg, char *status)
{
    return _MAV_RETURN_char_array(msg, status, 200,  0);
}

/**
 * @brief Decode a aux_device_status message into a struct
 *
 * @param msg The message to decode
 * @param aux_device_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_aux_device_status_decode(const mavlink_message_t* msg, mavlink_aux_device_status_t* aux_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aux_device_status_get_status(msg, aux_device_status->status);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN;
        memset(aux_device_status, 0, MAVLINK_MSG_ID_AUX_DEVICE_STATUS_LEN);
    memcpy(aux_device_status, _MAV_PAYLOAD(msg), len);
#endif
}
