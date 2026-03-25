#pragma once
// MESSAGE PAYLOAD_DEVICE_STATUS PACKING

#define MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS 13800


typedef struct __mavlink_payload_device_status_t {
 uint32_t countdown; /*< [s] Countdown timer value for SST or other timed state.*/
 uint8_t state; /*<  Device state.*/
 uint8_t mode; /*<  Mode; this can be a sub-state within the current state or other indicator. This may vary based on device and state.*/
 char message[64]; /*<  Message, or other text from device.*/
} mavlink_payload_device_status_t;

#define MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN 70
#define MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN 70
#define MAVLINK_MSG_ID_13800_LEN 70
#define MAVLINK_MSG_ID_13800_MIN_LEN 70

#define MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC 90
#define MAVLINK_MSG_ID_13800_CRC 90

#define MAVLINK_MSG_PAYLOAD_DEVICE_STATUS_FIELD_MESSAGE_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PAYLOAD_DEVICE_STATUS { \
    13800, \
    "PAYLOAD_DEVICE_STATUS", \
    4, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_payload_device_status_t, state) }, \
         { "countdown", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_payload_device_status_t, countdown) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_payload_device_status_t, mode) }, \
         { "message", NULL, MAVLINK_TYPE_CHAR, 64, 6, offsetof(mavlink_payload_device_status_t, message) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PAYLOAD_DEVICE_STATUS { \
    "PAYLOAD_DEVICE_STATUS", \
    4, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_payload_device_status_t, state) }, \
         { "countdown", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_payload_device_status_t, countdown) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_payload_device_status_t, mode) }, \
         { "message", NULL, MAVLINK_TYPE_CHAR, 64, 6, offsetof(mavlink_payload_device_status_t, message) }, \
         } \
}
#endif

/**
 * @brief Pack a payload_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  Device state.
 * @param countdown [s] Countdown timer value for SST or other timed state.
 * @param mode  Mode; this can be a sub-state within the current state or other indicator. This may vary based on device and state.
 * @param message  Message, or other text from device.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_payload_device_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t state, uint32_t countdown, uint8_t mode, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, countdown);
    _mav_put_uint8_t(buf, 4, state);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_char_array(buf, 6, message, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
#else
    mavlink_payload_device_status_t packet;
    packet.countdown = countdown;
    packet.state = state;
    packet.mode = mode;
    mav_array_memcpy(packet.message, message, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
}

/**
 * @brief Pack a payload_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  Device state.
 * @param countdown [s] Countdown timer value for SST or other timed state.
 * @param mode  Mode; this can be a sub-state within the current state or other indicator. This may vary based on device and state.
 * @param message  Message, or other text from device.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_device_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t state, uint32_t countdown, uint8_t mode, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, countdown);
    _mav_put_uint8_t(buf, 4, state);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_char_array(buf, 6, message, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
#else
    mavlink_payload_device_status_t packet;
    packet.countdown = countdown;
    packet.state = state;
    packet.mode = mode;
    mav_array_memcpy(packet.message, message, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a payload_device_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state  Device state.
 * @param countdown [s] Countdown timer value for SST or other timed state.
 * @param mode  Mode; this can be a sub-state within the current state or other indicator. This may vary based on device and state.
 * @param message  Message, or other text from device.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_payload_device_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t state,uint32_t countdown,uint8_t mode,const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, countdown);
    _mav_put_uint8_t(buf, 4, state);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_char_array(buf, 6, message, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
#else
    mavlink_payload_device_status_t packet;
    packet.countdown = countdown;
    packet.state = state;
    packet.mode = mode;
    mav_array_memcpy(packet.message, message, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
}

/**
 * @brief Encode a payload_device_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param payload_device_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_payload_device_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_payload_device_status_t* payload_device_status)
{
    return mavlink_msg_payload_device_status_pack(system_id, component_id, msg, payload_device_status->state, payload_device_status->countdown, payload_device_status->mode, payload_device_status->message);
}

/**
 * @brief Encode a payload_device_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param payload_device_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_payload_device_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_payload_device_status_t* payload_device_status)
{
    return mavlink_msg_payload_device_status_pack_chan(system_id, component_id, chan, msg, payload_device_status->state, payload_device_status->countdown, payload_device_status->mode, payload_device_status->message);
}

/**
 * @brief Encode a payload_device_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param payload_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_device_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_payload_device_status_t* payload_device_status)
{
    return mavlink_msg_payload_device_status_pack_status(system_id, component_id, _status, msg,  payload_device_status->state, payload_device_status->countdown, payload_device_status->mode, payload_device_status->message);
}

/**
 * @brief Send a payload_device_status message
 * @param chan MAVLink channel to send the message
 *
 * @param state  Device state.
 * @param countdown [s] Countdown timer value for SST or other timed state.
 * @param mode  Mode; this can be a sub-state within the current state or other indicator. This may vary based on device and state.
 * @param message  Message, or other text from device.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_payload_device_status_send(mavlink_channel_t chan, uint8_t state, uint32_t countdown, uint8_t mode, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, countdown);
    _mav_put_uint8_t(buf, 4, state);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_char_array(buf, 6, message, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS, buf, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
#else
    mavlink_payload_device_status_t packet;
    packet.countdown = countdown;
    packet.state = state;
    packet.mode = mode;
    mav_array_memcpy(packet.message, message, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
#endif
}

/**
 * @brief Send a payload_device_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_payload_device_status_send_struct(mavlink_channel_t chan, const mavlink_payload_device_status_t* payload_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_payload_device_status_send(chan, payload_device_status->state, payload_device_status->countdown, payload_device_status->mode, payload_device_status->message);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS, (const char *)payload_device_status, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_payload_device_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t state, uint32_t countdown, uint8_t mode, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, countdown);
    _mav_put_uint8_t(buf, 4, state);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_char_array(buf, 6, message, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS, buf, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
#else
    mavlink_payload_device_status_t *packet = (mavlink_payload_device_status_t *)msgbuf;
    packet->countdown = countdown;
    packet->state = state;
    packet->mode = mode;
    mav_array_memcpy(packet->message, message, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS, (const char *)packet, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE PAYLOAD_DEVICE_STATUS UNPACKING


/**
 * @brief Get field state from payload_device_status message
 *
 * @return  Device state.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_payload_device_status_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field countdown from payload_device_status message
 *
 * @return [s] Countdown timer value for SST or other timed state.
 */
MAVLINK_WIP
static inline uint32_t mavlink_msg_payload_device_status_get_countdown(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mode from payload_device_status message
 *
 * @return  Mode; this can be a sub-state within the current state or other indicator. This may vary based on device and state.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_payload_device_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field message from payload_device_status message
 *
 * @return  Message, or other text from device.
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_payload_device_status_get_message(const mavlink_message_t* msg, char *message)
{
    return _MAV_RETURN_char_array(msg, message, 64,  6);
}

/**
 * @brief Decode a payload_device_status message into a struct
 *
 * @param msg The message to decode
 * @param payload_device_status C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_payload_device_status_decode(const mavlink_message_t* msg, mavlink_payload_device_status_t* payload_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    payload_device_status->countdown = mavlink_msg_payload_device_status_get_countdown(msg);
    payload_device_status->state = mavlink_msg_payload_device_status_get_state(msg);
    payload_device_status->mode = mavlink_msg_payload_device_status_get_mode(msg);
    mavlink_msg_payload_device_status_get_message(msg, payload_device_status->message);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN;
        memset(payload_device_status, 0, MAVLINK_MSG_ID_PAYLOAD_DEVICE_STATUS_LEN);
    memcpy(payload_device_status, _MAV_PAYLOAD(msg), len);
#endif
}
