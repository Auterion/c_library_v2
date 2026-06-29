#pragma once
// MESSAGE ONBOARD_COMPUTER_APPS_STATUS PACKING

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS 13801


typedef struct __mavlink_onboard_computer_apps_status_t {
 uint8_t overall_health; /*<  
        Overall system health state
      */
 uint8_t failure_flags; /*<  
        Bitmask of detected failure types across all default bundled apps
      */
 uint8_t failing_apps_count; /*<  
        Number of default bundled apps in failure state
      */
} mavlink_onboard_computer_apps_status_t;

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN 3
#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN 3
#define MAVLINK_MSG_ID_13801_LEN 3
#define MAVLINK_MSG_ID_13801_MIN_LEN 3

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC 251
#define MAVLINK_MSG_ID_13801_CRC 251



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ONBOARD_COMPUTER_APPS_STATUS { \
    13801, \
    "ONBOARD_COMPUTER_APPS_STATUS", \
    3, \
    {  { "overall_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_onboard_computer_apps_status_t, overall_health) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_onboard_computer_apps_status_t, failure_flags) }, \
         { "failing_apps_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_onboard_computer_apps_status_t, failing_apps_count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ONBOARD_COMPUTER_APPS_STATUS { \
    "ONBOARD_COMPUTER_APPS_STATUS", \
    3, \
    {  { "overall_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_onboard_computer_apps_status_t, overall_health) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_onboard_computer_apps_status_t, failure_flags) }, \
         { "failing_apps_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_onboard_computer_apps_status_t, failing_apps_count) }, \
         } \
}
#endif

/**
 * @brief Pack a onboard_computer_apps_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param failing_apps_count  
        Number of default bundled apps in failure state
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_apps_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t overall_health, uint8_t failure_flags, uint8_t failing_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, failing_apps_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
#else
    mavlink_onboard_computer_apps_status_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.failing_apps_count = failing_apps_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
}

/**
 * @brief Pack a onboard_computer_apps_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param failing_apps_count  
        Number of default bundled apps in failure state
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_computer_apps_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t overall_health, uint8_t failure_flags, uint8_t failing_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, failing_apps_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
#else
    mavlink_onboard_computer_apps_status_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.failing_apps_count = failing_apps_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
#endif
}

/**
 * @brief Pack a onboard_computer_apps_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param failing_apps_count  
        Number of default bundled apps in failure state
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_apps_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t overall_health,uint8_t failure_flags,uint8_t failing_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, failing_apps_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
#else
    mavlink_onboard_computer_apps_status_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.failing_apps_count = failing_apps_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
}

/**
 * @brief Encode a onboard_computer_apps_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param onboard_computer_apps_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_apps_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_onboard_computer_apps_status_t* onboard_computer_apps_status)
{
    return mavlink_msg_onboard_computer_apps_status_pack(system_id, component_id, msg, onboard_computer_apps_status->overall_health, onboard_computer_apps_status->failure_flags, onboard_computer_apps_status->failing_apps_count);
}

/**
 * @brief Encode a onboard_computer_apps_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_computer_apps_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_apps_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_onboard_computer_apps_status_t* onboard_computer_apps_status)
{
    return mavlink_msg_onboard_computer_apps_status_pack_chan(system_id, component_id, chan, msg, onboard_computer_apps_status->overall_health, onboard_computer_apps_status->failure_flags, onboard_computer_apps_status->failing_apps_count);
}

/**
 * @brief Encode a onboard_computer_apps_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param onboard_computer_apps_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_computer_apps_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_onboard_computer_apps_status_t* onboard_computer_apps_status)
{
    return mavlink_msg_onboard_computer_apps_status_pack_status(system_id, component_id, _status, msg,  onboard_computer_apps_status->overall_health, onboard_computer_apps_status->failure_flags, onboard_computer_apps_status->failing_apps_count);
}

/**
 * @brief Send a onboard_computer_apps_status message
 * @param chan MAVLink channel to send the message
 *
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param failing_apps_count  
        Number of default bundled apps in failure state
      
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_apps_status_send(mavlink_channel_t chan, uint8_t overall_health, uint8_t failure_flags, uint8_t failing_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, failing_apps_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS, buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
#else
    mavlink_onboard_computer_apps_status_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.failing_apps_count = failing_apps_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
#endif
}

/**
 * @brief Send a onboard_computer_apps_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_apps_status_send_struct(mavlink_channel_t chan, const mavlink_onboard_computer_apps_status_t* onboard_computer_apps_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_onboard_computer_apps_status_send(chan, onboard_computer_apps_status->overall_health, onboard_computer_apps_status->failure_flags, onboard_computer_apps_status->failing_apps_count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS, (const char *)onboard_computer_apps_status, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_apps_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t overall_health, uint8_t failure_flags, uint8_t failing_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, failing_apps_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS, buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
#else
    mavlink_onboard_computer_apps_status_t *packet = (mavlink_onboard_computer_apps_status_t *)msgbuf;
    packet->overall_health = overall_health;
    packet->failure_flags = failure_flags;
    packet->failing_apps_count = failing_apps_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ONBOARD_COMPUTER_APPS_STATUS UNPACKING


/**
 * @brief Get field overall_health from onboard_computer_apps_status message
 *
 * @return  
        Overall system health state
      
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_onboard_computer_apps_status_get_overall_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field failure_flags from onboard_computer_apps_status message
 *
 * @return  
        Bitmask of detected failure types across all default bundled apps
      
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_onboard_computer_apps_status_get_failure_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field failing_apps_count from onboard_computer_apps_status message
 *
 * @return  
        Number of default bundled apps in failure state
      
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_onboard_computer_apps_status_get_failing_apps_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a onboard_computer_apps_status message into a struct
 *
 * @param msg The message to decode
 * @param onboard_computer_apps_status C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_apps_status_decode(const mavlink_message_t* msg, mavlink_onboard_computer_apps_status_t* onboard_computer_apps_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    onboard_computer_apps_status->overall_health = mavlink_msg_onboard_computer_apps_status_get_overall_health(msg);
    onboard_computer_apps_status->failure_flags = mavlink_msg_onboard_computer_apps_status_get_failure_flags(msg);
    onboard_computer_apps_status->failing_apps_count = mavlink_msg_onboard_computer_apps_status_get_failing_apps_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN;
        memset(onboard_computer_apps_status, 0, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APPS_STATUS_LEN);
    memcpy(onboard_computer_apps_status, _MAV_PAYLOAD(msg), len);
#endif
}
