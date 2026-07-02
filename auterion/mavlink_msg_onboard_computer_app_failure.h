#pragma once
// MESSAGE ONBOARD_COMPUTER_APP_FAILURE PACKING

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE 13802


typedef struct __mavlink_onboard_computer_app_failure_t {
 char app_name[128]; /*<  
        Name of the application
      */
 uint8_t failure_flags; /*<  
        Bitmask of failure reasons for this application
      */
 char expected_version[32]; /*<  
        Expected bundled version
      */
 char actual_version[32]; /*<  
        Actual installed/running version
      */
} mavlink_onboard_computer_app_failure_t;

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN 193
#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN 193
#define MAVLINK_MSG_ID_13802_LEN 193
#define MAVLINK_MSG_ID_13802_MIN_LEN 193

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC 166
#define MAVLINK_MSG_ID_13802_CRC 166

#define MAVLINK_MSG_ONBOARD_COMPUTER_APP_FAILURE_FIELD_APP_NAME_LEN 128
#define MAVLINK_MSG_ONBOARD_COMPUTER_APP_FAILURE_FIELD_EXPECTED_VERSION_LEN 32
#define MAVLINK_MSG_ONBOARD_COMPUTER_APP_FAILURE_FIELD_ACTUAL_VERSION_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ONBOARD_COMPUTER_APP_FAILURE { \
    13802, \
    "ONBOARD_COMPUTER_APP_FAILURE", \
    4, \
    {  { "app_name", NULL, MAVLINK_TYPE_CHAR, 128, 0, offsetof(mavlink_onboard_computer_app_failure_t, app_name) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 128, offsetof(mavlink_onboard_computer_app_failure_t, failure_flags) }, \
         { "expected_version", NULL, MAVLINK_TYPE_CHAR, 32, 129, offsetof(mavlink_onboard_computer_app_failure_t, expected_version) }, \
         { "actual_version", NULL, MAVLINK_TYPE_CHAR, 32, 161, offsetof(mavlink_onboard_computer_app_failure_t, actual_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ONBOARD_COMPUTER_APP_FAILURE { \
    "ONBOARD_COMPUTER_APP_FAILURE", \
    4, \
    {  { "app_name", NULL, MAVLINK_TYPE_CHAR, 128, 0, offsetof(mavlink_onboard_computer_app_failure_t, app_name) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 128, offsetof(mavlink_onboard_computer_app_failure_t, failure_flags) }, \
         { "expected_version", NULL, MAVLINK_TYPE_CHAR, 32, 129, offsetof(mavlink_onboard_computer_app_failure_t, expected_version) }, \
         { "actual_version", NULL, MAVLINK_TYPE_CHAR, 32, 161, offsetof(mavlink_onboard_computer_app_failure_t, actual_version) }, \
         } \
}
#endif

/**
 * @brief Pack a onboard_computer_app_failure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param app_name  
        Name of the application
      
 * @param failure_flags  
        Bitmask of failure reasons for this application
      
 * @param expected_version  
        Expected bundled version
      
 * @param actual_version  
        Actual installed/running version
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_app_failure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *app_name, uint8_t failure_flags, const char *expected_version, const char *actual_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN];
    _mav_put_uint8_t(buf, 128, failure_flags);
    _mav_put_char_array(buf, 0, app_name, 128);
    _mav_put_char_array(buf, 129, expected_version, 32);
    _mav_put_char_array(buf, 161, actual_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
#else
    mavlink_onboard_computer_app_failure_t packet;
    packet.failure_flags = failure_flags;
    mav_array_memcpy(packet.app_name, app_name, sizeof(char)*128);
    mav_array_memcpy(packet.expected_version, expected_version, sizeof(char)*32);
    mav_array_memcpy(packet.actual_version, actual_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
}

/**
 * @brief Pack a onboard_computer_app_failure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param app_name  
        Name of the application
      
 * @param failure_flags  
        Bitmask of failure reasons for this application
      
 * @param expected_version  
        Expected bundled version
      
 * @param actual_version  
        Actual installed/running version
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_computer_app_failure_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *app_name, uint8_t failure_flags, const char *expected_version, const char *actual_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN];
    _mav_put_uint8_t(buf, 128, failure_flags);
    _mav_put_char_array(buf, 0, app_name, 128);
    _mav_put_char_array(buf, 129, expected_version, 32);
    _mav_put_char_array(buf, 161, actual_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
#else
    mavlink_onboard_computer_app_failure_t packet;
    packet.failure_flags = failure_flags;
    mav_array_memcpy(packet.app_name, app_name, sizeof(char)*128);
    mav_array_memcpy(packet.expected_version, expected_version, sizeof(char)*32);
    mav_array_memcpy(packet.actual_version, actual_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
#endif
}

/**
 * @brief Pack a onboard_computer_app_failure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param app_name  
        Name of the application
      
 * @param failure_flags  
        Bitmask of failure reasons for this application
      
 * @param expected_version  
        Expected bundled version
      
 * @param actual_version  
        Actual installed/running version
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_app_failure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *app_name,uint8_t failure_flags,const char *expected_version,const char *actual_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN];
    _mav_put_uint8_t(buf, 128, failure_flags);
    _mav_put_char_array(buf, 0, app_name, 128);
    _mav_put_char_array(buf, 129, expected_version, 32);
    _mav_put_char_array(buf, 161, actual_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
#else
    mavlink_onboard_computer_app_failure_t packet;
    packet.failure_flags = failure_flags;
    mav_array_memcpy(packet.app_name, app_name, sizeof(char)*128);
    mav_array_memcpy(packet.expected_version, expected_version, sizeof(char)*32);
    mav_array_memcpy(packet.actual_version, actual_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
}

/**
 * @brief Encode a onboard_computer_app_failure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param onboard_computer_app_failure C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_app_failure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_onboard_computer_app_failure_t* onboard_computer_app_failure)
{
    return mavlink_msg_onboard_computer_app_failure_pack(system_id, component_id, msg, onboard_computer_app_failure->app_name, onboard_computer_app_failure->failure_flags, onboard_computer_app_failure->expected_version, onboard_computer_app_failure->actual_version);
}

/**
 * @brief Encode a onboard_computer_app_failure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_computer_app_failure C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_app_failure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_onboard_computer_app_failure_t* onboard_computer_app_failure)
{
    return mavlink_msg_onboard_computer_app_failure_pack_chan(system_id, component_id, chan, msg, onboard_computer_app_failure->app_name, onboard_computer_app_failure->failure_flags, onboard_computer_app_failure->expected_version, onboard_computer_app_failure->actual_version);
}

/**
 * @brief Encode a onboard_computer_app_failure struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param onboard_computer_app_failure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_computer_app_failure_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_onboard_computer_app_failure_t* onboard_computer_app_failure)
{
    return mavlink_msg_onboard_computer_app_failure_pack_status(system_id, component_id, _status, msg,  onboard_computer_app_failure->app_name, onboard_computer_app_failure->failure_flags, onboard_computer_app_failure->expected_version, onboard_computer_app_failure->actual_version);
}

/**
 * @brief Send a onboard_computer_app_failure message
 * @param chan MAVLink channel to send the message
 *
 * @param app_name  
        Name of the application
      
 * @param failure_flags  
        Bitmask of failure reasons for this application
      
 * @param expected_version  
        Expected bundled version
      
 * @param actual_version  
        Actual installed/running version
      
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_app_failure_send(mavlink_channel_t chan, const char *app_name, uint8_t failure_flags, const char *expected_version, const char *actual_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN];
    _mav_put_uint8_t(buf, 128, failure_flags);
    _mav_put_char_array(buf, 0, app_name, 128);
    _mav_put_char_array(buf, 129, expected_version, 32);
    _mav_put_char_array(buf, 161, actual_version, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE, buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
#else
    mavlink_onboard_computer_app_failure_t packet;
    packet.failure_flags = failure_flags;
    mav_array_memcpy(packet.app_name, app_name, sizeof(char)*128);
    mav_array_memcpy(packet.expected_version, expected_version, sizeof(char)*32);
    mav_array_memcpy(packet.actual_version, actual_version, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
#endif
}

/**
 * @brief Send a onboard_computer_app_failure message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_app_failure_send_struct(mavlink_channel_t chan, const mavlink_onboard_computer_app_failure_t* onboard_computer_app_failure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_onboard_computer_app_failure_send(chan, onboard_computer_app_failure->app_name, onboard_computer_app_failure->failure_flags, onboard_computer_app_failure->expected_version, onboard_computer_app_failure->actual_version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE, (const char *)onboard_computer_app_failure, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_app_failure_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *app_name, uint8_t failure_flags, const char *expected_version, const char *actual_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 128, failure_flags);
    _mav_put_char_array(buf, 0, app_name, 128);
    _mav_put_char_array(buf, 129, expected_version, 32);
    _mav_put_char_array(buf, 161, actual_version, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE, buf, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
#else
    mavlink_onboard_computer_app_failure_t *packet = (mavlink_onboard_computer_app_failure_t *)msgbuf;
    packet->failure_flags = failure_flags;
    mav_array_memcpy(packet->app_name, app_name, sizeof(char)*128);
    mav_array_memcpy(packet->expected_version, expected_version, sizeof(char)*32);
    mav_array_memcpy(packet->actual_version, actual_version, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_CRC);
#endif
}
#endif

#endif

// MESSAGE ONBOARD_COMPUTER_APP_FAILURE UNPACKING


/**
 * @brief Get field app_name from onboard_computer_app_failure message
 *
 * @return  
        Name of the application
      
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_app_failure_get_app_name(const mavlink_message_t* msg, char *app_name)
{
    return _MAV_RETURN_char_array(msg, app_name, 128,  0);
}

/**
 * @brief Get field failure_flags from onboard_computer_app_failure message
 *
 * @return  
        Bitmask of failure reasons for this application
      
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_onboard_computer_app_failure_get_failure_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  128);
}

/**
 * @brief Get field expected_version from onboard_computer_app_failure message
 *
 * @return  
        Expected bundled version
      
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_app_failure_get_expected_version(const mavlink_message_t* msg, char *expected_version)
{
    return _MAV_RETURN_char_array(msg, expected_version, 32,  129);
}

/**
 * @brief Get field actual_version from onboard_computer_app_failure message
 *
 * @return  
        Actual installed/running version
      
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_onboard_computer_app_failure_get_actual_version(const mavlink_message_t* msg, char *actual_version)
{
    return _MAV_RETURN_char_array(msg, actual_version, 32,  161);
}

/**
 * @brief Decode a onboard_computer_app_failure message into a struct
 *
 * @param msg The message to decode
 * @param onboard_computer_app_failure C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_onboard_computer_app_failure_decode(const mavlink_message_t* msg, mavlink_onboard_computer_app_failure_t* onboard_computer_app_failure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_onboard_computer_app_failure_get_app_name(msg, onboard_computer_app_failure->app_name);
    onboard_computer_app_failure->failure_flags = mavlink_msg_onboard_computer_app_failure_get_failure_flags(msg);
    mavlink_msg_onboard_computer_app_failure_get_expected_version(msg, onboard_computer_app_failure->expected_version);
    mavlink_msg_onboard_computer_app_failure_get_actual_version(msg, onboard_computer_app_failure->actual_version);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN? msg->len : MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN;
        memset(onboard_computer_app_failure, 0, MAVLINK_MSG_ID_ONBOARD_COMPUTER_APP_FAILURE_LEN);
    memcpy(onboard_computer_app_failure, _MAV_PAYLOAD(msg), len);
#endif
}
