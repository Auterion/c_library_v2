#pragma once
// MESSAGE AOS_BUNDLED_APP_HEALTH_SUMMARY PACKING

#define MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY 13801


typedef struct __mavlink_aos_bundled_app_health_summary_t {
 uint8_t overall_health; /*<  
        Overall system health state
      */
 uint8_t failure_flags; /*<  
        Bitmask of detected failure types across all default bundled apps
      */
 uint8_t total_apps_count; /*<  
        Total number of default bundled apps tracked (number of
        AOS_BUNDLED_APP_HEALTH messages to expect on request)
      */
} mavlink_aos_bundled_app_health_summary_t;

#define MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN 3
#define MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN 3
#define MAVLINK_MSG_ID_13801_LEN 3
#define MAVLINK_MSG_ID_13801_MIN_LEN 3

#define MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC 139
#define MAVLINK_MSG_ID_13801_CRC 139



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AOS_BUNDLED_APP_HEALTH_SUMMARY { \
    13801, \
    "AOS_BUNDLED_APP_HEALTH_SUMMARY", \
    3, \
    {  { "overall_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_aos_bundled_app_health_summary_t, overall_health) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_aos_bundled_app_health_summary_t, failure_flags) }, \
         { "total_apps_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_aos_bundled_app_health_summary_t, total_apps_count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AOS_BUNDLED_APP_HEALTH_SUMMARY { \
    "AOS_BUNDLED_APP_HEALTH_SUMMARY", \
    3, \
    {  { "overall_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_aos_bundled_app_health_summary_t, overall_health) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_aos_bundled_app_health_summary_t, failure_flags) }, \
         { "total_apps_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_aos_bundled_app_health_summary_t, total_apps_count) }, \
         } \
}
#endif

/**
 * @brief Pack a aos_bundled_app_health_summary message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param total_apps_count  
        Total number of default bundled apps tracked (number of
        AOS_BUNDLED_APP_HEALTH messages to expect on request)
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_aos_bundled_app_health_summary_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t overall_health, uint8_t failure_flags, uint8_t total_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, total_apps_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
#else
    mavlink_aos_bundled_app_health_summary_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.total_apps_count = total_apps_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
}

/**
 * @brief Pack a aos_bundled_app_health_summary message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param total_apps_count  
        Total number of default bundled apps tracked (number of
        AOS_BUNDLED_APP_HEALTH messages to expect on request)
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aos_bundled_app_health_summary_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t overall_health, uint8_t failure_flags, uint8_t total_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, total_apps_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
#else
    mavlink_aos_bundled_app_health_summary_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.total_apps_count = total_apps_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
#endif
}

/**
 * @brief Pack a aos_bundled_app_health_summary message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param total_apps_count  
        Total number of default bundled apps tracked (number of
        AOS_BUNDLED_APP_HEALTH messages to expect on request)
      
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_aos_bundled_app_health_summary_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t overall_health,uint8_t failure_flags,uint8_t total_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, total_apps_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
#else
    mavlink_aos_bundled_app_health_summary_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.total_apps_count = total_apps_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
}

/**
 * @brief Encode a aos_bundled_app_health_summary struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aos_bundled_app_health_summary C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_aos_bundled_app_health_summary_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aos_bundled_app_health_summary_t* aos_bundled_app_health_summary)
{
    return mavlink_msg_aos_bundled_app_health_summary_pack(system_id, component_id, msg, aos_bundled_app_health_summary->overall_health, aos_bundled_app_health_summary->failure_flags, aos_bundled_app_health_summary->total_apps_count);
}

/**
 * @brief Encode a aos_bundled_app_health_summary struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aos_bundled_app_health_summary C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_aos_bundled_app_health_summary_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aos_bundled_app_health_summary_t* aos_bundled_app_health_summary)
{
    return mavlink_msg_aos_bundled_app_health_summary_pack_chan(system_id, component_id, chan, msg, aos_bundled_app_health_summary->overall_health, aos_bundled_app_health_summary->failure_flags, aos_bundled_app_health_summary->total_apps_count);
}

/**
 * @brief Encode a aos_bundled_app_health_summary struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param aos_bundled_app_health_summary C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aos_bundled_app_health_summary_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_aos_bundled_app_health_summary_t* aos_bundled_app_health_summary)
{
    return mavlink_msg_aos_bundled_app_health_summary_pack_status(system_id, component_id, _status, msg,  aos_bundled_app_health_summary->overall_health, aos_bundled_app_health_summary->failure_flags, aos_bundled_app_health_summary->total_apps_count);
}

/**
 * @brief Send a aos_bundled_app_health_summary message
 * @param chan MAVLink channel to send the message
 *
 * @param overall_health  
        Overall system health state
      
 * @param failure_flags  
        Bitmask of detected failure types across all default bundled apps
      
 * @param total_apps_count  
        Total number of default bundled apps tracked (number of
        AOS_BUNDLED_APP_HEALTH messages to expect on request)
      
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_aos_bundled_app_health_summary_send(mavlink_channel_t chan, uint8_t overall_health, uint8_t failure_flags, uint8_t total_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN];
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, total_apps_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY, buf, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
#else
    mavlink_aos_bundled_app_health_summary_t packet;
    packet.overall_health = overall_health;
    packet.failure_flags = failure_flags;
    packet.total_apps_count = total_apps_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY, (const char *)&packet, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
#endif
}

/**
 * @brief Send a aos_bundled_app_health_summary message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_aos_bundled_app_health_summary_send_struct(mavlink_channel_t chan, const mavlink_aos_bundled_app_health_summary_t* aos_bundled_app_health_summary)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aos_bundled_app_health_summary_send(chan, aos_bundled_app_health_summary->overall_health, aos_bundled_app_health_summary->failure_flags, aos_bundled_app_health_summary->total_apps_count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY, (const char *)aos_bundled_app_health_summary, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
#endif
}

#if MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_aos_bundled_app_health_summary_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t overall_health, uint8_t failure_flags, uint8_t total_apps_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, overall_health);
    _mav_put_uint8_t(buf, 1, failure_flags);
    _mav_put_uint8_t(buf, 2, total_apps_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY, buf, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
#else
    mavlink_aos_bundled_app_health_summary_t *packet = (mavlink_aos_bundled_app_health_summary_t *)msgbuf;
    packet->overall_health = overall_health;
    packet->failure_flags = failure_flags;
    packet->total_apps_count = total_apps_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY, (const char *)packet, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_MIN_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_CRC);
#endif
}
#endif

#endif

// MESSAGE AOS_BUNDLED_APP_HEALTH_SUMMARY UNPACKING


/**
 * @brief Get field overall_health from aos_bundled_app_health_summary message
 *
 * @return  
        Overall system health state
      
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_aos_bundled_app_health_summary_get_overall_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field failure_flags from aos_bundled_app_health_summary message
 *
 * @return  
        Bitmask of detected failure types across all default bundled apps
      
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_aos_bundled_app_health_summary_get_failure_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field total_apps_count from aos_bundled_app_health_summary message
 *
 * @return  
        Total number of default bundled apps tracked (number of
        AOS_BUNDLED_APP_HEALTH messages to expect on request)
      
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_aos_bundled_app_health_summary_get_total_apps_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a aos_bundled_app_health_summary message into a struct
 *
 * @param msg The message to decode
 * @param aos_bundled_app_health_summary C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_aos_bundled_app_health_summary_decode(const mavlink_message_t* msg, mavlink_aos_bundled_app_health_summary_t* aos_bundled_app_health_summary)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    aos_bundled_app_health_summary->overall_health = mavlink_msg_aos_bundled_app_health_summary_get_overall_health(msg);
    aos_bundled_app_health_summary->failure_flags = mavlink_msg_aos_bundled_app_health_summary_get_failure_flags(msg);
    aos_bundled_app_health_summary->total_apps_count = mavlink_msg_aos_bundled_app_health_summary_get_total_apps_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN? msg->len : MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN;
        memset(aos_bundled_app_health_summary, 0, MAVLINK_MSG_ID_AOS_BUNDLED_APP_HEALTH_SUMMARY_LEN);
    memcpy(aos_bundled_app_health_summary, _MAV_PAYLOAD(msg), len);
#endif
}
