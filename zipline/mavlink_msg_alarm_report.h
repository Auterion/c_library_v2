#pragma once
// MESSAGE ALARM_REPORT PACKING

#define MAVLINK_MSG_ID_ALARM_REPORT 7004


typedef struct __mavlink_alarm_report_t {
 uint64_t alarm_mask[10]; /*< [mask] Active alarm bitmask*/
 uint8_t blocking_preflight; /*< [boolean] An exception is currently blocking preflight*/
 uint8_t should_abort; /*< [boolean] An exception is indicating that we should probably abort any missions*/
} mavlink_alarm_report_t;

#define MAVLINK_MSG_ID_ALARM_REPORT_LEN 82
#define MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN 82
#define MAVLINK_MSG_ID_7004_LEN 82
#define MAVLINK_MSG_ID_7004_MIN_LEN 82

#define MAVLINK_MSG_ID_ALARM_REPORT_CRC 146
#define MAVLINK_MSG_ID_7004_CRC 146

#define MAVLINK_MSG_ALARM_REPORT_FIELD_ALARM_MASK_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ALARM_REPORT { \
    7004, \
    "ALARM_REPORT", \
    3, \
    {  { "alarm_mask", NULL, MAVLINK_TYPE_UINT64_T, 10, 0, offsetof(mavlink_alarm_report_t, alarm_mask) }, \
         { "blocking_preflight", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_alarm_report_t, blocking_preflight) }, \
         { "should_abort", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_alarm_report_t, should_abort) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ALARM_REPORT { \
    "ALARM_REPORT", \
    3, \
    {  { "alarm_mask", NULL, MAVLINK_TYPE_UINT64_T, 10, 0, offsetof(mavlink_alarm_report_t, alarm_mask) }, \
         { "blocking_preflight", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_alarm_report_t, blocking_preflight) }, \
         { "should_abort", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_alarm_report_t, should_abort) }, \
         } \
}
#endif

/**
 * @brief Pack a alarm_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param alarm_mask [mask] Active alarm bitmask
 * @param blocking_preflight [boolean] An exception is currently blocking preflight
 * @param should_abort [boolean] An exception is indicating that we should probably abort any missions
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_alarm_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint64_t *alarm_mask, uint8_t blocking_preflight, uint8_t should_abort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ALARM_REPORT_LEN];
    _mav_put_uint8_t(buf, 80, blocking_preflight);
    _mav_put_uint8_t(buf, 81, should_abort);
    _mav_put_uint64_t_array(buf, 0, alarm_mask, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ALARM_REPORT_LEN);
#else
    mavlink_alarm_report_t packet;
    packet.blocking_preflight = blocking_preflight;
    packet.should_abort = should_abort;
    mav_array_memcpy(packet.alarm_mask, alarm_mask, sizeof(uint64_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ALARM_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ALARM_REPORT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN, MAVLINK_MSG_ID_ALARM_REPORT_LEN, MAVLINK_MSG_ID_ALARM_REPORT_CRC);
}

/**
 * @brief Pack a alarm_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param alarm_mask [mask] Active alarm bitmask
 * @param blocking_preflight [boolean] An exception is currently blocking preflight
 * @param should_abort [boolean] An exception is indicating that we should probably abort any missions
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_alarm_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint64_t *alarm_mask,uint8_t blocking_preflight,uint8_t should_abort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ALARM_REPORT_LEN];
    _mav_put_uint8_t(buf, 80, blocking_preflight);
    _mav_put_uint8_t(buf, 81, should_abort);
    _mav_put_uint64_t_array(buf, 0, alarm_mask, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ALARM_REPORT_LEN);
#else
    mavlink_alarm_report_t packet;
    packet.blocking_preflight = blocking_preflight;
    packet.should_abort = should_abort;
    mav_array_memcpy(packet.alarm_mask, alarm_mask, sizeof(uint64_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ALARM_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ALARM_REPORT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN, MAVLINK_MSG_ID_ALARM_REPORT_LEN, MAVLINK_MSG_ID_ALARM_REPORT_CRC);
}

/**
 * @brief Encode a alarm_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param alarm_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_alarm_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_alarm_report_t* alarm_report)
{
    return mavlink_msg_alarm_report_pack(system_id, component_id, msg, alarm_report->alarm_mask, alarm_report->blocking_preflight, alarm_report->should_abort);
}

/**
 * @brief Encode a alarm_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param alarm_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_alarm_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_alarm_report_t* alarm_report)
{
    return mavlink_msg_alarm_report_pack_chan(system_id, component_id, chan, msg, alarm_report->alarm_mask, alarm_report->blocking_preflight, alarm_report->should_abort);
}

/**
 * @brief Send a alarm_report message
 * @param chan MAVLink channel to send the message
 *
 * @param alarm_mask [mask] Active alarm bitmask
 * @param blocking_preflight [boolean] An exception is currently blocking preflight
 * @param should_abort [boolean] An exception is indicating that we should probably abort any missions
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_alarm_report_send(mavlink_channel_t chan, const uint64_t *alarm_mask, uint8_t blocking_preflight, uint8_t should_abort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ALARM_REPORT_LEN];
    _mav_put_uint8_t(buf, 80, blocking_preflight);
    _mav_put_uint8_t(buf, 81, should_abort);
    _mav_put_uint64_t_array(buf, 0, alarm_mask, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALARM_REPORT, buf, MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN, MAVLINK_MSG_ID_ALARM_REPORT_LEN, MAVLINK_MSG_ID_ALARM_REPORT_CRC);
#else
    mavlink_alarm_report_t packet;
    packet.blocking_preflight = blocking_preflight;
    packet.should_abort = should_abort;
    mav_array_memcpy(packet.alarm_mask, alarm_mask, sizeof(uint64_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALARM_REPORT, (const char *)&packet, MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN, MAVLINK_MSG_ID_ALARM_REPORT_LEN, MAVLINK_MSG_ID_ALARM_REPORT_CRC);
#endif
}

/**
 * @brief Send a alarm_report message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_alarm_report_send_struct(mavlink_channel_t chan, const mavlink_alarm_report_t* alarm_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_alarm_report_send(chan, alarm_report->alarm_mask, alarm_report->blocking_preflight, alarm_report->should_abort);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALARM_REPORT, (const char *)alarm_report, MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN, MAVLINK_MSG_ID_ALARM_REPORT_LEN, MAVLINK_MSG_ID_ALARM_REPORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ALARM_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_alarm_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint64_t *alarm_mask, uint8_t blocking_preflight, uint8_t should_abort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 80, blocking_preflight);
    _mav_put_uint8_t(buf, 81, should_abort);
    _mav_put_uint64_t_array(buf, 0, alarm_mask, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALARM_REPORT, buf, MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN, MAVLINK_MSG_ID_ALARM_REPORT_LEN, MAVLINK_MSG_ID_ALARM_REPORT_CRC);
#else
    mavlink_alarm_report_t *packet = (mavlink_alarm_report_t *)msgbuf;
    packet->blocking_preflight = blocking_preflight;
    packet->should_abort = should_abort;
    mav_array_memcpy(packet->alarm_mask, alarm_mask, sizeof(uint64_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALARM_REPORT, (const char *)packet, MAVLINK_MSG_ID_ALARM_REPORT_MIN_LEN, MAVLINK_MSG_ID_ALARM_REPORT_LEN, MAVLINK_MSG_ID_ALARM_REPORT_CRC);
#endif
}
#endif

#endif

// MESSAGE ALARM_REPORT UNPACKING


/**
 * @brief Get field alarm_mask from alarm_report message
 *
 * @return [mask] Active alarm bitmask
 */
static inline uint16_t mavlink_msg_alarm_report_get_alarm_mask(const mavlink_message_t* msg, uint64_t *alarm_mask)
{
    return _MAV_RETURN_uint64_t_array(msg, alarm_mask, 10,  0);
}

/**
 * @brief Get field blocking_preflight from alarm_report message
 *
 * @return [boolean] An exception is currently blocking preflight
 */
static inline uint8_t mavlink_msg_alarm_report_get_blocking_preflight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field should_abort from alarm_report message
 *
 * @return [boolean] An exception is indicating that we should probably abort any missions
 */
static inline uint8_t mavlink_msg_alarm_report_get_should_abort(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  81);
}

/**
 * @brief Decode a alarm_report message into a struct
 *
 * @param msg The message to decode
 * @param alarm_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_alarm_report_decode(const mavlink_message_t* msg, mavlink_alarm_report_t* alarm_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_alarm_report_get_alarm_mask(msg, alarm_report->alarm_mask);
    alarm_report->blocking_preflight = mavlink_msg_alarm_report_get_blocking_preflight(msg);
    alarm_report->should_abort = mavlink_msg_alarm_report_get_should_abort(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ALARM_REPORT_LEN? msg->len : MAVLINK_MSG_ID_ALARM_REPORT_LEN;
        memset(alarm_report, 0, MAVLINK_MSG_ID_ALARM_REPORT_LEN);
    memcpy(alarm_report, _MAV_PAYLOAD(msg), len);
#endif
}
