#pragma once
// MESSAGE SET_ALARM PACKING

#define MAVLINK_MSG_ID_SET_ALARM 7005


typedef struct __mavlink_set_alarm_t {
 uint32_t alarm_id; /*< [id] ID of the alarm to set/clear*/
 uint8_t set; /*< [boolean] True to set, False to clear*/
} mavlink_set_alarm_t;

#define MAVLINK_MSG_ID_SET_ALARM_LEN 5
#define MAVLINK_MSG_ID_SET_ALARM_MIN_LEN 5
#define MAVLINK_MSG_ID_7005_LEN 5
#define MAVLINK_MSG_ID_7005_MIN_LEN 5

#define MAVLINK_MSG_ID_SET_ALARM_CRC 135
#define MAVLINK_MSG_ID_7005_CRC 135



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_ALARM { \
    7005, \
    "SET_ALARM", \
    2, \
    {  { "alarm_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_alarm_t, alarm_id) }, \
         { "set", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_alarm_t, set) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_ALARM { \
    "SET_ALARM", \
    2, \
    {  { "alarm_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_alarm_t, alarm_id) }, \
         { "set", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_set_alarm_t, set) }, \
         } \
}
#endif

/**
 * @brief Pack a set_alarm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param alarm_id [id] ID of the alarm to set/clear
 * @param set [boolean] True to set, False to clear
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_alarm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t alarm_id, uint8_t set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ALARM_LEN];
    _mav_put_uint32_t(buf, 0, alarm_id);
    _mav_put_uint8_t(buf, 4, set);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ALARM_LEN);
#else
    mavlink_set_alarm_t packet;
    packet.alarm_id = alarm_id;
    packet.set = set;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ALARM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ALARM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ALARM_MIN_LEN, MAVLINK_MSG_ID_SET_ALARM_LEN, MAVLINK_MSG_ID_SET_ALARM_CRC);
}

/**
 * @brief Pack a set_alarm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param alarm_id [id] ID of the alarm to set/clear
 * @param set [boolean] True to set, False to clear
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_alarm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t alarm_id,uint8_t set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ALARM_LEN];
    _mav_put_uint32_t(buf, 0, alarm_id);
    _mav_put_uint8_t(buf, 4, set);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ALARM_LEN);
#else
    mavlink_set_alarm_t packet;
    packet.alarm_id = alarm_id;
    packet.set = set;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ALARM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ALARM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ALARM_MIN_LEN, MAVLINK_MSG_ID_SET_ALARM_LEN, MAVLINK_MSG_ID_SET_ALARM_CRC);
}

/**
 * @brief Encode a set_alarm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_alarm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_alarm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_alarm_t* set_alarm)
{
    return mavlink_msg_set_alarm_pack(system_id, component_id, msg, set_alarm->alarm_id, set_alarm->set);
}

/**
 * @brief Encode a set_alarm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_alarm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_alarm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_alarm_t* set_alarm)
{
    return mavlink_msg_set_alarm_pack_chan(system_id, component_id, chan, msg, set_alarm->alarm_id, set_alarm->set);
}

/**
 * @brief Send a set_alarm message
 * @param chan MAVLink channel to send the message
 *
 * @param alarm_id [id] ID of the alarm to set/clear
 * @param set [boolean] True to set, False to clear
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_alarm_send(mavlink_channel_t chan, uint32_t alarm_id, uint8_t set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ALARM_LEN];
    _mav_put_uint32_t(buf, 0, alarm_id);
    _mav_put_uint8_t(buf, 4, set);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ALARM, buf, MAVLINK_MSG_ID_SET_ALARM_MIN_LEN, MAVLINK_MSG_ID_SET_ALARM_LEN, MAVLINK_MSG_ID_SET_ALARM_CRC);
#else
    mavlink_set_alarm_t packet;
    packet.alarm_id = alarm_id;
    packet.set = set;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ALARM, (const char *)&packet, MAVLINK_MSG_ID_SET_ALARM_MIN_LEN, MAVLINK_MSG_ID_SET_ALARM_LEN, MAVLINK_MSG_ID_SET_ALARM_CRC);
#endif
}

/**
 * @brief Send a set_alarm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_alarm_send_struct(mavlink_channel_t chan, const mavlink_set_alarm_t* set_alarm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_alarm_send(chan, set_alarm->alarm_id, set_alarm->set);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ALARM, (const char *)set_alarm, MAVLINK_MSG_ID_SET_ALARM_MIN_LEN, MAVLINK_MSG_ID_SET_ALARM_LEN, MAVLINK_MSG_ID_SET_ALARM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_ALARM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_alarm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t alarm_id, uint8_t set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, alarm_id);
    _mav_put_uint8_t(buf, 4, set);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ALARM, buf, MAVLINK_MSG_ID_SET_ALARM_MIN_LEN, MAVLINK_MSG_ID_SET_ALARM_LEN, MAVLINK_MSG_ID_SET_ALARM_CRC);
#else
    mavlink_set_alarm_t *packet = (mavlink_set_alarm_t *)msgbuf;
    packet->alarm_id = alarm_id;
    packet->set = set;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ALARM, (const char *)packet, MAVLINK_MSG_ID_SET_ALARM_MIN_LEN, MAVLINK_MSG_ID_SET_ALARM_LEN, MAVLINK_MSG_ID_SET_ALARM_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_ALARM UNPACKING


/**
 * @brief Get field alarm_id from set_alarm message
 *
 * @return [id] ID of the alarm to set/clear
 */
static inline uint32_t mavlink_msg_set_alarm_get_alarm_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field set from set_alarm message
 *
 * @return [boolean] True to set, False to clear
 */
static inline uint8_t mavlink_msg_set_alarm_get_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a set_alarm message into a struct
 *
 * @param msg The message to decode
 * @param set_alarm C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_alarm_decode(const mavlink_message_t* msg, mavlink_set_alarm_t* set_alarm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_alarm->alarm_id = mavlink_msg_set_alarm_get_alarm_id(msg);
    set_alarm->set = mavlink_msg_set_alarm_get_set(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_ALARM_LEN? msg->len : MAVLINK_MSG_ID_SET_ALARM_LEN;
        memset(set_alarm, 0, MAVLINK_MSG_ID_SET_ALARM_LEN);
    memcpy(set_alarm, _MAV_PAYLOAD(msg), len);
#endif
}
