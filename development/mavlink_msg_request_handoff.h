#pragma once
// MESSAGE REQUEST_HANDOFF PACKING

#define MAVLINK_MSG_ID_REQUEST_HANDOFF 445


typedef struct __mavlink_request_handoff_t {
 uint8_t control_target; /*<  Control target to handoff control ownership.*/
 char requester_id[10]; /*<  Identification of the control entity requesting ownership.*/
 char reason[50]; /*<  Reason from the control entity requesting ownership.*/
} mavlink_request_handoff_t;

#define MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN 61
#define MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN 61
#define MAVLINK_MSG_ID_445_LEN 61
#define MAVLINK_MSG_ID_445_MIN_LEN 61

#define MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC 116
#define MAVLINK_MSG_ID_445_CRC 116

#define MAVLINK_MSG_REQUEST_HANDOFF_FIELD_REQUESTER_ID_LEN 10
#define MAVLINK_MSG_REQUEST_HANDOFF_FIELD_REASON_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REQUEST_HANDOFF { \
    445, \
    "REQUEST_HANDOFF", \
    3, \
    {  { "control_target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_request_handoff_t, control_target) }, \
         { "requester_id", NULL, MAVLINK_TYPE_CHAR, 10, 1, offsetof(mavlink_request_handoff_t, requester_id) }, \
         { "reason", NULL, MAVLINK_TYPE_CHAR, 50, 11, offsetof(mavlink_request_handoff_t, reason) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REQUEST_HANDOFF { \
    "REQUEST_HANDOFF", \
    3, \
    {  { "control_target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_request_handoff_t, control_target) }, \
         { "requester_id", NULL, MAVLINK_TYPE_CHAR, 10, 1, offsetof(mavlink_request_handoff_t, requester_id) }, \
         { "reason", NULL, MAVLINK_TYPE_CHAR, 50, 11, offsetof(mavlink_request_handoff_t, reason) }, \
         } \
}
#endif

/**
 * @brief Pack a request_handoff message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param control_target  Control target to handoff control ownership.
 * @param requester_id  Identification of the control entity requesting ownership.
 * @param reason  Reason from the control entity requesting ownership.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_handoff_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t control_target, const char *requester_id, const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN];
    _mav_put_uint8_t(buf, 0, control_target);
    _mav_put_char_array(buf, 1, requester_id, 10);
    _mav_put_char_array(buf, 11, reason, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN);
#else
    mavlink_request_handoff_t packet;
    packet.control_target = control_target;
    mav_array_memcpy(packet.requester_id, requester_id, sizeof(char)*10);
    mav_array_memcpy(packet.reason, reason, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REQUEST_HANDOFF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC);
}

/**
 * @brief Pack a request_handoff message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param control_target  Control target to handoff control ownership.
 * @param requester_id  Identification of the control entity requesting ownership.
 * @param reason  Reason from the control entity requesting ownership.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_handoff_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t control_target,const char *requester_id,const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN];
    _mav_put_uint8_t(buf, 0, control_target);
    _mav_put_char_array(buf, 1, requester_id, 10);
    _mav_put_char_array(buf, 11, reason, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN);
#else
    mavlink_request_handoff_t packet;
    packet.control_target = control_target;
    mav_array_memcpy(packet.requester_id, requester_id, sizeof(char)*10);
    mav_array_memcpy(packet.reason, reason, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REQUEST_HANDOFF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC);
}

/**
 * @brief Encode a request_handoff struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_handoff C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_handoff_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_handoff_t* request_handoff)
{
    return mavlink_msg_request_handoff_pack(system_id, component_id, msg, request_handoff->control_target, request_handoff->requester_id, request_handoff->reason);
}

/**
 * @brief Encode a request_handoff struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_handoff C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_handoff_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_request_handoff_t* request_handoff)
{
    return mavlink_msg_request_handoff_pack_chan(system_id, component_id, chan, msg, request_handoff->control_target, request_handoff->requester_id, request_handoff->reason);
}

/**
 * @brief Send a request_handoff message
 * @param chan MAVLink channel to send the message
 *
 * @param control_target  Control target to handoff control ownership.
 * @param requester_id  Identification of the control entity requesting ownership.
 * @param reason  Reason from the control entity requesting ownership.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_handoff_send(mavlink_channel_t chan, uint8_t control_target, const char *requester_id, const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN];
    _mav_put_uint8_t(buf, 0, control_target);
    _mav_put_char_array(buf, 1, requester_id, 10);
    _mav_put_char_array(buf, 11, reason, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_HANDOFF, buf, MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC);
#else
    mavlink_request_handoff_t packet;
    packet.control_target = control_target;
    mav_array_memcpy(packet.requester_id, requester_id, sizeof(char)*10);
    mav_array_memcpy(packet.reason, reason, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_HANDOFF, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC);
#endif
}

/**
 * @brief Send a request_handoff message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_request_handoff_send_struct(mavlink_channel_t chan, const mavlink_request_handoff_t* request_handoff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_request_handoff_send(chan, request_handoff->control_target, request_handoff->requester_id, request_handoff->reason);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_HANDOFF, (const char *)request_handoff, MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC);
#endif
}

#if MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_request_handoff_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t control_target, const char *requester_id, const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, control_target);
    _mav_put_char_array(buf, 1, requester_id, 10);
    _mav_put_char_array(buf, 11, reason, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_HANDOFF, buf, MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC);
#else
    mavlink_request_handoff_t *packet = (mavlink_request_handoff_t *)msgbuf;
    packet->control_target = control_target;
    mav_array_memcpy(packet->requester_id, requester_id, sizeof(char)*10);
    mav_array_memcpy(packet->reason, reason, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_HANDOFF, (const char *)packet, MAVLINK_MSG_ID_REQUEST_HANDOFF_MIN_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN, MAVLINK_MSG_ID_REQUEST_HANDOFF_CRC);
#endif
}
#endif

#endif

// MESSAGE REQUEST_HANDOFF UNPACKING


/**
 * @brief Get field control_target from request_handoff message
 *
 * @return  Control target to handoff control ownership.
 */
static inline uint8_t mavlink_msg_request_handoff_get_control_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field requester_id from request_handoff message
 *
 * @return  Identification of the control entity requesting ownership.
 */
static inline uint16_t mavlink_msg_request_handoff_get_requester_id(const mavlink_message_t* msg, char *requester_id)
{
    return _MAV_RETURN_char_array(msg, requester_id, 10,  1);
}

/**
 * @brief Get field reason from request_handoff message
 *
 * @return  Reason from the control entity requesting ownership.
 */
static inline uint16_t mavlink_msg_request_handoff_get_reason(const mavlink_message_t* msg, char *reason)
{
    return _MAV_RETURN_char_array(msg, reason, 50,  11);
}

/**
 * @brief Decode a request_handoff message into a struct
 *
 * @param msg The message to decode
 * @param request_handoff C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_handoff_decode(const mavlink_message_t* msg, mavlink_request_handoff_t* request_handoff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    request_handoff->control_target = mavlink_msg_request_handoff_get_control_target(msg);
    mavlink_msg_request_handoff_get_requester_id(msg, request_handoff->requester_id);
    mavlink_msg_request_handoff_get_reason(msg, request_handoff->reason);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN? msg->len : MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN;
        memset(request_handoff, 0, MAVLINK_MSG_ID_REQUEST_HANDOFF_LEN);
    memcpy(request_handoff, _MAV_PAYLOAD(msg), len);
#endif
}
