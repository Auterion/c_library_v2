#pragma once
// MESSAGE VIDEO_STREAM_RECEIVER_REPORT PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT 13810


typedef struct __mavlink_video_stream_receiver_report_t {
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID of the camera.*/
 uint8_t stream_id; /*<  Video stream ID (as in VIDEO_STREAM_INFORMATION).*/
 uint8_t health; /*<  Stream health.*/
 uint8_t request; /*<  Request carried by this report; NONE otherwise.*/
 uint8_t request_count; /*<  Identifies the request: the sender acts once per value. 0 when request is NONE; wraps from 255 to 1.*/
} mavlink_video_stream_receiver_report_t;

#define MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN 6
#define MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN 4
#define MAVLINK_MSG_ID_13810_LEN 6
#define MAVLINK_MSG_ID_13810_MIN_LEN 4

#define MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC 45
#define MAVLINK_MSG_ID_13810_CRC 45



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIDEO_STREAM_RECEIVER_REPORT { \
    13810, \
    "VIDEO_STREAM_RECEIVER_REPORT", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_video_stream_receiver_report_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_video_stream_receiver_report_t, target_component) }, \
         { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_video_stream_receiver_report_t, stream_id) }, \
         { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_video_stream_receiver_report_t, health) }, \
         { "request", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_video_stream_receiver_report_t, request) }, \
         { "request_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_video_stream_receiver_report_t, request_count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIDEO_STREAM_RECEIVER_REPORT { \
    "VIDEO_STREAM_RECEIVER_REPORT", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_video_stream_receiver_report_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_video_stream_receiver_report_t, target_component) }, \
         { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_video_stream_receiver_report_t, stream_id) }, \
         { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_video_stream_receiver_report_t, health) }, \
         { "request", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_video_stream_receiver_report_t, request) }, \
         { "request_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_video_stream_receiver_report_t, request_count) }, \
         } \
}
#endif

/**
 * @brief Pack a video_stream_receiver_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID of the camera.
 * @param stream_id  Video stream ID (as in VIDEO_STREAM_INFORMATION).
 * @param health  Stream health.
 * @param request  Request carried by this report; NONE otherwise.
 * @param request_count  Identifies the request: the sender acts once per value. 0 when request is NONE; wraps from 255 to 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_video_stream_receiver_report_pack(uint32_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t target_system, uint8_t target_component, uint8_t stream_id, uint8_t health, uint8_t request, uint8_t request_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN];
    _mav_put_uint8_t(buf, 0, mavlink_msg_target_field(target_system));
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, stream_id);
    _mav_put_uint8_t(buf, 3, health);
    _mav_put_uint8_t(buf, 4, request);
    _mav_put_uint8_t(buf, 5, request_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN);
#else
    mavlink_video_stream_receiver_report_t packet;
    packet.target_system = mavlink_msg_target_field(target_system);
    packet.target_component = target_component;
    packet.stream_id = stream_id;
    packet.health = health;
    packet.request = request;
    packet.request_count = request_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT;
    return mavlink_finalize_message_target(msg, system_id, component_id, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC, target_system);
}

/**
 * @brief Pack a video_stream_receiver_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID of the camera.
 * @param stream_id  Video stream ID (as in VIDEO_STREAM_INFORMATION).
 * @param health  Stream health.
 * @param request  Request carried by this report; NONE otherwise.
 * @param request_count  Identifies the request: the sender acts once per value. 0 when request is NONE; wraps from 255 to 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_video_stream_receiver_report_pack_status(uint32_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t target_system, uint8_t target_component, uint8_t stream_id, uint8_t health, uint8_t request, uint8_t request_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN];
    _mav_put_uint8_t(buf, 0, mavlink_msg_target_field(target_system));
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, stream_id);
    _mav_put_uint8_t(buf, 3, health);
    _mav_put_uint8_t(buf, 4, request);
    _mav_put_uint8_t(buf, 5, request_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN);
#else
    mavlink_video_stream_receiver_report_t packet;
    packet.target_system = mavlink_msg_target_field(target_system);
    packet.target_component = target_component;
    packet.stream_id = stream_id;
    packet.health = health;
    packet.request = request;
    packet.request_count = request_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer_target(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC, target_system);
#else
    return mavlink_finalize_message_buffer_target(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, 0, target_system);
#endif
}

/**
 * @brief Pack a video_stream_receiver_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID of the camera.
 * @param stream_id  Video stream ID (as in VIDEO_STREAM_INFORMATION).
 * @param health  Stream health.
 * @param request  Request carried by this report; NONE otherwise.
 * @param request_count  Identifies the request: the sender acts once per value. 0 when request is NONE; wraps from 255 to 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_video_stream_receiver_report_pack_chan(uint32_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t target_system,uint8_t target_component,uint8_t stream_id,uint8_t health,uint8_t request,uint8_t request_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN];
    _mav_put_uint8_t(buf, 0, mavlink_msg_target_field(target_system));
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, stream_id);
    _mav_put_uint8_t(buf, 3, health);
    _mav_put_uint8_t(buf, 4, request);
    _mav_put_uint8_t(buf, 5, request_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN);
#else
    mavlink_video_stream_receiver_report_t packet;
    packet.target_system = mavlink_msg_target_field(target_system);
    packet.target_component = target_component;
    packet.stream_id = stream_id;
    packet.health = health;
    packet.request = request;
    packet.request_count = request_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT;
    return mavlink_finalize_message_chan_target(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC, target_system);
}

/**
 * @brief Encode a video_stream_receiver_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param video_stream_receiver_report C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_video_stream_receiver_report_encode(uint32_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_video_stream_receiver_report_t* video_stream_receiver_report)
{
    return mavlink_msg_video_stream_receiver_report_pack(system_id, component_id, msg, video_stream_receiver_report->target_system, video_stream_receiver_report->target_component, video_stream_receiver_report->stream_id, video_stream_receiver_report->health, video_stream_receiver_report->request, video_stream_receiver_report->request_count);
}

/**
 * @brief Encode a video_stream_receiver_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param video_stream_receiver_report C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_video_stream_receiver_report_encode_chan(uint32_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_video_stream_receiver_report_t* video_stream_receiver_report)
{
    return mavlink_msg_video_stream_receiver_report_pack_chan(system_id, component_id, chan, msg, video_stream_receiver_report->target_system, video_stream_receiver_report->target_component, video_stream_receiver_report->stream_id, video_stream_receiver_report->health, video_stream_receiver_report->request, video_stream_receiver_report->request_count);
}

/**
 * @brief Encode a video_stream_receiver_report struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param video_stream_receiver_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_video_stream_receiver_report_encode_status(uint32_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_video_stream_receiver_report_t* video_stream_receiver_report)
{
    return mavlink_msg_video_stream_receiver_report_pack_status(system_id, component_id, _status, msg,  video_stream_receiver_report->target_system, video_stream_receiver_report->target_component, video_stream_receiver_report->stream_id, video_stream_receiver_report->health, video_stream_receiver_report->request, video_stream_receiver_report->request_count);
}

/**
 * @brief Send a video_stream_receiver_report message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID of the camera.
 * @param stream_id  Video stream ID (as in VIDEO_STREAM_INFORMATION).
 * @param health  Stream health.
 * @param request  Request carried by this report; NONE otherwise.
 * @param request_count  Identifies the request: the sender acts once per value. 0 when request is NONE; wraps from 255 to 1.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_video_stream_receiver_report_send(mavlink_channel_t chan, uint32_t target_system, uint8_t target_component, uint8_t stream_id, uint8_t health, uint8_t request, uint8_t request_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN];
    _mav_put_uint8_t(buf, 0, mavlink_msg_target_field(target_system));
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, stream_id);
    _mav_put_uint8_t(buf, 3, health);
    _mav_put_uint8_t(buf, 4, request);
    _mav_put_uint8_t(buf, 5, request_count);

    _mav_finalize_message_chan_send_target(chan, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT, buf, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC, target_system);
#else
    mavlink_video_stream_receiver_report_t packet;
    packet.target_system = mavlink_msg_target_field(target_system);
    packet.target_component = target_component;
    packet.stream_id = stream_id;
    packet.health = health;
    packet.request = request;
    packet.request_count = request_count;

    _mav_finalize_message_chan_send_target(chan, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT, (const char *)&packet, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC, target_system);
#endif
}

/**
 * @brief Send a video_stream_receiver_report message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_video_stream_receiver_report_send_struct(mavlink_channel_t chan, const mavlink_video_stream_receiver_report_t* video_stream_receiver_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_video_stream_receiver_report_send(chan, video_stream_receiver_report->target_system, video_stream_receiver_report->target_component, video_stream_receiver_report->stream_id, video_stream_receiver_report->health, video_stream_receiver_report->request, video_stream_receiver_report->request_count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT, (const char *)video_stream_receiver_report, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_video_stream_receiver_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t target_system, uint8_t target_component, uint8_t stream_id, uint8_t health, uint8_t request, uint8_t request_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, mavlink_msg_target_field(target_system));
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, stream_id);
    _mav_put_uint8_t(buf, 3, health);
    _mav_put_uint8_t(buf, 4, request);
    _mav_put_uint8_t(buf, 5, request_count);

    _mav_finalize_message_chan_send_target(chan, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT, buf, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC, target_system);
#else
    mavlink_video_stream_receiver_report_t *packet = (mavlink_video_stream_receiver_report_t *)msgbuf;
    packet->target_system = mavlink_msg_target_field(target_system);
    packet->target_component = target_component;
    packet->stream_id = stream_id;
    packet->health = health;
    packet->request = request;
    packet->request_count = request_count;

    _mav_finalize_message_chan_send_target(chan, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT, (const char *)packet, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_MIN_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_CRC, target_system);
#endif
}
#endif

#endif

// MESSAGE VIDEO_STREAM_RECEIVER_REPORT UNPACKING


/**
 * @brief Get field target_component from video_stream_receiver_report message
 *
 * @return  Component ID of the camera.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_video_stream_receiver_report_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field stream_id from video_stream_receiver_report message
 *
 * @return  Video stream ID (as in VIDEO_STREAM_INFORMATION).
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_video_stream_receiver_report_get_stream_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field health from video_stream_receiver_report message
 *
 * @return  Stream health.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_video_stream_receiver_report_get_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field request from video_stream_receiver_report message
 *
 * @return  Request carried by this report; NONE otherwise.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_video_stream_receiver_report_get_request(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field request_count from video_stream_receiver_report message
 *
 * @return  Identifies the request: the sender acts once per value. 0 when request is NONE; wraps from 255 to 1.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_video_stream_receiver_report_get_request_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a video_stream_receiver_report message into a struct
 *
 * @param msg The message to decode
 * @param video_stream_receiver_report C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_video_stream_receiver_report_decode(const mavlink_message_t* msg, mavlink_video_stream_receiver_report_t* video_stream_receiver_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    video_stream_receiver_report->target_system = _MAV_RETURN_uint8_t(msg, 0);
    video_stream_receiver_report->target_component = mavlink_msg_video_stream_receiver_report_get_target_component(msg);
    video_stream_receiver_report->stream_id = mavlink_msg_video_stream_receiver_report_get_stream_id(msg);
    video_stream_receiver_report->health = mavlink_msg_video_stream_receiver_report_get_health(msg);
    video_stream_receiver_report->request = mavlink_msg_video_stream_receiver_report_get_request(msg);
    video_stream_receiver_report->request_count = mavlink_msg_video_stream_receiver_report_get_request_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN? msg->len : MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN;
        memset(video_stream_receiver_report, 0, MAVLINK_MSG_ID_VIDEO_STREAM_RECEIVER_REPORT_LEN);
    memcpy(video_stream_receiver_report, _MAV_PAYLOAD(msg), len);
#endif
    if (msg->incompat_flags & MAVLINK_IFLAG_TARGET32) {
        // This uint8_t field cannot retain a wide target. The sentinel is NOT reserved.
        // For routing use mavlink_msg_get_target_sysid(msg, mavlink_get_msg_entry(msg->msgid)).
        // To relay after edits, pass that full target to _pack*(); _encode*() loses it.
        video_stream_receiver_report->target_system = mavlink_msg_target_field(msg->target_sysid);
    }

}
