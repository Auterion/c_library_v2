#pragma once
// MESSAGE CORRIDOR_STATUS PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_CORRIDOR_STATUS 13518


typedef struct __mavlink_corridor_status_t {
 uint32_t corridor_id; /*<  Id of the current on-vehicle corridor graph plan. This is the opaque_id returned in MISSION_ACK when the graph was uploaded, and it changes whenever the stored graph changes. 0 if there is no corridor graph loaded or plan ids are not supported.*/
 uint16_t destination_node; /*<  Node ID of the final node of the route the vehicle is flying along the corridor graph (for example the landing site selected for a return). 0 if the vehicle is not following the corridor graph.*/
 uint16_t previous_node; /*<  Node ID of the node on the route that the vehicle most recently passed. Together with the first entry of route this identifies the edge the vehicle is on. 0 if the vehicle has not yet passed a node on the route, or is not following the corridor graph.*/
 uint16_t state; /*<  State of the vehicle with respect to the corridor graph.*/
 uint16_t route[32]; /*<  Node IDs of the upcoming nodes on the route, in the order they will be flown. The first entry is the node the vehicle is currently flying towards. Unused entries are 0. If more than 32 nodes remain, only the next 32 are listed (the end of the route is given by destination_node).*/
} mavlink_corridor_status_t;

#define MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN 74
#define MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN 74
#define MAVLINK_MSG_ID_13518_LEN 74
#define MAVLINK_MSG_ID_13518_MIN_LEN 74

#define MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC 249
#define MAVLINK_MSG_ID_13518_CRC 249

#define MAVLINK_MSG_CORRIDOR_STATUS_FIELD_ROUTE_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CORRIDOR_STATUS { \
    13518, \
    "CORRIDOR_STATUS", \
    5, \
    {  { "corridor_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_corridor_status_t, corridor_id) }, \
         { "destination_node", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_corridor_status_t, destination_node) }, \
         { "previous_node", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_corridor_status_t, previous_node) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_corridor_status_t, state) }, \
         { "route", NULL, MAVLINK_TYPE_UINT16_T, 32, 10, offsetof(mavlink_corridor_status_t, route) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CORRIDOR_STATUS { \
    "CORRIDOR_STATUS", \
    5, \
    {  { "corridor_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_corridor_status_t, corridor_id) }, \
         { "destination_node", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_corridor_status_t, destination_node) }, \
         { "previous_node", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_corridor_status_t, previous_node) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_corridor_status_t, state) }, \
         { "route", NULL, MAVLINK_TYPE_UINT16_T, 32, 10, offsetof(mavlink_corridor_status_t, route) }, \
         } \
}
#endif

/**
 * @brief Pack a corridor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param corridor_id  Id of the current on-vehicle corridor graph plan. This is the opaque_id returned in MISSION_ACK when the graph was uploaded, and it changes whenever the stored graph changes. 0 if there is no corridor graph loaded or plan ids are not supported.
 * @param destination_node  Node ID of the final node of the route the vehicle is flying along the corridor graph (for example the landing site selected for a return). 0 if the vehicle is not following the corridor graph.
 * @param previous_node  Node ID of the node on the route that the vehicle most recently passed. Together with the first entry of route this identifies the edge the vehicle is on. 0 if the vehicle has not yet passed a node on the route, or is not following the corridor graph.
 * @param state  State of the vehicle with respect to the corridor graph.
 * @param route  Node IDs of the upcoming nodes on the route, in the order they will be flown. The first entry is the node the vehicle is currently flying towards. Unused entries are 0. If more than 32 nodes remain, only the next 32 are listed (the end of the route is given by destination_node).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_pack(uint32_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t corridor_id, uint16_t destination_node, uint16_t previous_node, uint16_t state, const uint16_t *route)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, corridor_id);
    _mav_put_uint16_t(buf, 4, destination_node);
    _mav_put_uint16_t(buf, 6, previous_node);
    _mav_put_uint16_t(buf, 8, state);
    _mav_put_uint16_t_array(buf, 10, route, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN);
#else
    mavlink_corridor_status_t packet;
    packet.corridor_id = corridor_id;
    packet.destination_node = destination_node;
    packet.previous_node = previous_node;
    packet.state = state;
    mav_array_memcpy(packet.route, route, sizeof(uint16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CORRIDOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
}

/**
 * @brief Pack a corridor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param corridor_id  Id of the current on-vehicle corridor graph plan. This is the opaque_id returned in MISSION_ACK when the graph was uploaded, and it changes whenever the stored graph changes. 0 if there is no corridor graph loaded or plan ids are not supported.
 * @param destination_node  Node ID of the final node of the route the vehicle is flying along the corridor graph (for example the landing site selected for a return). 0 if the vehicle is not following the corridor graph.
 * @param previous_node  Node ID of the node on the route that the vehicle most recently passed. Together with the first entry of route this identifies the edge the vehicle is on. 0 if the vehicle has not yet passed a node on the route, or is not following the corridor graph.
 * @param state  State of the vehicle with respect to the corridor graph.
 * @param route  Node IDs of the upcoming nodes on the route, in the order they will be flown. The first entry is the node the vehicle is currently flying towards. Unused entries are 0. If more than 32 nodes remain, only the next 32 are listed (the end of the route is given by destination_node).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_corridor_status_pack_status(uint32_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t corridor_id, uint16_t destination_node, uint16_t previous_node, uint16_t state, const uint16_t *route)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, corridor_id);
    _mav_put_uint16_t(buf, 4, destination_node);
    _mav_put_uint16_t(buf, 6, previous_node);
    _mav_put_uint16_t(buf, 8, state);
    _mav_put_uint16_t_array(buf, 10, route, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN);
#else
    mavlink_corridor_status_t packet;
    packet.corridor_id = corridor_id;
    packet.destination_node = destination_node;
    packet.previous_node = previous_node;
    packet.state = state;
    mav_array_memcpy(packet.route, route, sizeof(uint16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CORRIDOR_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, 0);
#endif
}

/**
 * @brief Pack a corridor_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param corridor_id  Id of the current on-vehicle corridor graph plan. This is the opaque_id returned in MISSION_ACK when the graph was uploaded, and it changes whenever the stored graph changes. 0 if there is no corridor graph loaded or plan ids are not supported.
 * @param destination_node  Node ID of the final node of the route the vehicle is flying along the corridor graph (for example the landing site selected for a return). 0 if the vehicle is not following the corridor graph.
 * @param previous_node  Node ID of the node on the route that the vehicle most recently passed. Together with the first entry of route this identifies the edge the vehicle is on. 0 if the vehicle has not yet passed a node on the route, or is not following the corridor graph.
 * @param state  State of the vehicle with respect to the corridor graph.
 * @param route  Node IDs of the upcoming nodes on the route, in the order they will be flown. The first entry is the node the vehicle is currently flying towards. Unused entries are 0. If more than 32 nodes remain, only the next 32 are listed (the end of the route is given by destination_node).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_pack_chan(uint32_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t corridor_id,uint16_t destination_node,uint16_t previous_node,uint16_t state,const uint16_t *route)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, corridor_id);
    _mav_put_uint16_t(buf, 4, destination_node);
    _mav_put_uint16_t(buf, 6, previous_node);
    _mav_put_uint16_t(buf, 8, state);
    _mav_put_uint16_t_array(buf, 10, route, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN);
#else
    mavlink_corridor_status_t packet;
    packet.corridor_id = corridor_id;
    packet.destination_node = destination_node;
    packet.previous_node = previous_node;
    packet.state = state;
    mav_array_memcpy(packet.route, route, sizeof(uint16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CORRIDOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
}

/**
 * @brief Encode a corridor_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param corridor_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_encode(uint32_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_corridor_status_t* corridor_status)
{
    return mavlink_msg_corridor_status_pack(system_id, component_id, msg, corridor_status->corridor_id, corridor_status->destination_node, corridor_status->previous_node, corridor_status->state, corridor_status->route);
}

/**
 * @brief Encode a corridor_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param corridor_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_encode_chan(uint32_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_corridor_status_t* corridor_status)
{
    return mavlink_msg_corridor_status_pack_chan(system_id, component_id, chan, msg, corridor_status->corridor_id, corridor_status->destination_node, corridor_status->previous_node, corridor_status->state, corridor_status->route);
}

/**
 * @brief Encode a corridor_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param corridor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_corridor_status_encode_status(uint32_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_corridor_status_t* corridor_status)
{
    return mavlink_msg_corridor_status_pack_status(system_id, component_id, _status, msg,  corridor_status->corridor_id, corridor_status->destination_node, corridor_status->previous_node, corridor_status->state, corridor_status->route);
}

/**
 * @brief Send a corridor_status message
 * @param chan MAVLink channel to send the message
 *
 * @param corridor_id  Id of the current on-vehicle corridor graph plan. This is the opaque_id returned in MISSION_ACK when the graph was uploaded, and it changes whenever the stored graph changes. 0 if there is no corridor graph loaded or plan ids are not supported.
 * @param destination_node  Node ID of the final node of the route the vehicle is flying along the corridor graph (for example the landing site selected for a return). 0 if the vehicle is not following the corridor graph.
 * @param previous_node  Node ID of the node on the route that the vehicle most recently passed. Together with the first entry of route this identifies the edge the vehicle is on. 0 if the vehicle has not yet passed a node on the route, or is not following the corridor graph.
 * @param state  State of the vehicle with respect to the corridor graph.
 * @param route  Node IDs of the upcoming nodes on the route, in the order they will be flown. The first entry is the node the vehicle is currently flying towards. Unused entries are 0. If more than 32 nodes remain, only the next 32 are listed (the end of the route is given by destination_node).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_corridor_status_send(mavlink_channel_t chan, uint32_t corridor_id, uint16_t destination_node, uint16_t previous_node, uint16_t state, const uint16_t *route)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, corridor_id);
    _mav_put_uint16_t(buf, 4, destination_node);
    _mav_put_uint16_t(buf, 6, previous_node);
    _mav_put_uint16_t(buf, 8, state);
    _mav_put_uint16_t_array(buf, 10, route, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORRIDOR_STATUS, buf, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
#else
    mavlink_corridor_status_t packet;
    packet.corridor_id = corridor_id;
    packet.destination_node = destination_node;
    packet.previous_node = previous_node;
    packet.state = state;
    mav_array_memcpy(packet.route, route, sizeof(uint16_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORRIDOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a corridor_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_corridor_status_send_struct(mavlink_channel_t chan, const mavlink_corridor_status_t* corridor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_corridor_status_send(chan, corridor_status->corridor_id, corridor_status->destination_node, corridor_status->previous_node, corridor_status->state, corridor_status->route);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORRIDOR_STATUS, (const char *)corridor_status, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_corridor_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t corridor_id, uint16_t destination_node, uint16_t previous_node, uint16_t state, const uint16_t *route)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, corridor_id);
    _mav_put_uint16_t(buf, 4, destination_node);
    _mav_put_uint16_t(buf, 6, previous_node);
    _mav_put_uint16_t(buf, 8, state);
    _mav_put_uint16_t_array(buf, 10, route, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORRIDOR_STATUS, buf, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
#else
    mavlink_corridor_status_t *packet = (mavlink_corridor_status_t *)msgbuf;
    packet->corridor_id = corridor_id;
    packet->destination_node = destination_node;
    packet->previous_node = previous_node;
    packet->state = state;
    mav_array_memcpy(packet->route, route, sizeof(uint16_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORRIDOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_CORRIDOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN, MAVLINK_MSG_ID_CORRIDOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CORRIDOR_STATUS UNPACKING


/**
 * @brief Get field corridor_id from corridor_status message
 *
 * @return  Id of the current on-vehicle corridor graph plan. This is the opaque_id returned in MISSION_ACK when the graph was uploaded, and it changes whenever the stored graph changes. 0 if there is no corridor graph loaded or plan ids are not supported.
 */
MAVLINK_WIP
static inline uint32_t mavlink_msg_corridor_status_get_corridor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field destination_node from corridor_status message
 *
 * @return  Node ID of the final node of the route the vehicle is flying along the corridor graph (for example the landing site selected for a return). 0 if the vehicle is not following the corridor graph.
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_get_destination_node(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field previous_node from corridor_status message
 *
 * @return  Node ID of the node on the route that the vehicle most recently passed. Together with the first entry of route this identifies the edge the vehicle is on. 0 if the vehicle has not yet passed a node on the route, or is not following the corridor graph.
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_get_previous_node(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field state from corridor_status message
 *
 * @return  State of the vehicle with respect to the corridor graph.
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field route from corridor_status message
 *
 * @return  Node IDs of the upcoming nodes on the route, in the order they will be flown. The first entry is the node the vehicle is currently flying towards. Unused entries are 0. If more than 32 nodes remain, only the next 32 are listed (the end of the route is given by destination_node).
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_corridor_status_get_route(const mavlink_message_t* msg, uint16_t *route)
{
    return _MAV_RETURN_uint16_t_array(msg, route, 32,  10);
}

/**
 * @brief Decode a corridor_status message into a struct
 *
 * @param msg The message to decode
 * @param corridor_status C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_corridor_status_decode(const mavlink_message_t* msg, mavlink_corridor_status_t* corridor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    corridor_status->corridor_id = mavlink_msg_corridor_status_get_corridor_id(msg);
    corridor_status->destination_node = mavlink_msg_corridor_status_get_destination_node(msg);
    corridor_status->previous_node = mavlink_msg_corridor_status_get_previous_node(msg);
    corridor_status->state = mavlink_msg_corridor_status_get_state(msg);
    mavlink_msg_corridor_status_get_route(msg, corridor_status->route);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN;
        memset(corridor_status, 0, MAVLINK_MSG_ID_CORRIDOR_STATUS_LEN);
    memcpy(corridor_status, _MAV_PAYLOAD(msg), len);
#endif

}
