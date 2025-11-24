#pragma once
// MESSAGE BOAT_FLUID_LEVEL PACKING

#define MAVLINK_MSG_ID_BOAT_FLUID_LEVEL 669


typedef struct __mavlink_boat_fluid_level_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float level; /*< [%] Fluid level in percentage of full tank.*/
 float capacity; /*< [l] Fluid capacity left in tank in liters.*/
 uint8_t instance; /*<  Fluid tank instance.*/
 uint8_t type; /*<  Fluid tank type.*/
} mavlink_boat_fluid_level_t;

#define MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN 18
#define MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN 18
#define MAVLINK_MSG_ID_669_LEN 18
#define MAVLINK_MSG_ID_669_MIN_LEN 18

#define MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC 214
#define MAVLINK_MSG_ID_669_CRC 214



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BOAT_FLUID_LEVEL { \
    669, \
    "BOAT_FLUID_LEVEL", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_boat_fluid_level_t, time_usec) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_boat_fluid_level_t, instance) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_boat_fluid_level_t, type) }, \
         { "level", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_boat_fluid_level_t, level) }, \
         { "capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_boat_fluid_level_t, capacity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BOAT_FLUID_LEVEL { \
    "BOAT_FLUID_LEVEL", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_boat_fluid_level_t, time_usec) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_boat_fluid_level_t, instance) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_boat_fluid_level_t, type) }, \
         { "level", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_boat_fluid_level_t, level) }, \
         { "capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_boat_fluid_level_t, capacity) }, \
         } \
}
#endif

/**
 * @brief Pack a boat_fluid_level message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Fluid tank instance.
 * @param type  Fluid tank type.
 * @param level [%] Fluid level in percentage of full tank.
 * @param capacity [l] Fluid capacity left in tank in liters.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_fluid_level_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t instance, uint8_t type, float level, float capacity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, level);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, instance);
    _mav_put_uint8_t(buf, 17, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
#else
    mavlink_boat_fluid_level_t packet;
    packet.time_usec = time_usec;
    packet.level = level;
    packet.capacity = capacity;
    packet.instance = instance;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_FLUID_LEVEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
}

/**
 * @brief Pack a boat_fluid_level message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Fluid tank instance.
 * @param type  Fluid tank type.
 * @param level [%] Fluid level in percentage of full tank.
 * @param capacity [l] Fluid capacity left in tank in liters.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boat_fluid_level_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t instance, uint8_t type, float level, float capacity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, level);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, instance);
    _mav_put_uint8_t(buf, 17, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
#else
    mavlink_boat_fluid_level_t packet;
    packet.time_usec = time_usec;
    packet.level = level;
    packet.capacity = capacity;
    packet.instance = instance;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_FLUID_LEVEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
#endif
}

/**
 * @brief Pack a boat_fluid_level message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Fluid tank instance.
 * @param type  Fluid tank type.
 * @param level [%] Fluid level in percentage of full tank.
 * @param capacity [l] Fluid capacity left in tank in liters.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_fluid_level_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t instance,uint8_t type,float level,float capacity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, level);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, instance);
    _mav_put_uint8_t(buf, 17, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
#else
    mavlink_boat_fluid_level_t packet;
    packet.time_usec = time_usec;
    packet.level = level;
    packet.capacity = capacity;
    packet.instance = instance;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_FLUID_LEVEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
}

/**
 * @brief Encode a boat_fluid_level struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boat_fluid_level C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_fluid_level_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boat_fluid_level_t* boat_fluid_level)
{
    return mavlink_msg_boat_fluid_level_pack(system_id, component_id, msg, boat_fluid_level->time_usec, boat_fluid_level->instance, boat_fluid_level->type, boat_fluid_level->level, boat_fluid_level->capacity);
}

/**
 * @brief Encode a boat_fluid_level struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boat_fluid_level C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_fluid_level_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_boat_fluid_level_t* boat_fluid_level)
{
    return mavlink_msg_boat_fluid_level_pack_chan(system_id, component_id, chan, msg, boat_fluid_level->time_usec, boat_fluid_level->instance, boat_fluid_level->type, boat_fluid_level->level, boat_fluid_level->capacity);
}

/**
 * @brief Encode a boat_fluid_level struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param boat_fluid_level C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boat_fluid_level_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_boat_fluid_level_t* boat_fluid_level)
{
    return mavlink_msg_boat_fluid_level_pack_status(system_id, component_id, _status, msg,  boat_fluid_level->time_usec, boat_fluid_level->instance, boat_fluid_level->type, boat_fluid_level->level, boat_fluid_level->capacity);
}

/**
 * @brief Send a boat_fluid_level message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Fluid tank instance.
 * @param type  Fluid tank type.
 * @param level [%] Fluid level in percentage of full tank.
 * @param capacity [l] Fluid capacity left in tank in liters.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_boat_fluid_level_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t instance, uint8_t type, float level, float capacity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, level);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, instance);
    _mav_put_uint8_t(buf, 17, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL, buf, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
#else
    mavlink_boat_fluid_level_t packet;
    packet.time_usec = time_usec;
    packet.level = level;
    packet.capacity = capacity;
    packet.instance = instance;
    packet.type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL, (const char *)&packet, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
#endif
}

/**
 * @brief Send a boat_fluid_level message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_fluid_level_send_struct(mavlink_channel_t chan, const mavlink_boat_fluid_level_t* boat_fluid_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_boat_fluid_level_send(chan, boat_fluid_level->time_usec, boat_fluid_level->instance, boat_fluid_level->type, boat_fluid_level->level, boat_fluid_level->capacity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL, (const char *)boat_fluid_level, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_fluid_level_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t instance, uint8_t type, float level, float capacity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, level);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, instance);
    _mav_put_uint8_t(buf, 17, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL, buf, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
#else
    mavlink_boat_fluid_level_t *packet = (mavlink_boat_fluid_level_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->level = level;
    packet->capacity = capacity;
    packet->instance = instance;
    packet->type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL, (const char *)packet, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_MIN_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_CRC);
#endif
}
#endif

#endif

// MESSAGE BOAT_FLUID_LEVEL UNPACKING


/**
 * @brief Get field time_usec from boat_fluid_level message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
MAVLINK_WIP
static inline uint64_t mavlink_msg_boat_fluid_level_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field instance from boat_fluid_level message
 *
 * @return  Fluid tank instance.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_boat_fluid_level_get_instance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field type from boat_fluid_level message
 *
 * @return  Fluid tank type.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_boat_fluid_level_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field level from boat_fluid_level message
 *
 * @return [%] Fluid level in percentage of full tank.
 */
MAVLINK_WIP
static inline float mavlink_msg_boat_fluid_level_get_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field capacity from boat_fluid_level message
 *
 * @return [l] Fluid capacity left in tank in liters.
 */
MAVLINK_WIP
static inline float mavlink_msg_boat_fluid_level_get_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a boat_fluid_level message into a struct
 *
 * @param msg The message to decode
 * @param boat_fluid_level C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_fluid_level_decode(const mavlink_message_t* msg, mavlink_boat_fluid_level_t* boat_fluid_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    boat_fluid_level->time_usec = mavlink_msg_boat_fluid_level_get_time_usec(msg);
    boat_fluid_level->level = mavlink_msg_boat_fluid_level_get_level(msg);
    boat_fluid_level->capacity = mavlink_msg_boat_fluid_level_get_capacity(msg);
    boat_fluid_level->instance = mavlink_msg_boat_fluid_level_get_instance(msg);
    boat_fluid_level->type = mavlink_msg_boat_fluid_level_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN? msg->len : MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN;
        memset(boat_fluid_level, 0, MAVLINK_MSG_ID_BOAT_FLUID_LEVEL_LEN);
    memcpy(boat_fluid_level, _MAV_PAYLOAD(msg), len);
#endif
}
