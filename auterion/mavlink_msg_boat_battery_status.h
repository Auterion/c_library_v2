#pragma once
// MESSAGE BOAT_BATTERY_STATUS PACKING

#define MAVLINK_MSG_ID_BOAT_BATTERY_STATUS 668


typedef struct __mavlink_boat_battery_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float voltage; /*< [V] Battery voltage.*/
 float current; /*< [A] Battery current.*/
 float temperature; /*< [K] Battery temperature.*/
 uint8_t instance; /*<  Battery instance.*/
} mavlink_boat_battery_status_t;

#define MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN 21
#define MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN 21
#define MAVLINK_MSG_ID_668_LEN 21
#define MAVLINK_MSG_ID_668_MIN_LEN 21

#define MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC 144
#define MAVLINK_MSG_ID_668_CRC 144



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BOAT_BATTERY_STATUS { \
    668, \
    "BOAT_BATTERY_STATUS", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_boat_battery_status_t, time_usec) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_boat_battery_status_t, instance) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_boat_battery_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_boat_battery_status_t, current) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_boat_battery_status_t, temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BOAT_BATTERY_STATUS { \
    "BOAT_BATTERY_STATUS", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_boat_battery_status_t, time_usec) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_boat_battery_status_t, instance) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_boat_battery_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_boat_battery_status_t, current) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_boat_battery_status_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a boat_battery_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Battery instance.
 * @param voltage [V] Battery voltage.
 * @param current [A] Battery current.
 * @param temperature [K] Battery temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_battery_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t instance, float voltage, float current, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, current);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint8_t(buf, 20, instance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
#else
    mavlink_boat_battery_status_t packet;
    packet.time_usec = time_usec;
    packet.voltage = voltage;
    packet.current = current;
    packet.temperature = temperature;
    packet.instance = instance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_BATTERY_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
}

/**
 * @brief Pack a boat_battery_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Battery instance.
 * @param voltage [V] Battery voltage.
 * @param current [A] Battery current.
 * @param temperature [K] Battery temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boat_battery_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t instance, float voltage, float current, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, current);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint8_t(buf, 20, instance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
#else
    mavlink_boat_battery_status_t packet;
    packet.time_usec = time_usec;
    packet.voltage = voltage;
    packet.current = current;
    packet.temperature = temperature;
    packet.instance = instance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_BATTERY_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
#endif
}

/**
 * @brief Pack a boat_battery_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Battery instance.
 * @param voltage [V] Battery voltage.
 * @param current [A] Battery current.
 * @param temperature [K] Battery temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_battery_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t instance,float voltage,float current,float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, current);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint8_t(buf, 20, instance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
#else
    mavlink_boat_battery_status_t packet;
    packet.time_usec = time_usec;
    packet.voltage = voltage;
    packet.current = current;
    packet.temperature = temperature;
    packet.instance = instance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOAT_BATTERY_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
}

/**
 * @brief Encode a boat_battery_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boat_battery_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_battery_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boat_battery_status_t* boat_battery_status)
{
    return mavlink_msg_boat_battery_status_pack(system_id, component_id, msg, boat_battery_status->time_usec, boat_battery_status->instance, boat_battery_status->voltage, boat_battery_status->current, boat_battery_status->temperature);
}

/**
 * @brief Encode a boat_battery_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boat_battery_status C-struct to read the message contents from
 */
MAVLINK_WIP
static inline uint16_t mavlink_msg_boat_battery_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_boat_battery_status_t* boat_battery_status)
{
    return mavlink_msg_boat_battery_status_pack_chan(system_id, component_id, chan, msg, boat_battery_status->time_usec, boat_battery_status->instance, boat_battery_status->voltage, boat_battery_status->current, boat_battery_status->temperature);
}

/**
 * @brief Encode a boat_battery_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param boat_battery_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boat_battery_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_boat_battery_status_t* boat_battery_status)
{
    return mavlink_msg_boat_battery_status_pack_status(system_id, component_id, _status, msg,  boat_battery_status->time_usec, boat_battery_status->instance, boat_battery_status->voltage, boat_battery_status->current, boat_battery_status->temperature);
}

/**
 * @brief Send a boat_battery_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param instance  Battery instance.
 * @param voltage [V] Battery voltage.
 * @param current [A] Battery current.
 * @param temperature [K] Battery temperature.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_WIP
static inline void mavlink_msg_boat_battery_status_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t instance, float voltage, float current, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, current);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint8_t(buf, 20, instance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS, buf, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
#else
    mavlink_boat_battery_status_t packet;
    packet.time_usec = time_usec;
    packet.voltage = voltage;
    packet.current = current;
    packet.temperature = temperature;
    packet.instance = instance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
#endif
}

/**
 * @brief Send a boat_battery_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_battery_status_send_struct(mavlink_channel_t chan, const mavlink_boat_battery_status_t* boat_battery_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_boat_battery_status_send(chan, boat_battery_status->time_usec, boat_battery_status->instance, boat_battery_status->voltage, boat_battery_status->current, boat_battery_status->temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS, (const char *)boat_battery_status, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_battery_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t instance, float voltage, float current, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, current);
    _mav_put_float(buf, 16, temperature);
    _mav_put_uint8_t(buf, 20, instance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS, buf, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
#else
    mavlink_boat_battery_status_t *packet = (mavlink_boat_battery_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->voltage = voltage;
    packet->current = current;
    packet->temperature = temperature;
    packet->instance = instance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS, (const char *)packet, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE BOAT_BATTERY_STATUS UNPACKING


/**
 * @brief Get field time_usec from boat_battery_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
MAVLINK_WIP
static inline uint64_t mavlink_msg_boat_battery_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field instance from boat_battery_status message
 *
 * @return  Battery instance.
 */
MAVLINK_WIP
static inline uint8_t mavlink_msg_boat_battery_status_get_instance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field voltage from boat_battery_status message
 *
 * @return [V] Battery voltage.
 */
MAVLINK_WIP
static inline float mavlink_msg_boat_battery_status_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field current from boat_battery_status message
 *
 * @return [A] Battery current.
 */
MAVLINK_WIP
static inline float mavlink_msg_boat_battery_status_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field temperature from boat_battery_status message
 *
 * @return [K] Battery temperature.
 */
MAVLINK_WIP
static inline float mavlink_msg_boat_battery_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a boat_battery_status message into a struct
 *
 * @param msg The message to decode
 * @param boat_battery_status C-struct to decode the message contents into
 */
MAVLINK_WIP
static inline void mavlink_msg_boat_battery_status_decode(const mavlink_message_t* msg, mavlink_boat_battery_status_t* boat_battery_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    boat_battery_status->time_usec = mavlink_msg_boat_battery_status_get_time_usec(msg);
    boat_battery_status->voltage = mavlink_msg_boat_battery_status_get_voltage(msg);
    boat_battery_status->current = mavlink_msg_boat_battery_status_get_current(msg);
    boat_battery_status->temperature = mavlink_msg_boat_battery_status_get_temperature(msg);
    boat_battery_status->instance = mavlink_msg_boat_battery_status_get_instance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN? msg->len : MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN;
        memset(boat_battery_status, 0, MAVLINK_MSG_ID_BOAT_BATTERY_STATUS_LEN);
    memcpy(boat_battery_status, _MAV_PAYLOAD(msg), len);
#endif
}
