#pragma once
// MESSAGE TRISONICA_WIND PACKING

#define MAVLINK_MSG_ID_TRISONICA_WIND 7006


typedef struct __mavlink_trisonica_wind_t {
 float wind_n; /*< [m/s] Velocity of air w.r.t ground in north*/
 float wind_e; /*< [m/s] Velocity of air w.r.t ground in east*/
 float wind_d; /*< [m/s] Velocity of air w.r.t ground in down*/
 float temperature; /*< [C] Temperature*/
 float humidity; /*< [percent] Humidity*/
 float pressure; /*< [Pa] Pressure*/
 float wind_speed; /*< [m/s] Absolute lateral wind speed*/
 float wind_heading; /*< [deg] Heading from north about down*/
} mavlink_trisonica_wind_t;

#define MAVLINK_MSG_ID_TRISONICA_WIND_LEN 32
#define MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN 32
#define MAVLINK_MSG_ID_7006_LEN 32
#define MAVLINK_MSG_ID_7006_MIN_LEN 32

#define MAVLINK_MSG_ID_TRISONICA_WIND_CRC 60
#define MAVLINK_MSG_ID_7006_CRC 60



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRISONICA_WIND { \
    7006, \
    "TRISONICA_WIND", \
    8, \
    {  { "wind_n", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_trisonica_wind_t, wind_n) }, \
         { "wind_e", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_trisonica_wind_t, wind_e) }, \
         { "wind_d", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_trisonica_wind_t, wind_d) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_trisonica_wind_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_trisonica_wind_t, humidity) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_trisonica_wind_t, pressure) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_trisonica_wind_t, wind_speed) }, \
         { "wind_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_trisonica_wind_t, wind_heading) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRISONICA_WIND { \
    "TRISONICA_WIND", \
    8, \
    {  { "wind_n", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_trisonica_wind_t, wind_n) }, \
         { "wind_e", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_trisonica_wind_t, wind_e) }, \
         { "wind_d", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_trisonica_wind_t, wind_d) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_trisonica_wind_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_trisonica_wind_t, humidity) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_trisonica_wind_t, pressure) }, \
         { "wind_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_trisonica_wind_t, wind_speed) }, \
         { "wind_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_trisonica_wind_t, wind_heading) }, \
         } \
}
#endif

/**
 * @brief Pack a trisonica_wind message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param wind_n [m/s] Velocity of air w.r.t ground in north
 * @param wind_e [m/s] Velocity of air w.r.t ground in east
 * @param wind_d [m/s] Velocity of air w.r.t ground in down
 * @param temperature [C] Temperature
 * @param humidity [percent] Humidity
 * @param pressure [Pa] Pressure
 * @param wind_speed [m/s] Absolute lateral wind speed
 * @param wind_heading [deg] Heading from north about down
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trisonica_wind_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float wind_n, float wind_e, float wind_d, float temperature, float humidity, float pressure, float wind_speed, float wind_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRISONICA_WIND_LEN];
    _mav_put_float(buf, 0, wind_n);
    _mav_put_float(buf, 4, wind_e);
    _mav_put_float(buf, 8, wind_d);
    _mav_put_float(buf, 12, temperature);
    _mav_put_float(buf, 16, humidity);
    _mav_put_float(buf, 20, pressure);
    _mav_put_float(buf, 24, wind_speed);
    _mav_put_float(buf, 28, wind_heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRISONICA_WIND_LEN);
#else
    mavlink_trisonica_wind_t packet;
    packet.wind_n = wind_n;
    packet.wind_e = wind_e;
    packet.wind_d = wind_d;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.wind_speed = wind_speed;
    packet.wind_heading = wind_heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRISONICA_WIND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRISONICA_WIND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_CRC);
}

/**
 * @brief Pack a trisonica_wind message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wind_n [m/s] Velocity of air w.r.t ground in north
 * @param wind_e [m/s] Velocity of air w.r.t ground in east
 * @param wind_d [m/s] Velocity of air w.r.t ground in down
 * @param temperature [C] Temperature
 * @param humidity [percent] Humidity
 * @param pressure [Pa] Pressure
 * @param wind_speed [m/s] Absolute lateral wind speed
 * @param wind_heading [deg] Heading from north about down
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trisonica_wind_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float wind_n,float wind_e,float wind_d,float temperature,float humidity,float pressure,float wind_speed,float wind_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRISONICA_WIND_LEN];
    _mav_put_float(buf, 0, wind_n);
    _mav_put_float(buf, 4, wind_e);
    _mav_put_float(buf, 8, wind_d);
    _mav_put_float(buf, 12, temperature);
    _mav_put_float(buf, 16, humidity);
    _mav_put_float(buf, 20, pressure);
    _mav_put_float(buf, 24, wind_speed);
    _mav_put_float(buf, 28, wind_heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRISONICA_WIND_LEN);
#else
    mavlink_trisonica_wind_t packet;
    packet.wind_n = wind_n;
    packet.wind_e = wind_e;
    packet.wind_d = wind_d;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.wind_speed = wind_speed;
    packet.wind_heading = wind_heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRISONICA_WIND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRISONICA_WIND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_CRC);
}

/**
 * @brief Encode a trisonica_wind struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param trisonica_wind C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trisonica_wind_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_trisonica_wind_t* trisonica_wind)
{
    return mavlink_msg_trisonica_wind_pack(system_id, component_id, msg, trisonica_wind->wind_n, trisonica_wind->wind_e, trisonica_wind->wind_d, trisonica_wind->temperature, trisonica_wind->humidity, trisonica_wind->pressure, trisonica_wind->wind_speed, trisonica_wind->wind_heading);
}

/**
 * @brief Encode a trisonica_wind struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trisonica_wind C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trisonica_wind_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_trisonica_wind_t* trisonica_wind)
{
    return mavlink_msg_trisonica_wind_pack_chan(system_id, component_id, chan, msg, trisonica_wind->wind_n, trisonica_wind->wind_e, trisonica_wind->wind_d, trisonica_wind->temperature, trisonica_wind->humidity, trisonica_wind->pressure, trisonica_wind->wind_speed, trisonica_wind->wind_heading);
}

/**
 * @brief Send a trisonica_wind message
 * @param chan MAVLink channel to send the message
 *
 * @param wind_n [m/s] Velocity of air w.r.t ground in north
 * @param wind_e [m/s] Velocity of air w.r.t ground in east
 * @param wind_d [m/s] Velocity of air w.r.t ground in down
 * @param temperature [C] Temperature
 * @param humidity [percent] Humidity
 * @param pressure [Pa] Pressure
 * @param wind_speed [m/s] Absolute lateral wind speed
 * @param wind_heading [deg] Heading from north about down
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_trisonica_wind_send(mavlink_channel_t chan, float wind_n, float wind_e, float wind_d, float temperature, float humidity, float pressure, float wind_speed, float wind_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRISONICA_WIND_LEN];
    _mav_put_float(buf, 0, wind_n);
    _mav_put_float(buf, 4, wind_e);
    _mav_put_float(buf, 8, wind_d);
    _mav_put_float(buf, 12, temperature);
    _mav_put_float(buf, 16, humidity);
    _mav_put_float(buf, 20, pressure);
    _mav_put_float(buf, 24, wind_speed);
    _mav_put_float(buf, 28, wind_heading);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRISONICA_WIND, buf, MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_CRC);
#else
    mavlink_trisonica_wind_t packet;
    packet.wind_n = wind_n;
    packet.wind_e = wind_e;
    packet.wind_d = wind_d;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.pressure = pressure;
    packet.wind_speed = wind_speed;
    packet.wind_heading = wind_heading;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRISONICA_WIND, (const char *)&packet, MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_CRC);
#endif
}

/**
 * @brief Send a trisonica_wind message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_trisonica_wind_send_struct(mavlink_channel_t chan, const mavlink_trisonica_wind_t* trisonica_wind)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_trisonica_wind_send(chan, trisonica_wind->wind_n, trisonica_wind->wind_e, trisonica_wind->wind_d, trisonica_wind->temperature, trisonica_wind->humidity, trisonica_wind->pressure, trisonica_wind->wind_speed, trisonica_wind->wind_heading);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRISONICA_WIND, (const char *)trisonica_wind, MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRISONICA_WIND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_trisonica_wind_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float wind_n, float wind_e, float wind_d, float temperature, float humidity, float pressure, float wind_speed, float wind_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, wind_n);
    _mav_put_float(buf, 4, wind_e);
    _mav_put_float(buf, 8, wind_d);
    _mav_put_float(buf, 12, temperature);
    _mav_put_float(buf, 16, humidity);
    _mav_put_float(buf, 20, pressure);
    _mav_put_float(buf, 24, wind_speed);
    _mav_put_float(buf, 28, wind_heading);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRISONICA_WIND, buf, MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_CRC);
#else
    mavlink_trisonica_wind_t *packet = (mavlink_trisonica_wind_t *)msgbuf;
    packet->wind_n = wind_n;
    packet->wind_e = wind_e;
    packet->wind_d = wind_d;
    packet->temperature = temperature;
    packet->humidity = humidity;
    packet->pressure = pressure;
    packet->wind_speed = wind_speed;
    packet->wind_heading = wind_heading;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRISONICA_WIND, (const char *)packet, MAVLINK_MSG_ID_TRISONICA_WIND_MIN_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_LEN, MAVLINK_MSG_ID_TRISONICA_WIND_CRC);
#endif
}
#endif

#endif

// MESSAGE TRISONICA_WIND UNPACKING


/**
 * @brief Get field wind_n from trisonica_wind message
 *
 * @return [m/s] Velocity of air w.r.t ground in north
 */
static inline float mavlink_msg_trisonica_wind_get_wind_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field wind_e from trisonica_wind message
 *
 * @return [m/s] Velocity of air w.r.t ground in east
 */
static inline float mavlink_msg_trisonica_wind_get_wind_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field wind_d from trisonica_wind message
 *
 * @return [m/s] Velocity of air w.r.t ground in down
 */
static inline float mavlink_msg_trisonica_wind_get_wind_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature from trisonica_wind message
 *
 * @return [C] Temperature
 */
static inline float mavlink_msg_trisonica_wind_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field humidity from trisonica_wind message
 *
 * @return [percent] Humidity
 */
static inline float mavlink_msg_trisonica_wind_get_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pressure from trisonica_wind message
 *
 * @return [Pa] Pressure
 */
static inline float mavlink_msg_trisonica_wind_get_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field wind_speed from trisonica_wind message
 *
 * @return [m/s] Absolute lateral wind speed
 */
static inline float mavlink_msg_trisonica_wind_get_wind_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field wind_heading from trisonica_wind message
 *
 * @return [deg] Heading from north about down
 */
static inline float mavlink_msg_trisonica_wind_get_wind_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a trisonica_wind message into a struct
 *
 * @param msg The message to decode
 * @param trisonica_wind C-struct to decode the message contents into
 */
static inline void mavlink_msg_trisonica_wind_decode(const mavlink_message_t* msg, mavlink_trisonica_wind_t* trisonica_wind)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    trisonica_wind->wind_n = mavlink_msg_trisonica_wind_get_wind_n(msg);
    trisonica_wind->wind_e = mavlink_msg_trisonica_wind_get_wind_e(msg);
    trisonica_wind->wind_d = mavlink_msg_trisonica_wind_get_wind_d(msg);
    trisonica_wind->temperature = mavlink_msg_trisonica_wind_get_temperature(msg);
    trisonica_wind->humidity = mavlink_msg_trisonica_wind_get_humidity(msg);
    trisonica_wind->pressure = mavlink_msg_trisonica_wind_get_pressure(msg);
    trisonica_wind->wind_speed = mavlink_msg_trisonica_wind_get_wind_speed(msg);
    trisonica_wind->wind_heading = mavlink_msg_trisonica_wind_get_wind_heading(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRISONICA_WIND_LEN? msg->len : MAVLINK_MSG_ID_TRISONICA_WIND_LEN;
        memset(trisonica_wind, 0, MAVLINK_MSG_ID_TRISONICA_WIND_LEN);
    memcpy(trisonica_wind, _MAV_PAYLOAD(msg), len);
#endif
}
