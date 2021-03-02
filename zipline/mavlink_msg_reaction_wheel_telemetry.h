#pragma once
// MESSAGE REACTION_WHEEL_TELEMETRY PACKING

#define MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY 7003


typedef struct __mavlink_reaction_wheel_telemetry_t {
 float velocity; /*< [rad/s] Velocity of the wheel*/
 float torque; /*< [Nm] Torque*/
 float current; /*< [A] Current*/
 float voltage; /*< [V] Voltage*/
 uint8_t err; /*< [mask] Error code (see odrive docs to interpret)*/
 uint8_t motor_err; /*< [mask] Motor error code (see odrive docs to interpret)*/
 uint8_t encoder_err; /*< [mask] Encoder error code (see odrive docs to interpret)*/
 uint8_t state; /*<  Motor controller state*/
} mavlink_reaction_wheel_telemetry_t;

#define MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN 20
#define MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN 20
#define MAVLINK_MSG_ID_7003_LEN 20
#define MAVLINK_MSG_ID_7003_MIN_LEN 20

#define MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC 185
#define MAVLINK_MSG_ID_7003_CRC 185



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REACTION_WHEEL_TELEMETRY { \
    7003, \
    "REACTION_WHEEL_TELEMETRY", \
    8, \
    {  { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_reaction_wheel_telemetry_t, velocity) }, \
         { "torque", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_reaction_wheel_telemetry_t, torque) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_reaction_wheel_telemetry_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_reaction_wheel_telemetry_t, voltage) }, \
         { "err", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_reaction_wheel_telemetry_t, err) }, \
         { "motor_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_reaction_wheel_telemetry_t, motor_err) }, \
         { "encoder_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_reaction_wheel_telemetry_t, encoder_err) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_reaction_wheel_telemetry_t, state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REACTION_WHEEL_TELEMETRY { \
    "REACTION_WHEEL_TELEMETRY", \
    8, \
    {  { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_reaction_wheel_telemetry_t, velocity) }, \
         { "torque", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_reaction_wheel_telemetry_t, torque) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_reaction_wheel_telemetry_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_reaction_wheel_telemetry_t, voltage) }, \
         { "err", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_reaction_wheel_telemetry_t, err) }, \
         { "motor_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_reaction_wheel_telemetry_t, motor_err) }, \
         { "encoder_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_reaction_wheel_telemetry_t, encoder_err) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_reaction_wheel_telemetry_t, state) }, \
         } \
}
#endif

/**
 * @brief Pack a reaction_wheel_telemetry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param velocity [rad/s] Velocity of the wheel
 * @param torque [Nm] Torque
 * @param current [A] Current
 * @param voltage [V] Voltage
 * @param err [mask] Error code (see odrive docs to interpret)
 * @param motor_err [mask] Motor error code (see odrive docs to interpret)
 * @param encoder_err [mask] Encoder error code (see odrive docs to interpret)
 * @param state  Motor controller state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reaction_wheel_telemetry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float velocity, float torque, float current, float voltage, uint8_t err, uint8_t motor_err, uint8_t encoder_err, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN];
    _mav_put_float(buf, 0, velocity);
    _mav_put_float(buf, 4, torque);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, voltage);
    _mav_put_uint8_t(buf, 16, err);
    _mav_put_uint8_t(buf, 17, motor_err);
    _mav_put_uint8_t(buf, 18, encoder_err);
    _mav_put_uint8_t(buf, 19, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN);
#else
    mavlink_reaction_wheel_telemetry_t packet;
    packet.velocity = velocity;
    packet.torque = torque;
    packet.current = current;
    packet.voltage = voltage;
    packet.err = err;
    packet.motor_err = motor_err;
    packet.encoder_err = encoder_err;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC);
}

/**
 * @brief Pack a reaction_wheel_telemetry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param velocity [rad/s] Velocity of the wheel
 * @param torque [Nm] Torque
 * @param current [A] Current
 * @param voltage [V] Voltage
 * @param err [mask] Error code (see odrive docs to interpret)
 * @param motor_err [mask] Motor error code (see odrive docs to interpret)
 * @param encoder_err [mask] Encoder error code (see odrive docs to interpret)
 * @param state  Motor controller state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_reaction_wheel_telemetry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float velocity,float torque,float current,float voltage,uint8_t err,uint8_t motor_err,uint8_t encoder_err,uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN];
    _mav_put_float(buf, 0, velocity);
    _mav_put_float(buf, 4, torque);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, voltage);
    _mav_put_uint8_t(buf, 16, err);
    _mav_put_uint8_t(buf, 17, motor_err);
    _mav_put_uint8_t(buf, 18, encoder_err);
    _mav_put_uint8_t(buf, 19, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN);
#else
    mavlink_reaction_wheel_telemetry_t packet;
    packet.velocity = velocity;
    packet.torque = torque;
    packet.current = current;
    packet.voltage = voltage;
    packet.err = err;
    packet.motor_err = motor_err;
    packet.encoder_err = encoder_err;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC);
}

/**
 * @brief Encode a reaction_wheel_telemetry struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param reaction_wheel_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reaction_wheel_telemetry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_reaction_wheel_telemetry_t* reaction_wheel_telemetry)
{
    return mavlink_msg_reaction_wheel_telemetry_pack(system_id, component_id, msg, reaction_wheel_telemetry->velocity, reaction_wheel_telemetry->torque, reaction_wheel_telemetry->current, reaction_wheel_telemetry->voltage, reaction_wheel_telemetry->err, reaction_wheel_telemetry->motor_err, reaction_wheel_telemetry->encoder_err, reaction_wheel_telemetry->state);
}

/**
 * @brief Encode a reaction_wheel_telemetry struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reaction_wheel_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_reaction_wheel_telemetry_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_reaction_wheel_telemetry_t* reaction_wheel_telemetry)
{
    return mavlink_msg_reaction_wheel_telemetry_pack_chan(system_id, component_id, chan, msg, reaction_wheel_telemetry->velocity, reaction_wheel_telemetry->torque, reaction_wheel_telemetry->current, reaction_wheel_telemetry->voltage, reaction_wheel_telemetry->err, reaction_wheel_telemetry->motor_err, reaction_wheel_telemetry->encoder_err, reaction_wheel_telemetry->state);
}

/**
 * @brief Send a reaction_wheel_telemetry message
 * @param chan MAVLink channel to send the message
 *
 * @param velocity [rad/s] Velocity of the wheel
 * @param torque [Nm] Torque
 * @param current [A] Current
 * @param voltage [V] Voltage
 * @param err [mask] Error code (see odrive docs to interpret)
 * @param motor_err [mask] Motor error code (see odrive docs to interpret)
 * @param encoder_err [mask] Encoder error code (see odrive docs to interpret)
 * @param state  Motor controller state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_reaction_wheel_telemetry_send(mavlink_channel_t chan, float velocity, float torque, float current, float voltage, uint8_t err, uint8_t motor_err, uint8_t encoder_err, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN];
    _mav_put_float(buf, 0, velocity);
    _mav_put_float(buf, 4, torque);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, voltage);
    _mav_put_uint8_t(buf, 16, err);
    _mav_put_uint8_t(buf, 17, motor_err);
    _mav_put_uint8_t(buf, 18, encoder_err);
    _mav_put_uint8_t(buf, 19, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY, buf, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC);
#else
    mavlink_reaction_wheel_telemetry_t packet;
    packet.velocity = velocity;
    packet.torque = torque;
    packet.current = current;
    packet.voltage = voltage;
    packet.err = err;
    packet.motor_err = motor_err;
    packet.encoder_err = encoder_err;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY, (const char *)&packet, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC);
#endif
}

/**
 * @brief Send a reaction_wheel_telemetry message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_reaction_wheel_telemetry_send_struct(mavlink_channel_t chan, const mavlink_reaction_wheel_telemetry_t* reaction_wheel_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_reaction_wheel_telemetry_send(chan, reaction_wheel_telemetry->velocity, reaction_wheel_telemetry->torque, reaction_wheel_telemetry->current, reaction_wheel_telemetry->voltage, reaction_wheel_telemetry->err, reaction_wheel_telemetry->motor_err, reaction_wheel_telemetry->encoder_err, reaction_wheel_telemetry->state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY, (const char *)reaction_wheel_telemetry, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC);
#endif
}

#if MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_reaction_wheel_telemetry_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float velocity, float torque, float current, float voltage, uint8_t err, uint8_t motor_err, uint8_t encoder_err, uint8_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, velocity);
    _mav_put_float(buf, 4, torque);
    _mav_put_float(buf, 8, current);
    _mav_put_float(buf, 12, voltage);
    _mav_put_uint8_t(buf, 16, err);
    _mav_put_uint8_t(buf, 17, motor_err);
    _mav_put_uint8_t(buf, 18, encoder_err);
    _mav_put_uint8_t(buf, 19, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY, buf, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC);
#else
    mavlink_reaction_wheel_telemetry_t *packet = (mavlink_reaction_wheel_telemetry_t *)msgbuf;
    packet->velocity = velocity;
    packet->torque = torque;
    packet->current = current;
    packet->voltage = voltage;
    packet->err = err;
    packet->motor_err = motor_err;
    packet->encoder_err = encoder_err;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY, (const char *)packet, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_CRC);
#endif
}
#endif

#endif

// MESSAGE REACTION_WHEEL_TELEMETRY UNPACKING


/**
 * @brief Get field velocity from reaction_wheel_telemetry message
 *
 * @return [rad/s] Velocity of the wheel
 */
static inline float mavlink_msg_reaction_wheel_telemetry_get_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field torque from reaction_wheel_telemetry message
 *
 * @return [Nm] Torque
 */
static inline float mavlink_msg_reaction_wheel_telemetry_get_torque(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field current from reaction_wheel_telemetry message
 *
 * @return [A] Current
 */
static inline float mavlink_msg_reaction_wheel_telemetry_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field voltage from reaction_wheel_telemetry message
 *
 * @return [V] Voltage
 */
static inline float mavlink_msg_reaction_wheel_telemetry_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field err from reaction_wheel_telemetry message
 *
 * @return [mask] Error code (see odrive docs to interpret)
 */
static inline uint8_t mavlink_msg_reaction_wheel_telemetry_get_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field motor_err from reaction_wheel_telemetry message
 *
 * @return [mask] Motor error code (see odrive docs to interpret)
 */
static inline uint8_t mavlink_msg_reaction_wheel_telemetry_get_motor_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field encoder_err from reaction_wheel_telemetry message
 *
 * @return [mask] Encoder error code (see odrive docs to interpret)
 */
static inline uint8_t mavlink_msg_reaction_wheel_telemetry_get_encoder_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field state from reaction_wheel_telemetry message
 *
 * @return  Motor controller state
 */
static inline uint8_t mavlink_msg_reaction_wheel_telemetry_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Decode a reaction_wheel_telemetry message into a struct
 *
 * @param msg The message to decode
 * @param reaction_wheel_telemetry C-struct to decode the message contents into
 */
static inline void mavlink_msg_reaction_wheel_telemetry_decode(const mavlink_message_t* msg, mavlink_reaction_wheel_telemetry_t* reaction_wheel_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    reaction_wheel_telemetry->velocity = mavlink_msg_reaction_wheel_telemetry_get_velocity(msg);
    reaction_wheel_telemetry->torque = mavlink_msg_reaction_wheel_telemetry_get_torque(msg);
    reaction_wheel_telemetry->current = mavlink_msg_reaction_wheel_telemetry_get_current(msg);
    reaction_wheel_telemetry->voltage = mavlink_msg_reaction_wheel_telemetry_get_voltage(msg);
    reaction_wheel_telemetry->err = mavlink_msg_reaction_wheel_telemetry_get_err(msg);
    reaction_wheel_telemetry->motor_err = mavlink_msg_reaction_wheel_telemetry_get_motor_err(msg);
    reaction_wheel_telemetry->encoder_err = mavlink_msg_reaction_wheel_telemetry_get_encoder_err(msg);
    reaction_wheel_telemetry->state = mavlink_msg_reaction_wheel_telemetry_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN? msg->len : MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN;
        memset(reaction_wheel_telemetry, 0, MAVLINK_MSG_ID_REACTION_WHEEL_TELEMETRY_LEN);
    memcpy(reaction_wheel_telemetry, _MAV_PAYLOAD(msg), len);
#endif
}
