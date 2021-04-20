#pragma once
// MESSAGE WINCH_DEBUG_DATA PACKING

#define MAVLINK_MSG_ID_WINCH_DEBUG_DATA 7002


typedef struct __mavlink_winch_debug_data_t {
 float spool_x; /*< [m] Position of the winch motor (spool)*/
 float spool_v; /*< [m/s] Velocity of the winch motor (spool)*/
 float spool_x_setpoint; /*< [m] Commanded position of the winch motor (spool)*/
 float spool_v_setpoint; /*< [m/s] Commanded velocity of the winch motor (spool)*/
 float payout_x; /*< [m] Payout position of the winch encoder (roller)*/
 float payout_v; /*< [m/s] Payout velocity of the winch encoder (roller)*/
 float payout_x_setpoint; /*< [m] Commanded payout position of the winch encoder (roller)*/
 float payout_v_setpoint; /*< [m/s] Commanded payout velocity of the winch encoder (roller)*/
 float correction; /*< [m] Applied correction to the winch (from package.py)*/
 float datum; /*< [m] Zero datum for the winch (payout)*/
 float force; /*< [N] Line force on the winch (motor)*/
 float disparity; /*< [m] Disparity between spool and payout positions of the winch*/
 float current; /*< [A] Winch Current Draw*/
 float voltage; /*< [V] Winch Supply voltage*/
 uint16_t err; /*< [mask] Error code (see odrive docs to interpret)*/
 uint16_t motor_err; /*< [mask] Motor error code (see odrive docs to interpret)*/
 uint16_t encoder_err; /*< [mask] Encoder error code (see odrive docs to interpret)*/
 uint16_t state; /*<  Motor controller state*/
 uint8_t mode; /*<  Winch control mode*/
} mavlink_winch_debug_data_t;

#define MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN 65
#define MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN 65
#define MAVLINK_MSG_ID_7002_LEN 65
#define MAVLINK_MSG_ID_7002_MIN_LEN 65

#define MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC 62
#define MAVLINK_MSG_ID_7002_CRC 62



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WINCH_DEBUG_DATA { \
    7002, \
    "WINCH_DEBUG_DATA", \
    19, \
    {  { "spool_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_winch_debug_data_t, spool_x) }, \
         { "spool_v", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_winch_debug_data_t, spool_v) }, \
         { "spool_x_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_winch_debug_data_t, spool_x_setpoint) }, \
         { "spool_v_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_winch_debug_data_t, spool_v_setpoint) }, \
         { "payout_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_winch_debug_data_t, payout_x) }, \
         { "payout_v", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_winch_debug_data_t, payout_v) }, \
         { "payout_x_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_winch_debug_data_t, payout_x_setpoint) }, \
         { "payout_v_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_winch_debug_data_t, payout_v_setpoint) }, \
         { "correction", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_winch_debug_data_t, correction) }, \
         { "datum", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_winch_debug_data_t, datum) }, \
         { "force", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_winch_debug_data_t, force) }, \
         { "disparity", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_winch_debug_data_t, disparity) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_winch_debug_data_t, mode) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_winch_debug_data_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_winch_debug_data_t, voltage) }, \
         { "err", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_winch_debug_data_t, err) }, \
         { "motor_err", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_winch_debug_data_t, motor_err) }, \
         { "encoder_err", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_winch_debug_data_t, encoder_err) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_winch_debug_data_t, state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WINCH_DEBUG_DATA { \
    "WINCH_DEBUG_DATA", \
    19, \
    {  { "spool_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_winch_debug_data_t, spool_x) }, \
         { "spool_v", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_winch_debug_data_t, spool_v) }, \
         { "spool_x_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_winch_debug_data_t, spool_x_setpoint) }, \
         { "spool_v_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_winch_debug_data_t, spool_v_setpoint) }, \
         { "payout_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_winch_debug_data_t, payout_x) }, \
         { "payout_v", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_winch_debug_data_t, payout_v) }, \
         { "payout_x_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_winch_debug_data_t, payout_x_setpoint) }, \
         { "payout_v_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_winch_debug_data_t, payout_v_setpoint) }, \
         { "correction", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_winch_debug_data_t, correction) }, \
         { "datum", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_winch_debug_data_t, datum) }, \
         { "force", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_winch_debug_data_t, force) }, \
         { "disparity", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_winch_debug_data_t, disparity) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_winch_debug_data_t, mode) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_winch_debug_data_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_winch_debug_data_t, voltage) }, \
         { "err", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_winch_debug_data_t, err) }, \
         { "motor_err", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_winch_debug_data_t, motor_err) }, \
         { "encoder_err", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_winch_debug_data_t, encoder_err) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_winch_debug_data_t, state) }, \
         } \
}
#endif

/**
 * @brief Pack a winch_debug_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param spool_x [m] Position of the winch motor (spool)
 * @param spool_v [m/s] Velocity of the winch motor (spool)
 * @param spool_x_setpoint [m] Commanded position of the winch motor (spool)
 * @param spool_v_setpoint [m/s] Commanded velocity of the winch motor (spool)
 * @param payout_x [m] Payout position of the winch encoder (roller)
 * @param payout_v [m/s] Payout velocity of the winch encoder (roller)
 * @param payout_x_setpoint [m] Commanded payout position of the winch encoder (roller)
 * @param payout_v_setpoint [m/s] Commanded payout velocity of the winch encoder (roller)
 * @param correction [m] Applied correction to the winch (from package.py)
 * @param datum [m] Zero datum for the winch (payout)
 * @param force [N] Line force on the winch (motor)
 * @param disparity [m] Disparity between spool and payout positions of the winch
 * @param mode  Winch control mode
 * @param current [A] Winch Current Draw
 * @param voltage [V] Winch Supply voltage
 * @param err [mask] Error code (see odrive docs to interpret)
 * @param motor_err [mask] Motor error code (see odrive docs to interpret)
 * @param encoder_err [mask] Encoder error code (see odrive docs to interpret)
 * @param state  Motor controller state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_winch_debug_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float spool_x, float spool_v, float spool_x_setpoint, float spool_v_setpoint, float payout_x, float payout_v, float payout_x_setpoint, float payout_v_setpoint, float correction, float datum, float force, float disparity, uint8_t mode, float current, float voltage, uint16_t err, uint16_t motor_err, uint16_t encoder_err, uint16_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN];
    _mav_put_float(buf, 0, spool_x);
    _mav_put_float(buf, 4, spool_v);
    _mav_put_float(buf, 8, spool_x_setpoint);
    _mav_put_float(buf, 12, spool_v_setpoint);
    _mav_put_float(buf, 16, payout_x);
    _mav_put_float(buf, 20, payout_v);
    _mav_put_float(buf, 24, payout_x_setpoint);
    _mav_put_float(buf, 28, payout_v_setpoint);
    _mav_put_float(buf, 32, correction);
    _mav_put_float(buf, 36, datum);
    _mav_put_float(buf, 40, force);
    _mav_put_float(buf, 44, disparity);
    _mav_put_float(buf, 48, current);
    _mav_put_float(buf, 52, voltage);
    _mav_put_uint16_t(buf, 56, err);
    _mav_put_uint16_t(buf, 58, motor_err);
    _mav_put_uint16_t(buf, 60, encoder_err);
    _mav_put_uint16_t(buf, 62, state);
    _mav_put_uint8_t(buf, 64, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN);
#else
    mavlink_winch_debug_data_t packet;
    packet.spool_x = spool_x;
    packet.spool_v = spool_v;
    packet.spool_x_setpoint = spool_x_setpoint;
    packet.spool_v_setpoint = spool_v_setpoint;
    packet.payout_x = payout_x;
    packet.payout_v = payout_v;
    packet.payout_x_setpoint = payout_x_setpoint;
    packet.payout_v_setpoint = payout_v_setpoint;
    packet.correction = correction;
    packet.datum = datum;
    packet.force = force;
    packet.disparity = disparity;
    packet.current = current;
    packet.voltage = voltage;
    packet.err = err;
    packet.motor_err = motor_err;
    packet.encoder_err = encoder_err;
    packet.state = state;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINCH_DEBUG_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC);
}

/**
 * @brief Pack a winch_debug_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param spool_x [m] Position of the winch motor (spool)
 * @param spool_v [m/s] Velocity of the winch motor (spool)
 * @param spool_x_setpoint [m] Commanded position of the winch motor (spool)
 * @param spool_v_setpoint [m/s] Commanded velocity of the winch motor (spool)
 * @param payout_x [m] Payout position of the winch encoder (roller)
 * @param payout_v [m/s] Payout velocity of the winch encoder (roller)
 * @param payout_x_setpoint [m] Commanded payout position of the winch encoder (roller)
 * @param payout_v_setpoint [m/s] Commanded payout velocity of the winch encoder (roller)
 * @param correction [m] Applied correction to the winch (from package.py)
 * @param datum [m] Zero datum for the winch (payout)
 * @param force [N] Line force on the winch (motor)
 * @param disparity [m] Disparity between spool and payout positions of the winch
 * @param mode  Winch control mode
 * @param current [A] Winch Current Draw
 * @param voltage [V] Winch Supply voltage
 * @param err [mask] Error code (see odrive docs to interpret)
 * @param motor_err [mask] Motor error code (see odrive docs to interpret)
 * @param encoder_err [mask] Encoder error code (see odrive docs to interpret)
 * @param state  Motor controller state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_winch_debug_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float spool_x,float spool_v,float spool_x_setpoint,float spool_v_setpoint,float payout_x,float payout_v,float payout_x_setpoint,float payout_v_setpoint,float correction,float datum,float force,float disparity,uint8_t mode,float current,float voltage,uint16_t err,uint16_t motor_err,uint16_t encoder_err,uint16_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN];
    _mav_put_float(buf, 0, spool_x);
    _mav_put_float(buf, 4, spool_v);
    _mav_put_float(buf, 8, spool_x_setpoint);
    _mav_put_float(buf, 12, spool_v_setpoint);
    _mav_put_float(buf, 16, payout_x);
    _mav_put_float(buf, 20, payout_v);
    _mav_put_float(buf, 24, payout_x_setpoint);
    _mav_put_float(buf, 28, payout_v_setpoint);
    _mav_put_float(buf, 32, correction);
    _mav_put_float(buf, 36, datum);
    _mav_put_float(buf, 40, force);
    _mav_put_float(buf, 44, disparity);
    _mav_put_float(buf, 48, current);
    _mav_put_float(buf, 52, voltage);
    _mav_put_uint16_t(buf, 56, err);
    _mav_put_uint16_t(buf, 58, motor_err);
    _mav_put_uint16_t(buf, 60, encoder_err);
    _mav_put_uint16_t(buf, 62, state);
    _mav_put_uint8_t(buf, 64, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN);
#else
    mavlink_winch_debug_data_t packet;
    packet.spool_x = spool_x;
    packet.spool_v = spool_v;
    packet.spool_x_setpoint = spool_x_setpoint;
    packet.spool_v_setpoint = spool_v_setpoint;
    packet.payout_x = payout_x;
    packet.payout_v = payout_v;
    packet.payout_x_setpoint = payout_x_setpoint;
    packet.payout_v_setpoint = payout_v_setpoint;
    packet.correction = correction;
    packet.datum = datum;
    packet.force = force;
    packet.disparity = disparity;
    packet.current = current;
    packet.voltage = voltage;
    packet.err = err;
    packet.motor_err = motor_err;
    packet.encoder_err = encoder_err;
    packet.state = state;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINCH_DEBUG_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC);
}

/**
 * @brief Encode a winch_debug_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param winch_debug_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_winch_debug_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_winch_debug_data_t* winch_debug_data)
{
    return mavlink_msg_winch_debug_data_pack(system_id, component_id, msg, winch_debug_data->spool_x, winch_debug_data->spool_v, winch_debug_data->spool_x_setpoint, winch_debug_data->spool_v_setpoint, winch_debug_data->payout_x, winch_debug_data->payout_v, winch_debug_data->payout_x_setpoint, winch_debug_data->payout_v_setpoint, winch_debug_data->correction, winch_debug_data->datum, winch_debug_data->force, winch_debug_data->disparity, winch_debug_data->mode, winch_debug_data->current, winch_debug_data->voltage, winch_debug_data->err, winch_debug_data->motor_err, winch_debug_data->encoder_err, winch_debug_data->state);
}

/**
 * @brief Encode a winch_debug_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param winch_debug_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_winch_debug_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_winch_debug_data_t* winch_debug_data)
{
    return mavlink_msg_winch_debug_data_pack_chan(system_id, component_id, chan, msg, winch_debug_data->spool_x, winch_debug_data->spool_v, winch_debug_data->spool_x_setpoint, winch_debug_data->spool_v_setpoint, winch_debug_data->payout_x, winch_debug_data->payout_v, winch_debug_data->payout_x_setpoint, winch_debug_data->payout_v_setpoint, winch_debug_data->correction, winch_debug_data->datum, winch_debug_data->force, winch_debug_data->disparity, winch_debug_data->mode, winch_debug_data->current, winch_debug_data->voltage, winch_debug_data->err, winch_debug_data->motor_err, winch_debug_data->encoder_err, winch_debug_data->state);
}

/**
 * @brief Send a winch_debug_data message
 * @param chan MAVLink channel to send the message
 *
 * @param spool_x [m] Position of the winch motor (spool)
 * @param spool_v [m/s] Velocity of the winch motor (spool)
 * @param spool_x_setpoint [m] Commanded position of the winch motor (spool)
 * @param spool_v_setpoint [m/s] Commanded velocity of the winch motor (spool)
 * @param payout_x [m] Payout position of the winch encoder (roller)
 * @param payout_v [m/s] Payout velocity of the winch encoder (roller)
 * @param payout_x_setpoint [m] Commanded payout position of the winch encoder (roller)
 * @param payout_v_setpoint [m/s] Commanded payout velocity of the winch encoder (roller)
 * @param correction [m] Applied correction to the winch (from package.py)
 * @param datum [m] Zero datum for the winch (payout)
 * @param force [N] Line force on the winch (motor)
 * @param disparity [m] Disparity between spool and payout positions of the winch
 * @param mode  Winch control mode
 * @param current [A] Winch Current Draw
 * @param voltage [V] Winch Supply voltage
 * @param err [mask] Error code (see odrive docs to interpret)
 * @param motor_err [mask] Motor error code (see odrive docs to interpret)
 * @param encoder_err [mask] Encoder error code (see odrive docs to interpret)
 * @param state  Motor controller state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_winch_debug_data_send(mavlink_channel_t chan, float spool_x, float spool_v, float spool_x_setpoint, float spool_v_setpoint, float payout_x, float payout_v, float payout_x_setpoint, float payout_v_setpoint, float correction, float datum, float force, float disparity, uint8_t mode, float current, float voltage, uint16_t err, uint16_t motor_err, uint16_t encoder_err, uint16_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN];
    _mav_put_float(buf, 0, spool_x);
    _mav_put_float(buf, 4, spool_v);
    _mav_put_float(buf, 8, spool_x_setpoint);
    _mav_put_float(buf, 12, spool_v_setpoint);
    _mav_put_float(buf, 16, payout_x);
    _mav_put_float(buf, 20, payout_v);
    _mav_put_float(buf, 24, payout_x_setpoint);
    _mav_put_float(buf, 28, payout_v_setpoint);
    _mav_put_float(buf, 32, correction);
    _mav_put_float(buf, 36, datum);
    _mav_put_float(buf, 40, force);
    _mav_put_float(buf, 44, disparity);
    _mav_put_float(buf, 48, current);
    _mav_put_float(buf, 52, voltage);
    _mav_put_uint16_t(buf, 56, err);
    _mav_put_uint16_t(buf, 58, motor_err);
    _mav_put_uint16_t(buf, 60, encoder_err);
    _mav_put_uint16_t(buf, 62, state);
    _mav_put_uint8_t(buf, 64, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_DEBUG_DATA, buf, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC);
#else
    mavlink_winch_debug_data_t packet;
    packet.spool_x = spool_x;
    packet.spool_v = spool_v;
    packet.spool_x_setpoint = spool_x_setpoint;
    packet.spool_v_setpoint = spool_v_setpoint;
    packet.payout_x = payout_x;
    packet.payout_v = payout_v;
    packet.payout_x_setpoint = payout_x_setpoint;
    packet.payout_v_setpoint = payout_v_setpoint;
    packet.correction = correction;
    packet.datum = datum;
    packet.force = force;
    packet.disparity = disparity;
    packet.current = current;
    packet.voltage = voltage;
    packet.err = err;
    packet.motor_err = motor_err;
    packet.encoder_err = encoder_err;
    packet.state = state;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_DEBUG_DATA, (const char *)&packet, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC);
#endif
}

/**
 * @brief Send a winch_debug_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_winch_debug_data_send_struct(mavlink_channel_t chan, const mavlink_winch_debug_data_t* winch_debug_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_winch_debug_data_send(chan, winch_debug_data->spool_x, winch_debug_data->spool_v, winch_debug_data->spool_x_setpoint, winch_debug_data->spool_v_setpoint, winch_debug_data->payout_x, winch_debug_data->payout_v, winch_debug_data->payout_x_setpoint, winch_debug_data->payout_v_setpoint, winch_debug_data->correction, winch_debug_data->datum, winch_debug_data->force, winch_debug_data->disparity, winch_debug_data->mode, winch_debug_data->current, winch_debug_data->voltage, winch_debug_data->err, winch_debug_data->motor_err, winch_debug_data->encoder_err, winch_debug_data->state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_DEBUG_DATA, (const char *)winch_debug_data, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_winch_debug_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float spool_x, float spool_v, float spool_x_setpoint, float spool_v_setpoint, float payout_x, float payout_v, float payout_x_setpoint, float payout_v_setpoint, float correction, float datum, float force, float disparity, uint8_t mode, float current, float voltage, uint16_t err, uint16_t motor_err, uint16_t encoder_err, uint16_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, spool_x);
    _mav_put_float(buf, 4, spool_v);
    _mav_put_float(buf, 8, spool_x_setpoint);
    _mav_put_float(buf, 12, spool_v_setpoint);
    _mav_put_float(buf, 16, payout_x);
    _mav_put_float(buf, 20, payout_v);
    _mav_put_float(buf, 24, payout_x_setpoint);
    _mav_put_float(buf, 28, payout_v_setpoint);
    _mav_put_float(buf, 32, correction);
    _mav_put_float(buf, 36, datum);
    _mav_put_float(buf, 40, force);
    _mav_put_float(buf, 44, disparity);
    _mav_put_float(buf, 48, current);
    _mav_put_float(buf, 52, voltage);
    _mav_put_uint16_t(buf, 56, err);
    _mav_put_uint16_t(buf, 58, motor_err);
    _mav_put_uint16_t(buf, 60, encoder_err);
    _mav_put_uint16_t(buf, 62, state);
    _mav_put_uint8_t(buf, 64, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_DEBUG_DATA, buf, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC);
#else
    mavlink_winch_debug_data_t *packet = (mavlink_winch_debug_data_t *)msgbuf;
    packet->spool_x = spool_x;
    packet->spool_v = spool_v;
    packet->spool_x_setpoint = spool_x_setpoint;
    packet->spool_v_setpoint = spool_v_setpoint;
    packet->payout_x = payout_x;
    packet->payout_v = payout_v;
    packet->payout_x_setpoint = payout_x_setpoint;
    packet->payout_v_setpoint = payout_v_setpoint;
    packet->correction = correction;
    packet->datum = datum;
    packet->force = force;
    packet->disparity = disparity;
    packet->current = current;
    packet->voltage = voltage;
    packet->err = err;
    packet->motor_err = motor_err;
    packet->encoder_err = encoder_err;
    packet->state = state;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_DEBUG_DATA, (const char *)packet, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_MIN_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE WINCH_DEBUG_DATA UNPACKING


/**
 * @brief Get field spool_x from winch_debug_data message
 *
 * @return [m] Position of the winch motor (spool)
 */
static inline float mavlink_msg_winch_debug_data_get_spool_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field spool_v from winch_debug_data message
 *
 * @return [m/s] Velocity of the winch motor (spool)
 */
static inline float mavlink_msg_winch_debug_data_get_spool_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field spool_x_setpoint from winch_debug_data message
 *
 * @return [m] Commanded position of the winch motor (spool)
 */
static inline float mavlink_msg_winch_debug_data_get_spool_x_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field spool_v_setpoint from winch_debug_data message
 *
 * @return [m/s] Commanded velocity of the winch motor (spool)
 */
static inline float mavlink_msg_winch_debug_data_get_spool_v_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field payout_x from winch_debug_data message
 *
 * @return [m] Payout position of the winch encoder (roller)
 */
static inline float mavlink_msg_winch_debug_data_get_payout_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field payout_v from winch_debug_data message
 *
 * @return [m/s] Payout velocity of the winch encoder (roller)
 */
static inline float mavlink_msg_winch_debug_data_get_payout_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field payout_x_setpoint from winch_debug_data message
 *
 * @return [m] Commanded payout position of the winch encoder (roller)
 */
static inline float mavlink_msg_winch_debug_data_get_payout_x_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field payout_v_setpoint from winch_debug_data message
 *
 * @return [m/s] Commanded payout velocity of the winch encoder (roller)
 */
static inline float mavlink_msg_winch_debug_data_get_payout_v_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field correction from winch_debug_data message
 *
 * @return [m] Applied correction to the winch (from package.py)
 */
static inline float mavlink_msg_winch_debug_data_get_correction(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field datum from winch_debug_data message
 *
 * @return [m] Zero datum for the winch (payout)
 */
static inline float mavlink_msg_winch_debug_data_get_datum(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field force from winch_debug_data message
 *
 * @return [N] Line force on the winch (motor)
 */
static inline float mavlink_msg_winch_debug_data_get_force(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field disparity from winch_debug_data message
 *
 * @return [m] Disparity between spool and payout positions of the winch
 */
static inline float mavlink_msg_winch_debug_data_get_disparity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field mode from winch_debug_data message
 *
 * @return  Winch control mode
 */
static inline uint8_t mavlink_msg_winch_debug_data_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field current from winch_debug_data message
 *
 * @return [A] Winch Current Draw
 */
static inline float mavlink_msg_winch_debug_data_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field voltage from winch_debug_data message
 *
 * @return [V] Winch Supply voltage
 */
static inline float mavlink_msg_winch_debug_data_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field err from winch_debug_data message
 *
 * @return [mask] Error code (see odrive docs to interpret)
 */
static inline uint16_t mavlink_msg_winch_debug_data_get_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field motor_err from winch_debug_data message
 *
 * @return [mask] Motor error code (see odrive docs to interpret)
 */
static inline uint16_t mavlink_msg_winch_debug_data_get_motor_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  58);
}

/**
 * @brief Get field encoder_err from winch_debug_data message
 *
 * @return [mask] Encoder error code (see odrive docs to interpret)
 */
static inline uint16_t mavlink_msg_winch_debug_data_get_encoder_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field state from winch_debug_data message
 *
 * @return  Motor controller state
 */
static inline uint16_t mavlink_msg_winch_debug_data_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  62);
}

/**
 * @brief Decode a winch_debug_data message into a struct
 *
 * @param msg The message to decode
 * @param winch_debug_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_winch_debug_data_decode(const mavlink_message_t* msg, mavlink_winch_debug_data_t* winch_debug_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    winch_debug_data->spool_x = mavlink_msg_winch_debug_data_get_spool_x(msg);
    winch_debug_data->spool_v = mavlink_msg_winch_debug_data_get_spool_v(msg);
    winch_debug_data->spool_x_setpoint = mavlink_msg_winch_debug_data_get_spool_x_setpoint(msg);
    winch_debug_data->spool_v_setpoint = mavlink_msg_winch_debug_data_get_spool_v_setpoint(msg);
    winch_debug_data->payout_x = mavlink_msg_winch_debug_data_get_payout_x(msg);
    winch_debug_data->payout_v = mavlink_msg_winch_debug_data_get_payout_v(msg);
    winch_debug_data->payout_x_setpoint = mavlink_msg_winch_debug_data_get_payout_x_setpoint(msg);
    winch_debug_data->payout_v_setpoint = mavlink_msg_winch_debug_data_get_payout_v_setpoint(msg);
    winch_debug_data->correction = mavlink_msg_winch_debug_data_get_correction(msg);
    winch_debug_data->datum = mavlink_msg_winch_debug_data_get_datum(msg);
    winch_debug_data->force = mavlink_msg_winch_debug_data_get_force(msg);
    winch_debug_data->disparity = mavlink_msg_winch_debug_data_get_disparity(msg);
    winch_debug_data->current = mavlink_msg_winch_debug_data_get_current(msg);
    winch_debug_data->voltage = mavlink_msg_winch_debug_data_get_voltage(msg);
    winch_debug_data->err = mavlink_msg_winch_debug_data_get_err(msg);
    winch_debug_data->motor_err = mavlink_msg_winch_debug_data_get_motor_err(msg);
    winch_debug_data->encoder_err = mavlink_msg_winch_debug_data_get_encoder_err(msg);
    winch_debug_data->state = mavlink_msg_winch_debug_data_get_state(msg);
    winch_debug_data->mode = mavlink_msg_winch_debug_data_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN? msg->len : MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN;
        memset(winch_debug_data, 0, MAVLINK_MSG_ID_WINCH_DEBUG_DATA_LEN);
    memcpy(winch_debug_data, _MAV_PAYLOAD(msg), len);
#endif
}
