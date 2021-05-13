#pragma once
// MESSAGE PACKAGE_COMMANDER_STATUS PACKING

#define MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS 7008


typedef struct __mavlink_package_commander_status_t {
 uint64_t autoretract_time_utc; /*< [ms]  Time when retract will be initialized. If nonzero.*/
 float impact_pos_x; /*< [m/s] Position to retract from, x. (Trajectory can ignore)*/
 float impact_pos_y; /*< [m/s] Position to retract from, y. (Trajectory can ignore)*/
 float impact_pos_z; /*< [m/s] Position to retract from, z. (Trajectory can ignore)*/
 float target_impact_pos_x; /*< [m/s] Position to retract from, x. (Trajectory can ignore)*/
 float target_impact_pos_y; /*< [m/s] Position to retract from, y. (Trajectory can ignore)*/
 float target_impact_pos_z; /*< [m/s] Position to retract from, z. (Trajectory can ignore)*/
 uint8_t is_active; /*< [boolean]  Actively following a trajectory*/
 uint8_t on_ground; /*< [boolean]  On the ground*/
 uint8_t autoretract_allowed; /*< [boolean]  autoretract currently allowed by the trajectory*/
 uint8_t autoretract_enabed; /*< [boolean]  Mission node enabled autoretraction*/
 uint8_t ready_for_autoretract; /*< [boolean]  Mission node enabled autoretraction*/
} mavlink_package_commander_status_t;

#define MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN 37
#define MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN 37
#define MAVLINK_MSG_ID_7008_LEN 37
#define MAVLINK_MSG_ID_7008_MIN_LEN 37

#define MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC 190
#define MAVLINK_MSG_ID_7008_CRC 190



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PACKAGE_COMMANDER_STATUS { \
    7008, \
    "PACKAGE_COMMANDER_STATUS", \
    12, \
    {  { "is_active", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_package_commander_status_t, is_active) }, \
         { "on_ground", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_package_commander_status_t, on_ground) }, \
         { "autoretract_allowed", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_package_commander_status_t, autoretract_allowed) }, \
         { "autoretract_enabed", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_package_commander_status_t, autoretract_enabed) }, \
         { "autoretract_time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_package_commander_status_t, autoretract_time_utc) }, \
         { "ready_for_autoretract", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_package_commander_status_t, ready_for_autoretract) }, \
         { "impact_pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_package_commander_status_t, impact_pos_x) }, \
         { "impact_pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_package_commander_status_t, impact_pos_y) }, \
         { "impact_pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_package_commander_status_t, impact_pos_z) }, \
         { "target_impact_pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_package_commander_status_t, target_impact_pos_x) }, \
         { "target_impact_pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_package_commander_status_t, target_impact_pos_y) }, \
         { "target_impact_pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_package_commander_status_t, target_impact_pos_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PACKAGE_COMMANDER_STATUS { \
    "PACKAGE_COMMANDER_STATUS", \
    12, \
    {  { "is_active", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_package_commander_status_t, is_active) }, \
         { "on_ground", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_package_commander_status_t, on_ground) }, \
         { "autoretract_allowed", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_package_commander_status_t, autoretract_allowed) }, \
         { "autoretract_enabed", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_package_commander_status_t, autoretract_enabed) }, \
         { "autoretract_time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_package_commander_status_t, autoretract_time_utc) }, \
         { "ready_for_autoretract", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_package_commander_status_t, ready_for_autoretract) }, \
         { "impact_pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_package_commander_status_t, impact_pos_x) }, \
         { "impact_pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_package_commander_status_t, impact_pos_y) }, \
         { "impact_pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_package_commander_status_t, impact_pos_z) }, \
         { "target_impact_pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_package_commander_status_t, target_impact_pos_x) }, \
         { "target_impact_pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_package_commander_status_t, target_impact_pos_y) }, \
         { "target_impact_pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_package_commander_status_t, target_impact_pos_z) }, \
         } \
}
#endif

/**
 * @brief Pack a package_commander_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_active [boolean]  Actively following a trajectory
 * @param on_ground [boolean]  On the ground
 * @param autoretract_allowed [boolean]  autoretract currently allowed by the trajectory
 * @param autoretract_enabed [boolean]  Mission node enabled autoretraction
 * @param autoretract_time_utc [ms]  Time when retract will be initialized. If nonzero.
 * @param ready_for_autoretract [boolean]  Mission node enabled autoretraction
 * @param impact_pos_x [m/s] Position to retract from, x. (Trajectory can ignore)
 * @param impact_pos_y [m/s] Position to retract from, y. (Trajectory can ignore)
 * @param impact_pos_z [m/s] Position to retract from, z. (Trajectory can ignore)
 * @param target_impact_pos_x [m/s] Position to retract from, x. (Trajectory can ignore)
 * @param target_impact_pos_y [m/s] Position to retract from, y. (Trajectory can ignore)
 * @param target_impact_pos_z [m/s] Position to retract from, z. (Trajectory can ignore)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_package_commander_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t is_active, uint8_t on_ground, uint8_t autoretract_allowed, uint8_t autoretract_enabed, uint64_t autoretract_time_utc, uint8_t ready_for_autoretract, float impact_pos_x, float impact_pos_y, float impact_pos_z, float target_impact_pos_x, float target_impact_pos_y, float target_impact_pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, autoretract_time_utc);
    _mav_put_float(buf, 8, impact_pos_x);
    _mav_put_float(buf, 12, impact_pos_y);
    _mav_put_float(buf, 16, impact_pos_z);
    _mav_put_float(buf, 20, target_impact_pos_x);
    _mav_put_float(buf, 24, target_impact_pos_y);
    _mav_put_float(buf, 28, target_impact_pos_z);
    _mav_put_uint8_t(buf, 32, is_active);
    _mav_put_uint8_t(buf, 33, on_ground);
    _mav_put_uint8_t(buf, 34, autoretract_allowed);
    _mav_put_uint8_t(buf, 35, autoretract_enabed);
    _mav_put_uint8_t(buf, 36, ready_for_autoretract);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN);
#else
    mavlink_package_commander_status_t packet;
    packet.autoretract_time_utc = autoretract_time_utc;
    packet.impact_pos_x = impact_pos_x;
    packet.impact_pos_y = impact_pos_y;
    packet.impact_pos_z = impact_pos_z;
    packet.target_impact_pos_x = target_impact_pos_x;
    packet.target_impact_pos_y = target_impact_pos_y;
    packet.target_impact_pos_z = target_impact_pos_z;
    packet.is_active = is_active;
    packet.on_ground = on_ground;
    packet.autoretract_allowed = autoretract_allowed;
    packet.autoretract_enabed = autoretract_enabed;
    packet.ready_for_autoretract = ready_for_autoretract;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC);
}

/**
 * @brief Pack a package_commander_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param is_active [boolean]  Actively following a trajectory
 * @param on_ground [boolean]  On the ground
 * @param autoretract_allowed [boolean]  autoretract currently allowed by the trajectory
 * @param autoretract_enabed [boolean]  Mission node enabled autoretraction
 * @param autoretract_time_utc [ms]  Time when retract will be initialized. If nonzero.
 * @param ready_for_autoretract [boolean]  Mission node enabled autoretraction
 * @param impact_pos_x [m/s] Position to retract from, x. (Trajectory can ignore)
 * @param impact_pos_y [m/s] Position to retract from, y. (Trajectory can ignore)
 * @param impact_pos_z [m/s] Position to retract from, z. (Trajectory can ignore)
 * @param target_impact_pos_x [m/s] Position to retract from, x. (Trajectory can ignore)
 * @param target_impact_pos_y [m/s] Position to retract from, y. (Trajectory can ignore)
 * @param target_impact_pos_z [m/s] Position to retract from, z. (Trajectory can ignore)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_package_commander_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t is_active,uint8_t on_ground,uint8_t autoretract_allowed,uint8_t autoretract_enabed,uint64_t autoretract_time_utc,uint8_t ready_for_autoretract,float impact_pos_x,float impact_pos_y,float impact_pos_z,float target_impact_pos_x,float target_impact_pos_y,float target_impact_pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, autoretract_time_utc);
    _mav_put_float(buf, 8, impact_pos_x);
    _mav_put_float(buf, 12, impact_pos_y);
    _mav_put_float(buf, 16, impact_pos_z);
    _mav_put_float(buf, 20, target_impact_pos_x);
    _mav_put_float(buf, 24, target_impact_pos_y);
    _mav_put_float(buf, 28, target_impact_pos_z);
    _mav_put_uint8_t(buf, 32, is_active);
    _mav_put_uint8_t(buf, 33, on_ground);
    _mav_put_uint8_t(buf, 34, autoretract_allowed);
    _mav_put_uint8_t(buf, 35, autoretract_enabed);
    _mav_put_uint8_t(buf, 36, ready_for_autoretract);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN);
#else
    mavlink_package_commander_status_t packet;
    packet.autoretract_time_utc = autoretract_time_utc;
    packet.impact_pos_x = impact_pos_x;
    packet.impact_pos_y = impact_pos_y;
    packet.impact_pos_z = impact_pos_z;
    packet.target_impact_pos_x = target_impact_pos_x;
    packet.target_impact_pos_y = target_impact_pos_y;
    packet.target_impact_pos_z = target_impact_pos_z;
    packet.is_active = is_active;
    packet.on_ground = on_ground;
    packet.autoretract_allowed = autoretract_allowed;
    packet.autoretract_enabed = autoretract_enabed;
    packet.ready_for_autoretract = ready_for_autoretract;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC);
}

/**
 * @brief Encode a package_commander_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param package_commander_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_package_commander_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_package_commander_status_t* package_commander_status)
{
    return mavlink_msg_package_commander_status_pack(system_id, component_id, msg, package_commander_status->is_active, package_commander_status->on_ground, package_commander_status->autoretract_allowed, package_commander_status->autoretract_enabed, package_commander_status->autoretract_time_utc, package_commander_status->ready_for_autoretract, package_commander_status->impact_pos_x, package_commander_status->impact_pos_y, package_commander_status->impact_pos_z, package_commander_status->target_impact_pos_x, package_commander_status->target_impact_pos_y, package_commander_status->target_impact_pos_z);
}

/**
 * @brief Encode a package_commander_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param package_commander_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_package_commander_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_package_commander_status_t* package_commander_status)
{
    return mavlink_msg_package_commander_status_pack_chan(system_id, component_id, chan, msg, package_commander_status->is_active, package_commander_status->on_ground, package_commander_status->autoretract_allowed, package_commander_status->autoretract_enabed, package_commander_status->autoretract_time_utc, package_commander_status->ready_for_autoretract, package_commander_status->impact_pos_x, package_commander_status->impact_pos_y, package_commander_status->impact_pos_z, package_commander_status->target_impact_pos_x, package_commander_status->target_impact_pos_y, package_commander_status->target_impact_pos_z);
}

/**
 * @brief Send a package_commander_status message
 * @param chan MAVLink channel to send the message
 *
 * @param is_active [boolean]  Actively following a trajectory
 * @param on_ground [boolean]  On the ground
 * @param autoretract_allowed [boolean]  autoretract currently allowed by the trajectory
 * @param autoretract_enabed [boolean]  Mission node enabled autoretraction
 * @param autoretract_time_utc [ms]  Time when retract will be initialized. If nonzero.
 * @param ready_for_autoretract [boolean]  Mission node enabled autoretraction
 * @param impact_pos_x [m/s] Position to retract from, x. (Trajectory can ignore)
 * @param impact_pos_y [m/s] Position to retract from, y. (Trajectory can ignore)
 * @param impact_pos_z [m/s] Position to retract from, z. (Trajectory can ignore)
 * @param target_impact_pos_x [m/s] Position to retract from, x. (Trajectory can ignore)
 * @param target_impact_pos_y [m/s] Position to retract from, y. (Trajectory can ignore)
 * @param target_impact_pos_z [m/s] Position to retract from, z. (Trajectory can ignore)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_package_commander_status_send(mavlink_channel_t chan, uint8_t is_active, uint8_t on_ground, uint8_t autoretract_allowed, uint8_t autoretract_enabed, uint64_t autoretract_time_utc, uint8_t ready_for_autoretract, float impact_pos_x, float impact_pos_y, float impact_pos_z, float target_impact_pos_x, float target_impact_pos_y, float target_impact_pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, autoretract_time_utc);
    _mav_put_float(buf, 8, impact_pos_x);
    _mav_put_float(buf, 12, impact_pos_y);
    _mav_put_float(buf, 16, impact_pos_z);
    _mav_put_float(buf, 20, target_impact_pos_x);
    _mav_put_float(buf, 24, target_impact_pos_y);
    _mav_put_float(buf, 28, target_impact_pos_z);
    _mav_put_uint8_t(buf, 32, is_active);
    _mav_put_uint8_t(buf, 33, on_ground);
    _mav_put_uint8_t(buf, 34, autoretract_allowed);
    _mav_put_uint8_t(buf, 35, autoretract_enabed);
    _mav_put_uint8_t(buf, 36, ready_for_autoretract);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS, buf, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC);
#else
    mavlink_package_commander_status_t packet;
    packet.autoretract_time_utc = autoretract_time_utc;
    packet.impact_pos_x = impact_pos_x;
    packet.impact_pos_y = impact_pos_y;
    packet.impact_pos_z = impact_pos_z;
    packet.target_impact_pos_x = target_impact_pos_x;
    packet.target_impact_pos_y = target_impact_pos_y;
    packet.target_impact_pos_z = target_impact_pos_z;
    packet.is_active = is_active;
    packet.on_ground = on_ground;
    packet.autoretract_allowed = autoretract_allowed;
    packet.autoretract_enabed = autoretract_enabed;
    packet.ready_for_autoretract = ready_for_autoretract;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC);
#endif
}

/**
 * @brief Send a package_commander_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_package_commander_status_send_struct(mavlink_channel_t chan, const mavlink_package_commander_status_t* package_commander_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_package_commander_status_send(chan, package_commander_status->is_active, package_commander_status->on_ground, package_commander_status->autoretract_allowed, package_commander_status->autoretract_enabed, package_commander_status->autoretract_time_utc, package_commander_status->ready_for_autoretract, package_commander_status->impact_pos_x, package_commander_status->impact_pos_y, package_commander_status->impact_pos_z, package_commander_status->target_impact_pos_x, package_commander_status->target_impact_pos_y, package_commander_status->target_impact_pos_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS, (const char *)package_commander_status, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_package_commander_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t is_active, uint8_t on_ground, uint8_t autoretract_allowed, uint8_t autoretract_enabed, uint64_t autoretract_time_utc, uint8_t ready_for_autoretract, float impact_pos_x, float impact_pos_y, float impact_pos_z, float target_impact_pos_x, float target_impact_pos_y, float target_impact_pos_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, autoretract_time_utc);
    _mav_put_float(buf, 8, impact_pos_x);
    _mav_put_float(buf, 12, impact_pos_y);
    _mav_put_float(buf, 16, impact_pos_z);
    _mav_put_float(buf, 20, target_impact_pos_x);
    _mav_put_float(buf, 24, target_impact_pos_y);
    _mav_put_float(buf, 28, target_impact_pos_z);
    _mav_put_uint8_t(buf, 32, is_active);
    _mav_put_uint8_t(buf, 33, on_ground);
    _mav_put_uint8_t(buf, 34, autoretract_allowed);
    _mav_put_uint8_t(buf, 35, autoretract_enabed);
    _mav_put_uint8_t(buf, 36, ready_for_autoretract);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS, buf, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC);
#else
    mavlink_package_commander_status_t *packet = (mavlink_package_commander_status_t *)msgbuf;
    packet->autoretract_time_utc = autoretract_time_utc;
    packet->impact_pos_x = impact_pos_x;
    packet->impact_pos_y = impact_pos_y;
    packet->impact_pos_z = impact_pos_z;
    packet->target_impact_pos_x = target_impact_pos_x;
    packet->target_impact_pos_y = target_impact_pos_y;
    packet->target_impact_pos_z = target_impact_pos_z;
    packet->is_active = is_active;
    packet->on_ground = on_ground;
    packet->autoretract_allowed = autoretract_allowed;
    packet->autoretract_enabed = autoretract_enabed;
    packet->ready_for_autoretract = ready_for_autoretract;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS, (const char *)packet, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_MIN_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE PACKAGE_COMMANDER_STATUS UNPACKING


/**
 * @brief Get field is_active from package_commander_status message
 *
 * @return [boolean]  Actively following a trajectory
 */
static inline uint8_t mavlink_msg_package_commander_status_get_is_active(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field on_ground from package_commander_status message
 *
 * @return [boolean]  On the ground
 */
static inline uint8_t mavlink_msg_package_commander_status_get_on_ground(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field autoretract_allowed from package_commander_status message
 *
 * @return [boolean]  autoretract currently allowed by the trajectory
 */
static inline uint8_t mavlink_msg_package_commander_status_get_autoretract_allowed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field autoretract_enabed from package_commander_status message
 *
 * @return [boolean]  Mission node enabled autoretraction
 */
static inline uint8_t mavlink_msg_package_commander_status_get_autoretract_enabed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field autoretract_time_utc from package_commander_status message
 *
 * @return [ms]  Time when retract will be initialized. If nonzero.
 */
static inline uint64_t mavlink_msg_package_commander_status_get_autoretract_time_utc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field ready_for_autoretract from package_commander_status message
 *
 * @return [boolean]  Mission node enabled autoretraction
 */
static inline uint8_t mavlink_msg_package_commander_status_get_ready_for_autoretract(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field impact_pos_x from package_commander_status message
 *
 * @return [m/s] Position to retract from, x. (Trajectory can ignore)
 */
static inline float mavlink_msg_package_commander_status_get_impact_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field impact_pos_y from package_commander_status message
 *
 * @return [m/s] Position to retract from, y. (Trajectory can ignore)
 */
static inline float mavlink_msg_package_commander_status_get_impact_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field impact_pos_z from package_commander_status message
 *
 * @return [m/s] Position to retract from, z. (Trajectory can ignore)
 */
static inline float mavlink_msg_package_commander_status_get_impact_pos_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field target_impact_pos_x from package_commander_status message
 *
 * @return [m/s] Position to retract from, x. (Trajectory can ignore)
 */
static inline float mavlink_msg_package_commander_status_get_target_impact_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field target_impact_pos_y from package_commander_status message
 *
 * @return [m/s] Position to retract from, y. (Trajectory can ignore)
 */
static inline float mavlink_msg_package_commander_status_get_target_impact_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field target_impact_pos_z from package_commander_status message
 *
 * @return [m/s] Position to retract from, z. (Trajectory can ignore)
 */
static inline float mavlink_msg_package_commander_status_get_target_impact_pos_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a package_commander_status message into a struct
 *
 * @param msg The message to decode
 * @param package_commander_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_package_commander_status_decode(const mavlink_message_t* msg, mavlink_package_commander_status_t* package_commander_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    package_commander_status->autoretract_time_utc = mavlink_msg_package_commander_status_get_autoretract_time_utc(msg);
    package_commander_status->impact_pos_x = mavlink_msg_package_commander_status_get_impact_pos_x(msg);
    package_commander_status->impact_pos_y = mavlink_msg_package_commander_status_get_impact_pos_y(msg);
    package_commander_status->impact_pos_z = mavlink_msg_package_commander_status_get_impact_pos_z(msg);
    package_commander_status->target_impact_pos_x = mavlink_msg_package_commander_status_get_target_impact_pos_x(msg);
    package_commander_status->target_impact_pos_y = mavlink_msg_package_commander_status_get_target_impact_pos_y(msg);
    package_commander_status->target_impact_pos_z = mavlink_msg_package_commander_status_get_target_impact_pos_z(msg);
    package_commander_status->is_active = mavlink_msg_package_commander_status_get_is_active(msg);
    package_commander_status->on_ground = mavlink_msg_package_commander_status_get_on_ground(msg);
    package_commander_status->autoretract_allowed = mavlink_msg_package_commander_status_get_autoretract_allowed(msg);
    package_commander_status->autoretract_enabed = mavlink_msg_package_commander_status_get_autoretract_enabed(msg);
    package_commander_status->ready_for_autoretract = mavlink_msg_package_commander_status_get_ready_for_autoretract(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN;
        memset(package_commander_status, 0, MAVLINK_MSG_ID_PACKAGE_COMMANDER_STATUS_LEN);
    memcpy(package_commander_status, _MAV_PAYLOAD(msg), len);
#endif
}
