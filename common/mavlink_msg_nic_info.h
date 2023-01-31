#pragma once
// MESSAGE NIC_INFO PACKING

#define MAVLINK_MSG_ID_NIC_INFO 337


typedef struct __mavlink_nic_info_t {
 uint64_t imei; /*<  Unique NIC International Mobile Equipment Identity Number.*/
 uint64_t iccid; /*<  Integrated Circuit Card Identification Number of SIM Card.*/
 uint64_t imsi; /*<   Current SIM International mobile subscriber identity.*/
 float firmware_version; /*<  The firmware version installed on the modem.*/
 uint8_t id; /*<  NIC instance number.*/
 uint8_t nic_id; /*<  Unique id for NIC (Vlan).*/
 char nic_model[50]; /*<  NIC model.*/
} mavlink_nic_info_t;

#define MAVLINK_MSG_ID_NIC_INFO_LEN 80
#define MAVLINK_MSG_ID_NIC_INFO_MIN_LEN 80
#define MAVLINK_MSG_ID_337_LEN 80
#define MAVLINK_MSG_ID_337_MIN_LEN 80

#define MAVLINK_MSG_ID_NIC_INFO_CRC 23
#define MAVLINK_MSG_ID_337_CRC 23

#define MAVLINK_MSG_NIC_INFO_FIELD_NIC_MODEL_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NIC_INFO { \
    337, \
    "NIC_INFO", \
    7, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_nic_info_t, id) }, \
         { "nic_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_nic_info_t, nic_id) }, \
         { "nic_model", NULL, MAVLINK_TYPE_CHAR, 50, 30, offsetof(mavlink_nic_info_t, nic_model) }, \
         { "imei", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nic_info_t, imei) }, \
         { "iccid", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_nic_info_t, iccid) }, \
         { "imsi", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_nic_info_t, imsi) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nic_info_t, firmware_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NIC_INFO { \
    "NIC_INFO", \
    7, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_nic_info_t, id) }, \
         { "nic_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_nic_info_t, nic_id) }, \
         { "nic_model", NULL, MAVLINK_TYPE_CHAR, 50, 30, offsetof(mavlink_nic_info_t, nic_model) }, \
         { "imei", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nic_info_t, imei) }, \
         { "iccid", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_nic_info_t, iccid) }, \
         { "imsi", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_nic_info_t, imsi) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nic_info_t, firmware_version) }, \
         } \
}
#endif

/**
 * @brief Pack a nic_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  NIC instance number.
 * @param nic_id  Unique id for NIC (Vlan).
 * @param nic_model  NIC model.
 * @param imei  Unique NIC International Mobile Equipment Identity Number.
 * @param iccid  Integrated Circuit Card Identification Number of SIM Card.
 * @param imsi   Current SIM International mobile subscriber identity.
 * @param firmware_version  The firmware version installed on the modem.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nic_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t nic_id, const char *nic_model, uint64_t imei, uint64_t iccid, uint64_t imsi, float firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NIC_INFO_LEN];
    _mav_put_uint64_t(buf, 0, imei);
    _mav_put_uint64_t(buf, 8, iccid);
    _mav_put_uint64_t(buf, 16, imsi);
    _mav_put_float(buf, 24, firmware_version);
    _mav_put_uint8_t(buf, 28, id);
    _mav_put_uint8_t(buf, 29, nic_id);
    _mav_put_char_array(buf, 30, nic_model, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NIC_INFO_LEN);
#else
    mavlink_nic_info_t packet;
    packet.imei = imei;
    packet.iccid = iccid;
    packet.imsi = imsi;
    packet.firmware_version = firmware_version;
    packet.id = id;
    packet.nic_id = nic_id;
    mav_array_memcpy(packet.nic_model, nic_model, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NIC_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NIC_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NIC_INFO_MIN_LEN, MAVLINK_MSG_ID_NIC_INFO_LEN, MAVLINK_MSG_ID_NIC_INFO_CRC);
}

/**
 * @brief Pack a nic_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  NIC instance number.
 * @param nic_id  Unique id for NIC (Vlan).
 * @param nic_model  NIC model.
 * @param imei  Unique NIC International Mobile Equipment Identity Number.
 * @param iccid  Integrated Circuit Card Identification Number of SIM Card.
 * @param imsi   Current SIM International mobile subscriber identity.
 * @param firmware_version  The firmware version installed on the modem.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nic_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t nic_id,const char *nic_model,uint64_t imei,uint64_t iccid,uint64_t imsi,float firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NIC_INFO_LEN];
    _mav_put_uint64_t(buf, 0, imei);
    _mav_put_uint64_t(buf, 8, iccid);
    _mav_put_uint64_t(buf, 16, imsi);
    _mav_put_float(buf, 24, firmware_version);
    _mav_put_uint8_t(buf, 28, id);
    _mav_put_uint8_t(buf, 29, nic_id);
    _mav_put_char_array(buf, 30, nic_model, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NIC_INFO_LEN);
#else
    mavlink_nic_info_t packet;
    packet.imei = imei;
    packet.iccid = iccid;
    packet.imsi = imsi;
    packet.firmware_version = firmware_version;
    packet.id = id;
    packet.nic_id = nic_id;
    mav_array_memcpy(packet.nic_model, nic_model, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NIC_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NIC_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NIC_INFO_MIN_LEN, MAVLINK_MSG_ID_NIC_INFO_LEN, MAVLINK_MSG_ID_NIC_INFO_CRC);
}

/**
 * @brief Encode a nic_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nic_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nic_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nic_info_t* nic_info)
{
    return mavlink_msg_nic_info_pack(system_id, component_id, msg, nic_info->id, nic_info->nic_id, nic_info->nic_model, nic_info->imei, nic_info->iccid, nic_info->imsi, nic_info->firmware_version);
}

/**
 * @brief Encode a nic_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nic_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nic_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nic_info_t* nic_info)
{
    return mavlink_msg_nic_info_pack_chan(system_id, component_id, chan, msg, nic_info->id, nic_info->nic_id, nic_info->nic_model, nic_info->imei, nic_info->iccid, nic_info->imsi, nic_info->firmware_version);
}

/**
 * @brief Send a nic_info message
 * @param chan MAVLink channel to send the message
 *
 * @param id  NIC instance number.
 * @param nic_id  Unique id for NIC (Vlan).
 * @param nic_model  NIC model.
 * @param imei  Unique NIC International Mobile Equipment Identity Number.
 * @param iccid  Integrated Circuit Card Identification Number of SIM Card.
 * @param imsi   Current SIM International mobile subscriber identity.
 * @param firmware_version  The firmware version installed on the modem.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nic_info_send(mavlink_channel_t chan, uint8_t id, uint8_t nic_id, const char *nic_model, uint64_t imei, uint64_t iccid, uint64_t imsi, float firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NIC_INFO_LEN];
    _mav_put_uint64_t(buf, 0, imei);
    _mav_put_uint64_t(buf, 8, iccid);
    _mav_put_uint64_t(buf, 16, imsi);
    _mav_put_float(buf, 24, firmware_version);
    _mav_put_uint8_t(buf, 28, id);
    _mav_put_uint8_t(buf, 29, nic_id);
    _mav_put_char_array(buf, 30, nic_model, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NIC_INFO, buf, MAVLINK_MSG_ID_NIC_INFO_MIN_LEN, MAVLINK_MSG_ID_NIC_INFO_LEN, MAVLINK_MSG_ID_NIC_INFO_CRC);
#else
    mavlink_nic_info_t packet;
    packet.imei = imei;
    packet.iccid = iccid;
    packet.imsi = imsi;
    packet.firmware_version = firmware_version;
    packet.id = id;
    packet.nic_id = nic_id;
    mav_array_memcpy(packet.nic_model, nic_model, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NIC_INFO, (const char *)&packet, MAVLINK_MSG_ID_NIC_INFO_MIN_LEN, MAVLINK_MSG_ID_NIC_INFO_LEN, MAVLINK_MSG_ID_NIC_INFO_CRC);
#endif
}

/**
 * @brief Send a nic_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_nic_info_send_struct(mavlink_channel_t chan, const mavlink_nic_info_t* nic_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_nic_info_send(chan, nic_info->id, nic_info->nic_id, nic_info->nic_model, nic_info->imei, nic_info->iccid, nic_info->imsi, nic_info->firmware_version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NIC_INFO, (const char *)nic_info, MAVLINK_MSG_ID_NIC_INFO_MIN_LEN, MAVLINK_MSG_ID_NIC_INFO_LEN, MAVLINK_MSG_ID_NIC_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_NIC_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nic_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t nic_id, const char *nic_model, uint64_t imei, uint64_t iccid, uint64_t imsi, float firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, imei);
    _mav_put_uint64_t(buf, 8, iccid);
    _mav_put_uint64_t(buf, 16, imsi);
    _mav_put_float(buf, 24, firmware_version);
    _mav_put_uint8_t(buf, 28, id);
    _mav_put_uint8_t(buf, 29, nic_id);
    _mav_put_char_array(buf, 30, nic_model, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NIC_INFO, buf, MAVLINK_MSG_ID_NIC_INFO_MIN_LEN, MAVLINK_MSG_ID_NIC_INFO_LEN, MAVLINK_MSG_ID_NIC_INFO_CRC);
#else
    mavlink_nic_info_t *packet = (mavlink_nic_info_t *)msgbuf;
    packet->imei = imei;
    packet->iccid = iccid;
    packet->imsi = imsi;
    packet->firmware_version = firmware_version;
    packet->id = id;
    packet->nic_id = nic_id;
    mav_array_memcpy(packet->nic_model, nic_model, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NIC_INFO, (const char *)packet, MAVLINK_MSG_ID_NIC_INFO_MIN_LEN, MAVLINK_MSG_ID_NIC_INFO_LEN, MAVLINK_MSG_ID_NIC_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE NIC_INFO UNPACKING


/**
 * @brief Get field id from nic_info message
 *
 * @return  NIC instance number.
 */
static inline uint8_t mavlink_msg_nic_info_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field nic_id from nic_info message
 *
 * @return  Unique id for NIC (Vlan).
 */
static inline uint8_t mavlink_msg_nic_info_get_nic_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field nic_model from nic_info message
 *
 * @return  NIC model.
 */
static inline uint16_t mavlink_msg_nic_info_get_nic_model(const mavlink_message_t* msg, char *nic_model)
{
    return _MAV_RETURN_char_array(msg, nic_model, 50,  30);
}

/**
 * @brief Get field imei from nic_info message
 *
 * @return  Unique NIC International Mobile Equipment Identity Number.
 */
static inline uint64_t mavlink_msg_nic_info_get_imei(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field iccid from nic_info message
 *
 * @return  Integrated Circuit Card Identification Number of SIM Card.
 */
static inline uint64_t mavlink_msg_nic_info_get_iccid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field imsi from nic_info message
 *
 * @return   Current SIM International mobile subscriber identity.
 */
static inline uint64_t mavlink_msg_nic_info_get_imsi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field firmware_version from nic_info message
 *
 * @return  The firmware version installed on the modem.
 */
static inline float mavlink_msg_nic_info_get_firmware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a nic_info message into a struct
 *
 * @param msg The message to decode
 * @param nic_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_nic_info_decode(const mavlink_message_t* msg, mavlink_nic_info_t* nic_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    nic_info->imei = mavlink_msg_nic_info_get_imei(msg);
    nic_info->iccid = mavlink_msg_nic_info_get_iccid(msg);
    nic_info->imsi = mavlink_msg_nic_info_get_imsi(msg);
    nic_info->firmware_version = mavlink_msg_nic_info_get_firmware_version(msg);
    nic_info->id = mavlink_msg_nic_info_get_id(msg);
    nic_info->nic_id = mavlink_msg_nic_info_get_nic_id(msg);
    mavlink_msg_nic_info_get_nic_model(msg, nic_info->nic_model);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NIC_INFO_LEN? msg->len : MAVLINK_MSG_ID_NIC_INFO_LEN;
        memset(nic_info, 0, MAVLINK_MSG_ID_NIC_INFO_LEN);
    memcpy(nic_info, _MAV_PAYLOAD(msg), len);
#endif
}
