#pragma once
// MESSAGE RANGING_BEACON PACKING

#define MAVLINK_MSG_ID_RANGING_BEACON 13801


typedef struct __mavlink_ranging_beacon_t {
 uint64_t time_usec; /*< [us] Timestamp (epoch UTC time synced with GNSS system time or time since system boot)*/
 uint32_t range; /*< [mm] Range measurement in millimeters*/
 int32_t lat; /*< [degE7] Beacon latitude (WGS84)*/
 int32_t lon; /*< [degE7] Beacon longitude (WGS84)*/
 float alt_msl; /*< [m] Beacon altitude (MSL, WGS84) */
 float alt_ellipsoid; /*< [m] Beacon altitude (ellipsoid, WGS84)*/
 uint32_t hacc_est; /*< [mm] 1-sigma horizontal accuracy estimate*/
 uint32_t vacc_est; /*< [mm] 1-sigma vertical accuracy estimate*/
 float range_accuracy; /*< [m] Estimated range measurement accuracy*/
 uint16_t carrier_freq; /*< [MHz] Ranging carrier frequency*/
 uint8_t beacon_id; /*<  ID of the ranging beacon/station*/
 int8_t rssi_node; /*< [dBm] RSSI at node/vehicle side*/
 int8_t rssi_beacon; /*< [dBm] RSSI at beacon/station side*/
 uint8_t battery_status; /*< [%] Beacon battery status (0-100%, 255 if unknown)*/
 uint8_t sequence; /*<  Measurement sequence number*/
} mavlink_ranging_beacon_t;

#define MAVLINK_MSG_ID_RANGING_BEACON_LEN 47
#define MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN 47
#define MAVLINK_MSG_ID_13801_LEN 47
#define MAVLINK_MSG_ID_13801_MIN_LEN 47

#define MAVLINK_MSG_ID_RANGING_BEACON_CRC 55
#define MAVLINK_MSG_ID_13801_CRC 55



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RANGING_BEACON { \
    13801, \
    "RANGING_BEACON", \
    15, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ranging_beacon_t, time_usec) }, \
         { "beacon_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_ranging_beacon_t, beacon_id) }, \
         { "range", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_ranging_beacon_t, range) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_ranging_beacon_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ranging_beacon_t, lon) }, \
         { "alt_msl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ranging_beacon_t, alt_msl) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ranging_beacon_t, alt_ellipsoid) }, \
         { "hacc_est", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_ranging_beacon_t, hacc_est) }, \
         { "vacc_est", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_ranging_beacon_t, vacc_est) }, \
         { "rssi_node", NULL, MAVLINK_TYPE_INT8_T, 0, 43, offsetof(mavlink_ranging_beacon_t, rssi_node) }, \
         { "rssi_beacon", NULL, MAVLINK_TYPE_INT8_T, 0, 44, offsetof(mavlink_ranging_beacon_t, rssi_beacon) }, \
         { "battery_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_ranging_beacon_t, battery_status) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_ranging_beacon_t, sequence) }, \
         { "carrier_freq", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_ranging_beacon_t, carrier_freq) }, \
         { "range_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ranging_beacon_t, range_accuracy) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RANGING_BEACON { \
    "RANGING_BEACON", \
    15, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ranging_beacon_t, time_usec) }, \
         { "beacon_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_ranging_beacon_t, beacon_id) }, \
         { "range", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_ranging_beacon_t, range) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_ranging_beacon_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ranging_beacon_t, lon) }, \
         { "alt_msl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ranging_beacon_t, alt_msl) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ranging_beacon_t, alt_ellipsoid) }, \
         { "hacc_est", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_ranging_beacon_t, hacc_est) }, \
         { "vacc_est", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_ranging_beacon_t, vacc_est) }, \
         { "rssi_node", NULL, MAVLINK_TYPE_INT8_T, 0, 43, offsetof(mavlink_ranging_beacon_t, rssi_node) }, \
         { "rssi_beacon", NULL, MAVLINK_TYPE_INT8_T, 0, 44, offsetof(mavlink_ranging_beacon_t, rssi_beacon) }, \
         { "battery_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_ranging_beacon_t, battery_status) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_ranging_beacon_t, sequence) }, \
         { "carrier_freq", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_ranging_beacon_t, carrier_freq) }, \
         { "range_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ranging_beacon_t, range_accuracy) }, \
         } \
}
#endif

/**
 * @brief Pack a ranging_beacon message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (epoch UTC time synced with GNSS system time or time since system boot)
 * @param beacon_id  ID of the ranging beacon/station
 * @param range [mm] Range measurement in millimeters
 * @param lat [degE7] Beacon latitude (WGS84)
 * @param lon [degE7] Beacon longitude (WGS84)
 * @param alt_msl [m] Beacon altitude (MSL, WGS84) 
 * @param alt_ellipsoid [m] Beacon altitude (ellipsoid, WGS84)
 * @param hacc_est [mm] 1-sigma horizontal accuracy estimate
 * @param vacc_est [mm] 1-sigma vertical accuracy estimate
 * @param rssi_node [dBm] RSSI at node/vehicle side
 * @param rssi_beacon [dBm] RSSI at beacon/station side
 * @param battery_status [%] Beacon battery status (0-100%, 255 if unknown)
 * @param sequence  Measurement sequence number
 * @param carrier_freq [MHz] Ranging carrier frequency
 * @param range_accuracy [m] Estimated range measurement accuracy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ranging_beacon_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t beacon_id, uint32_t range, int32_t lat, int32_t lon, float alt_msl, float alt_ellipsoid, uint32_t hacc_est, uint32_t vacc_est, int8_t rssi_node, int8_t rssi_beacon, uint8_t battery_status, uint8_t sequence, uint16_t carrier_freq, float range_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGING_BEACON_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, range);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_msl);
    _mav_put_float(buf, 24, alt_ellipsoid);
    _mav_put_uint32_t(buf, 28, hacc_est);
    _mav_put_uint32_t(buf, 32, vacc_est);
    _mav_put_float(buf, 36, range_accuracy);
    _mav_put_uint16_t(buf, 40, carrier_freq);
    _mav_put_uint8_t(buf, 42, beacon_id);
    _mav_put_int8_t(buf, 43, rssi_node);
    _mav_put_int8_t(buf, 44, rssi_beacon);
    _mav_put_uint8_t(buf, 45, battery_status);
    _mav_put_uint8_t(buf, 46, sequence);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
#else
    mavlink_ranging_beacon_t packet;
    packet.time_usec = time_usec;
    packet.range = range;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.hacc_est = hacc_est;
    packet.vacc_est = vacc_est;
    packet.range_accuracy = range_accuracy;
    packet.carrier_freq = carrier_freq;
    packet.beacon_id = beacon_id;
    packet.rssi_node = rssi_node;
    packet.rssi_beacon = rssi_beacon;
    packet.battery_status = battery_status;
    packet.sequence = sequence;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGING_BEACON;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
}

/**
 * @brief Pack a ranging_beacon message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (epoch UTC time synced with GNSS system time or time since system boot)
 * @param beacon_id  ID of the ranging beacon/station
 * @param range [mm] Range measurement in millimeters
 * @param lat [degE7] Beacon latitude (WGS84)
 * @param lon [degE7] Beacon longitude (WGS84)
 * @param alt_msl [m] Beacon altitude (MSL, WGS84) 
 * @param alt_ellipsoid [m] Beacon altitude (ellipsoid, WGS84)
 * @param hacc_est [mm] 1-sigma horizontal accuracy estimate
 * @param vacc_est [mm] 1-sigma vertical accuracy estimate
 * @param rssi_node [dBm] RSSI at node/vehicle side
 * @param rssi_beacon [dBm] RSSI at beacon/station side
 * @param battery_status [%] Beacon battery status (0-100%, 255 if unknown)
 * @param sequence  Measurement sequence number
 * @param carrier_freq [MHz] Ranging carrier frequency
 * @param range_accuracy [m] Estimated range measurement accuracy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ranging_beacon_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t beacon_id, uint32_t range, int32_t lat, int32_t lon, float alt_msl, float alt_ellipsoid, uint32_t hacc_est, uint32_t vacc_est, int8_t rssi_node, int8_t rssi_beacon, uint8_t battery_status, uint8_t sequence, uint16_t carrier_freq, float range_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGING_BEACON_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, range);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_msl);
    _mav_put_float(buf, 24, alt_ellipsoid);
    _mav_put_uint32_t(buf, 28, hacc_est);
    _mav_put_uint32_t(buf, 32, vacc_est);
    _mav_put_float(buf, 36, range_accuracy);
    _mav_put_uint16_t(buf, 40, carrier_freq);
    _mav_put_uint8_t(buf, 42, beacon_id);
    _mav_put_int8_t(buf, 43, rssi_node);
    _mav_put_int8_t(buf, 44, rssi_beacon);
    _mav_put_uint8_t(buf, 45, battery_status);
    _mav_put_uint8_t(buf, 46, sequence);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
#else
    mavlink_ranging_beacon_t packet;
    packet.time_usec = time_usec;
    packet.range = range;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.hacc_est = hacc_est;
    packet.vacc_est = vacc_est;
    packet.range_accuracy = range_accuracy;
    packet.carrier_freq = carrier_freq;
    packet.beacon_id = beacon_id;
    packet.rssi_node = rssi_node;
    packet.rssi_beacon = rssi_beacon;
    packet.battery_status = battery_status;
    packet.sequence = sequence;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGING_BEACON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
#endif
}

/**
 * @brief Pack a ranging_beacon message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (epoch UTC time synced with GNSS system time or time since system boot)
 * @param beacon_id  ID of the ranging beacon/station
 * @param range [mm] Range measurement in millimeters
 * @param lat [degE7] Beacon latitude (WGS84)
 * @param lon [degE7] Beacon longitude (WGS84)
 * @param alt_msl [m] Beacon altitude (MSL, WGS84) 
 * @param alt_ellipsoid [m] Beacon altitude (ellipsoid, WGS84)
 * @param hacc_est [mm] 1-sigma horizontal accuracy estimate
 * @param vacc_est [mm] 1-sigma vertical accuracy estimate
 * @param rssi_node [dBm] RSSI at node/vehicle side
 * @param rssi_beacon [dBm] RSSI at beacon/station side
 * @param battery_status [%] Beacon battery status (0-100%, 255 if unknown)
 * @param sequence  Measurement sequence number
 * @param carrier_freq [MHz] Ranging carrier frequency
 * @param range_accuracy [m] Estimated range measurement accuracy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ranging_beacon_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t beacon_id,uint32_t range,int32_t lat,int32_t lon,float alt_msl,float alt_ellipsoid,uint32_t hacc_est,uint32_t vacc_est,int8_t rssi_node,int8_t rssi_beacon,uint8_t battery_status,uint8_t sequence,uint16_t carrier_freq,float range_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGING_BEACON_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, range);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_msl);
    _mav_put_float(buf, 24, alt_ellipsoid);
    _mav_put_uint32_t(buf, 28, hacc_est);
    _mav_put_uint32_t(buf, 32, vacc_est);
    _mav_put_float(buf, 36, range_accuracy);
    _mav_put_uint16_t(buf, 40, carrier_freq);
    _mav_put_uint8_t(buf, 42, beacon_id);
    _mav_put_int8_t(buf, 43, rssi_node);
    _mav_put_int8_t(buf, 44, rssi_beacon);
    _mav_put_uint8_t(buf, 45, battery_status);
    _mav_put_uint8_t(buf, 46, sequence);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
#else
    mavlink_ranging_beacon_t packet;
    packet.time_usec = time_usec;
    packet.range = range;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.hacc_est = hacc_est;
    packet.vacc_est = vacc_est;
    packet.range_accuracy = range_accuracy;
    packet.carrier_freq = carrier_freq;
    packet.beacon_id = beacon_id;
    packet.rssi_node = rssi_node;
    packet.rssi_beacon = rssi_beacon;
    packet.battery_status = battery_status;
    packet.sequence = sequence;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGING_BEACON;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
}

/**
 * @brief Encode a ranging_beacon struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ranging_beacon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ranging_beacon_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ranging_beacon_t* ranging_beacon)
{
    return mavlink_msg_ranging_beacon_pack(system_id, component_id, msg, ranging_beacon->time_usec, ranging_beacon->beacon_id, ranging_beacon->range, ranging_beacon->lat, ranging_beacon->lon, ranging_beacon->alt_msl, ranging_beacon->alt_ellipsoid, ranging_beacon->hacc_est, ranging_beacon->vacc_est, ranging_beacon->rssi_node, ranging_beacon->rssi_beacon, ranging_beacon->battery_status, ranging_beacon->sequence, ranging_beacon->carrier_freq, ranging_beacon->range_accuracy);
}

/**
 * @brief Encode a ranging_beacon struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ranging_beacon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ranging_beacon_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ranging_beacon_t* ranging_beacon)
{
    return mavlink_msg_ranging_beacon_pack_chan(system_id, component_id, chan, msg, ranging_beacon->time_usec, ranging_beacon->beacon_id, ranging_beacon->range, ranging_beacon->lat, ranging_beacon->lon, ranging_beacon->alt_msl, ranging_beacon->alt_ellipsoid, ranging_beacon->hacc_est, ranging_beacon->vacc_est, ranging_beacon->rssi_node, ranging_beacon->rssi_beacon, ranging_beacon->battery_status, ranging_beacon->sequence, ranging_beacon->carrier_freq, ranging_beacon->range_accuracy);
}

/**
 * @brief Encode a ranging_beacon struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ranging_beacon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ranging_beacon_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ranging_beacon_t* ranging_beacon)
{
    return mavlink_msg_ranging_beacon_pack_status(system_id, component_id, _status, msg,  ranging_beacon->time_usec, ranging_beacon->beacon_id, ranging_beacon->range, ranging_beacon->lat, ranging_beacon->lon, ranging_beacon->alt_msl, ranging_beacon->alt_ellipsoid, ranging_beacon->hacc_est, ranging_beacon->vacc_est, ranging_beacon->rssi_node, ranging_beacon->rssi_beacon, ranging_beacon->battery_status, ranging_beacon->sequence, ranging_beacon->carrier_freq, ranging_beacon->range_accuracy);
}

/**
 * @brief Send a ranging_beacon message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (epoch UTC time synced with GNSS system time or time since system boot)
 * @param beacon_id  ID of the ranging beacon/station
 * @param range [mm] Range measurement in millimeters
 * @param lat [degE7] Beacon latitude (WGS84)
 * @param lon [degE7] Beacon longitude (WGS84)
 * @param alt_msl [m] Beacon altitude (MSL, WGS84) 
 * @param alt_ellipsoid [m] Beacon altitude (ellipsoid, WGS84)
 * @param hacc_est [mm] 1-sigma horizontal accuracy estimate
 * @param vacc_est [mm] 1-sigma vertical accuracy estimate
 * @param rssi_node [dBm] RSSI at node/vehicle side
 * @param rssi_beacon [dBm] RSSI at beacon/station side
 * @param battery_status [%] Beacon battery status (0-100%, 255 if unknown)
 * @param sequence  Measurement sequence number
 * @param carrier_freq [MHz] Ranging carrier frequency
 * @param range_accuracy [m] Estimated range measurement accuracy
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ranging_beacon_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t beacon_id, uint32_t range, int32_t lat, int32_t lon, float alt_msl, float alt_ellipsoid, uint32_t hacc_est, uint32_t vacc_est, int8_t rssi_node, int8_t rssi_beacon, uint8_t battery_status, uint8_t sequence, uint16_t carrier_freq, float range_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGING_BEACON_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, range);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_msl);
    _mav_put_float(buf, 24, alt_ellipsoid);
    _mav_put_uint32_t(buf, 28, hacc_est);
    _mav_put_uint32_t(buf, 32, vacc_est);
    _mav_put_float(buf, 36, range_accuracy);
    _mav_put_uint16_t(buf, 40, carrier_freq);
    _mav_put_uint8_t(buf, 42, beacon_id);
    _mav_put_int8_t(buf, 43, rssi_node);
    _mav_put_int8_t(buf, 44, rssi_beacon);
    _mav_put_uint8_t(buf, 45, battery_status);
    _mav_put_uint8_t(buf, 46, sequence);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGING_BEACON, buf, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
#else
    mavlink_ranging_beacon_t packet;
    packet.time_usec = time_usec;
    packet.range = range;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.hacc_est = hacc_est;
    packet.vacc_est = vacc_est;
    packet.range_accuracy = range_accuracy;
    packet.carrier_freq = carrier_freq;
    packet.beacon_id = beacon_id;
    packet.rssi_node = rssi_node;
    packet.rssi_beacon = rssi_beacon;
    packet.battery_status = battery_status;
    packet.sequence = sequence;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGING_BEACON, (const char *)&packet, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
#endif
}

/**
 * @brief Send a ranging_beacon message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ranging_beacon_send_struct(mavlink_channel_t chan, const mavlink_ranging_beacon_t* ranging_beacon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ranging_beacon_send(chan, ranging_beacon->time_usec, ranging_beacon->beacon_id, ranging_beacon->range, ranging_beacon->lat, ranging_beacon->lon, ranging_beacon->alt_msl, ranging_beacon->alt_ellipsoid, ranging_beacon->hacc_est, ranging_beacon->vacc_est, ranging_beacon->rssi_node, ranging_beacon->rssi_beacon, ranging_beacon->battery_status, ranging_beacon->sequence, ranging_beacon->carrier_freq, ranging_beacon->range_accuracy);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGING_BEACON, (const char *)ranging_beacon, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
#endif
}

#if MAVLINK_MSG_ID_RANGING_BEACON_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ranging_beacon_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t beacon_id, uint32_t range, int32_t lat, int32_t lon, float alt_msl, float alt_ellipsoid, uint32_t hacc_est, uint32_t vacc_est, int8_t rssi_node, int8_t rssi_beacon, uint8_t battery_status, uint8_t sequence, uint16_t carrier_freq, float range_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, range);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt_msl);
    _mav_put_float(buf, 24, alt_ellipsoid);
    _mav_put_uint32_t(buf, 28, hacc_est);
    _mav_put_uint32_t(buf, 32, vacc_est);
    _mav_put_float(buf, 36, range_accuracy);
    _mav_put_uint16_t(buf, 40, carrier_freq);
    _mav_put_uint8_t(buf, 42, beacon_id);
    _mav_put_int8_t(buf, 43, rssi_node);
    _mav_put_int8_t(buf, 44, rssi_beacon);
    _mav_put_uint8_t(buf, 45, battery_status);
    _mav_put_uint8_t(buf, 46, sequence);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGING_BEACON, buf, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
#else
    mavlink_ranging_beacon_t *packet = (mavlink_ranging_beacon_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->range = range;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt_msl = alt_msl;
    packet->alt_ellipsoid = alt_ellipsoid;
    packet->hacc_est = hacc_est;
    packet->vacc_est = vacc_est;
    packet->range_accuracy = range_accuracy;
    packet->carrier_freq = carrier_freq;
    packet->beacon_id = beacon_id;
    packet->rssi_node = rssi_node;
    packet->rssi_beacon = rssi_beacon;
    packet->battery_status = battery_status;
    packet->sequence = sequence;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGING_BEACON, (const char *)packet, MAVLINK_MSG_ID_RANGING_BEACON_MIN_LEN, MAVLINK_MSG_ID_RANGING_BEACON_LEN, MAVLINK_MSG_ID_RANGING_BEACON_CRC);
#endif
}
#endif

#endif

// MESSAGE RANGING_BEACON UNPACKING


/**
 * @brief Get field time_usec from ranging_beacon message
 *
 * @return [us] Timestamp (epoch UTC time synced with GNSS system time or time since system boot)
 */
static inline uint64_t mavlink_msg_ranging_beacon_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field beacon_id from ranging_beacon message
 *
 * @return  ID of the ranging beacon/station
 */
static inline uint8_t mavlink_msg_ranging_beacon_get_beacon_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field range from ranging_beacon message
 *
 * @return [mm] Range measurement in millimeters
 */
static inline uint32_t mavlink_msg_ranging_beacon_get_range(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field lat from ranging_beacon message
 *
 * @return [degE7] Beacon latitude (WGS84)
 */
static inline int32_t mavlink_msg_ranging_beacon_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field lon from ranging_beacon message
 *
 * @return [degE7] Beacon longitude (WGS84)
 */
static inline int32_t mavlink_msg_ranging_beacon_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt_msl from ranging_beacon message
 *
 * @return [m] Beacon altitude (MSL, WGS84) 
 */
static inline float mavlink_msg_ranging_beacon_get_alt_msl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field alt_ellipsoid from ranging_beacon message
 *
 * @return [m] Beacon altitude (ellipsoid, WGS84)
 */
static inline float mavlink_msg_ranging_beacon_get_alt_ellipsoid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field hacc_est from ranging_beacon message
 *
 * @return [mm] 1-sigma horizontal accuracy estimate
 */
static inline uint32_t mavlink_msg_ranging_beacon_get_hacc_est(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field vacc_est from ranging_beacon message
 *
 * @return [mm] 1-sigma vertical accuracy estimate
 */
static inline uint32_t mavlink_msg_ranging_beacon_get_vacc_est(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field rssi_node from ranging_beacon message
 *
 * @return [dBm] RSSI at node/vehicle side
 */
static inline int8_t mavlink_msg_ranging_beacon_get_rssi_node(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  43);
}

/**
 * @brief Get field rssi_beacon from ranging_beacon message
 *
 * @return [dBm] RSSI at beacon/station side
 */
static inline int8_t mavlink_msg_ranging_beacon_get_rssi_beacon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  44);
}

/**
 * @brief Get field battery_status from ranging_beacon message
 *
 * @return [%] Beacon battery status (0-100%, 255 if unknown)
 */
static inline uint8_t mavlink_msg_ranging_beacon_get_battery_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field sequence from ranging_beacon message
 *
 * @return  Measurement sequence number
 */
static inline uint8_t mavlink_msg_ranging_beacon_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field carrier_freq from ranging_beacon message
 *
 * @return [MHz] Ranging carrier frequency
 */
static inline uint16_t mavlink_msg_ranging_beacon_get_carrier_freq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field range_accuracy from ranging_beacon message
 *
 * @return [m] Estimated range measurement accuracy
 */
static inline float mavlink_msg_ranging_beacon_get_range_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a ranging_beacon message into a struct
 *
 * @param msg The message to decode
 * @param ranging_beacon C-struct to decode the message contents into
 */
static inline void mavlink_msg_ranging_beacon_decode(const mavlink_message_t* msg, mavlink_ranging_beacon_t* ranging_beacon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ranging_beacon->time_usec = mavlink_msg_ranging_beacon_get_time_usec(msg);
    ranging_beacon->range = mavlink_msg_ranging_beacon_get_range(msg);
    ranging_beacon->lat = mavlink_msg_ranging_beacon_get_lat(msg);
    ranging_beacon->lon = mavlink_msg_ranging_beacon_get_lon(msg);
    ranging_beacon->alt_msl = mavlink_msg_ranging_beacon_get_alt_msl(msg);
    ranging_beacon->alt_ellipsoid = mavlink_msg_ranging_beacon_get_alt_ellipsoid(msg);
    ranging_beacon->hacc_est = mavlink_msg_ranging_beacon_get_hacc_est(msg);
    ranging_beacon->vacc_est = mavlink_msg_ranging_beacon_get_vacc_est(msg);
    ranging_beacon->range_accuracy = mavlink_msg_ranging_beacon_get_range_accuracy(msg);
    ranging_beacon->carrier_freq = mavlink_msg_ranging_beacon_get_carrier_freq(msg);
    ranging_beacon->beacon_id = mavlink_msg_ranging_beacon_get_beacon_id(msg);
    ranging_beacon->rssi_node = mavlink_msg_ranging_beacon_get_rssi_node(msg);
    ranging_beacon->rssi_beacon = mavlink_msg_ranging_beacon_get_rssi_beacon(msg);
    ranging_beacon->battery_status = mavlink_msg_ranging_beacon_get_battery_status(msg);
    ranging_beacon->sequence = mavlink_msg_ranging_beacon_get_sequence(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RANGING_BEACON_LEN? msg->len : MAVLINK_MSG_ID_RANGING_BEACON_LEN;
        memset(ranging_beacon, 0, MAVLINK_MSG_ID_RANGING_BEACON_LEN);
    memcpy(ranging_beacon, _MAV_PAYLOAD(msg), len);
#endif
}
