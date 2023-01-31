#pragma once
// MESSAGE CELLULAR_STATUS PACKING

#define MAVLINK_MSG_ID_CELLULAR_STATUS 334

MAVPACKED(
typedef struct __mavlink_cellular_status_t {
 uint16_t mcc; /*<  Mobile country code. If unknown, set to UINT16_MAX*/
 uint16_t mnc; /*<  Mobile network code. If unknown, set to UINT16_MAX*/
 uint16_t lac; /*<  Location area code. If unknown, set to 0*/
 uint8_t status; /*<  Cellular modem status*/
 uint8_t failure_reason; /*<  Failure reason when status in in CELLUAR_STATUS_FAILED*/
 uint8_t type; /*<  Cellular network radio type: gsm, cdma, lte...*/
 uint8_t quality; /*<  Signal quality in percent. If unknown, set to UINT8_MAX*/
 uint8_t id; /*<  Cellular instance number*/
 uint64_t download_rate; /*<  download rate in kbits/s*/
 uint64_t upload_rate; /*<  upload rate in kbits/s*/
 uint64_t ber; /*<  bit rate error measurement*/
 float rx_level; /*<  rx level.*/
 float tx_level; /*<  rx level.*/
 float signal_to_noise; /*<  signal to noise.*/
 char cell_tower_id[9]; /*<  signal to noise.*/
 uint8_t band_number; /*<  band number.*/
 float band_frequency; /*<  band number. */
 float arfcn; /*<  Absolute radio-frequency channel number.*/
}) mavlink_cellular_status_t;

#define MAVLINK_MSG_ID_CELLULAR_STATUS_LEN 65
#define MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN 10
#define MAVLINK_MSG_ID_334_LEN 65
#define MAVLINK_MSG_ID_334_MIN_LEN 10

#define MAVLINK_MSG_ID_CELLULAR_STATUS_CRC 72
#define MAVLINK_MSG_ID_334_CRC 72

#define MAVLINK_MSG_CELLULAR_STATUS_FIELD_CELL_TOWER_ID_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CELLULAR_STATUS { \
    334, \
    "CELLULAR_STATUS", \
    18, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_cellular_status_t, status) }, \
         { "failure_reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_cellular_status_t, failure_reason) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cellular_status_t, type) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cellular_status_t, quality) }, \
         { "mcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cellular_status_t, mcc) }, \
         { "mnc", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_cellular_status_t, mnc) }, \
         { "lac", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_cellular_status_t, lac) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_cellular_status_t, id) }, \
         { "download_rate", NULL, MAVLINK_TYPE_UINT64_T, 0, 11, offsetof(mavlink_cellular_status_t, download_rate) }, \
         { "upload_rate", NULL, MAVLINK_TYPE_UINT64_T, 0, 19, offsetof(mavlink_cellular_status_t, upload_rate) }, \
         { "ber", NULL, MAVLINK_TYPE_UINT64_T, 0, 27, offsetof(mavlink_cellular_status_t, ber) }, \
         { "rx_level", NULL, MAVLINK_TYPE_FLOAT, 0, 35, offsetof(mavlink_cellular_status_t, rx_level) }, \
         { "tx_level", NULL, MAVLINK_TYPE_FLOAT, 0, 39, offsetof(mavlink_cellular_status_t, tx_level) }, \
         { "signal_to_noise", NULL, MAVLINK_TYPE_FLOAT, 0, 43, offsetof(mavlink_cellular_status_t, signal_to_noise) }, \
         { "cell_tower_id", NULL, MAVLINK_TYPE_CHAR, 9, 47, offsetof(mavlink_cellular_status_t, cell_tower_id) }, \
         { "band_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_cellular_status_t, band_number) }, \
         { "band_frequency", NULL, MAVLINK_TYPE_FLOAT, 0, 57, offsetof(mavlink_cellular_status_t, band_frequency) }, \
         { "arfcn", NULL, MAVLINK_TYPE_FLOAT, 0, 61, offsetof(mavlink_cellular_status_t, arfcn) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CELLULAR_STATUS { \
    "CELLULAR_STATUS", \
    18, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_cellular_status_t, status) }, \
         { "failure_reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_cellular_status_t, failure_reason) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cellular_status_t, type) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cellular_status_t, quality) }, \
         { "mcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cellular_status_t, mcc) }, \
         { "mnc", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_cellular_status_t, mnc) }, \
         { "lac", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_cellular_status_t, lac) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_cellular_status_t, id) }, \
         { "download_rate", NULL, MAVLINK_TYPE_UINT64_T, 0, 11, offsetof(mavlink_cellular_status_t, download_rate) }, \
         { "upload_rate", NULL, MAVLINK_TYPE_UINT64_T, 0, 19, offsetof(mavlink_cellular_status_t, upload_rate) }, \
         { "ber", NULL, MAVLINK_TYPE_UINT64_T, 0, 27, offsetof(mavlink_cellular_status_t, ber) }, \
         { "rx_level", NULL, MAVLINK_TYPE_FLOAT, 0, 35, offsetof(mavlink_cellular_status_t, rx_level) }, \
         { "tx_level", NULL, MAVLINK_TYPE_FLOAT, 0, 39, offsetof(mavlink_cellular_status_t, tx_level) }, \
         { "signal_to_noise", NULL, MAVLINK_TYPE_FLOAT, 0, 43, offsetof(mavlink_cellular_status_t, signal_to_noise) }, \
         { "cell_tower_id", NULL, MAVLINK_TYPE_CHAR, 9, 47, offsetof(mavlink_cellular_status_t, cell_tower_id) }, \
         { "band_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_cellular_status_t, band_number) }, \
         { "band_frequency", NULL, MAVLINK_TYPE_FLOAT, 0, 57, offsetof(mavlink_cellular_status_t, band_frequency) }, \
         { "arfcn", NULL, MAVLINK_TYPE_FLOAT, 0, 61, offsetof(mavlink_cellular_status_t, arfcn) }, \
         } \
}
#endif

/**
 * @brief Pack a cellular_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Cellular modem status
 * @param failure_reason  Failure reason when status in in CELLUAR_STATUS_FAILED
 * @param type  Cellular network radio type: gsm, cdma, lte...
 * @param quality  Signal quality in percent. If unknown, set to UINT8_MAX
 * @param mcc  Mobile country code. If unknown, set to UINT16_MAX
 * @param mnc  Mobile network code. If unknown, set to UINT16_MAX
 * @param lac  Location area code. If unknown, set to 0
 * @param id  Cellular instance number
 * @param download_rate  download rate in kbits/s
 * @param upload_rate  upload rate in kbits/s
 * @param ber  bit rate error measurement
 * @param rx_level  rx level.
 * @param tx_level  rx level.
 * @param signal_to_noise  signal to noise.
 * @param cell_tower_id  signal to noise.
 * @param band_number  band number.
 * @param band_frequency  band number. 
 * @param arfcn  Absolute radio-frequency channel number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac, uint8_t id, uint64_t download_rate, uint64_t upload_rate, uint64_t ber, float rx_level, float tx_level, float signal_to_noise, const char *cell_tower_id, uint8_t band_number, float band_frequency, float arfcn)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint64_t(buf, 11, download_rate);
    _mav_put_uint64_t(buf, 19, upload_rate);
    _mav_put_uint64_t(buf, 27, ber);
    _mav_put_float(buf, 35, rx_level);
    _mav_put_float(buf, 39, tx_level);
    _mav_put_float(buf, 43, signal_to_noise);
    _mav_put_uint8_t(buf, 56, band_number);
    _mav_put_float(buf, 57, band_frequency);
    _mav_put_float(buf, 61, arfcn);
    _mav_put_char_array(buf, 47, cell_tower_id, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#else
    mavlink_cellular_status_t packet;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.lac = lac;
    packet.status = status;
    packet.failure_reason = failure_reason;
    packet.type = type;
    packet.quality = quality;
    packet.id = id;
    packet.download_rate = download_rate;
    packet.upload_rate = upload_rate;
    packet.ber = ber;
    packet.rx_level = rx_level;
    packet.tx_level = tx_level;
    packet.signal_to_noise = signal_to_noise;
    packet.band_number = band_number;
    packet.band_frequency = band_frequency;
    packet.arfcn = arfcn;
    mav_array_memcpy(packet.cell_tower_id, cell_tower_id, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
}

/**
 * @brief Pack a cellular_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  Cellular modem status
 * @param failure_reason  Failure reason when status in in CELLUAR_STATUS_FAILED
 * @param type  Cellular network radio type: gsm, cdma, lte...
 * @param quality  Signal quality in percent. If unknown, set to UINT8_MAX
 * @param mcc  Mobile country code. If unknown, set to UINT16_MAX
 * @param mnc  Mobile network code. If unknown, set to UINT16_MAX
 * @param lac  Location area code. If unknown, set to 0
 * @param id  Cellular instance number
 * @param download_rate  download rate in kbits/s
 * @param upload_rate  upload rate in kbits/s
 * @param ber  bit rate error measurement
 * @param rx_level  rx level.
 * @param tx_level  rx level.
 * @param signal_to_noise  signal to noise.
 * @param cell_tower_id  signal to noise.
 * @param band_number  band number.
 * @param band_frequency  band number. 
 * @param arfcn  Absolute radio-frequency channel number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cellular_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status,uint8_t failure_reason,uint8_t type,uint8_t quality,uint16_t mcc,uint16_t mnc,uint16_t lac,uint8_t id,uint64_t download_rate,uint64_t upload_rate,uint64_t ber,float rx_level,float tx_level,float signal_to_noise,const char *cell_tower_id,uint8_t band_number,float band_frequency,float arfcn)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint64_t(buf, 11, download_rate);
    _mav_put_uint64_t(buf, 19, upload_rate);
    _mav_put_uint64_t(buf, 27, ber);
    _mav_put_float(buf, 35, rx_level);
    _mav_put_float(buf, 39, tx_level);
    _mav_put_float(buf, 43, signal_to_noise);
    _mav_put_uint8_t(buf, 56, band_number);
    _mav_put_float(buf, 57, band_frequency);
    _mav_put_float(buf, 61, arfcn);
    _mav_put_char_array(buf, 47, cell_tower_id, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#else
    mavlink_cellular_status_t packet;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.lac = lac;
    packet.status = status;
    packet.failure_reason = failure_reason;
    packet.type = type;
    packet.quality = quality;
    packet.id = id;
    packet.download_rate = download_rate;
    packet.upload_rate = upload_rate;
    packet.ber = ber;
    packet.rx_level = rx_level;
    packet.tx_level = tx_level;
    packet.signal_to_noise = signal_to_noise;
    packet.band_number = band_number;
    packet.band_frequency = band_frequency;
    packet.arfcn = arfcn;
    mav_array_memcpy(packet.cell_tower_id, cell_tower_id, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CELLULAR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
}

/**
 * @brief Encode a cellular_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cellular_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cellular_status_t* cellular_status)
{
    return mavlink_msg_cellular_status_pack(system_id, component_id, msg, cellular_status->status, cellular_status->failure_reason, cellular_status->type, cellular_status->quality, cellular_status->mcc, cellular_status->mnc, cellular_status->lac, cellular_status->id, cellular_status->download_rate, cellular_status->upload_rate, cellular_status->ber, cellular_status->rx_level, cellular_status->tx_level, cellular_status->signal_to_noise, cellular_status->cell_tower_id, cellular_status->band_number, cellular_status->band_frequency, cellular_status->arfcn);
}

/**
 * @brief Encode a cellular_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cellular_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cellular_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cellular_status_t* cellular_status)
{
    return mavlink_msg_cellular_status_pack_chan(system_id, component_id, chan, msg, cellular_status->status, cellular_status->failure_reason, cellular_status->type, cellular_status->quality, cellular_status->mcc, cellular_status->mnc, cellular_status->lac, cellular_status->id, cellular_status->download_rate, cellular_status->upload_rate, cellular_status->ber, cellular_status->rx_level, cellular_status->tx_level, cellular_status->signal_to_noise, cellular_status->cell_tower_id, cellular_status->band_number, cellular_status->band_frequency, cellular_status->arfcn);
}

/**
 * @brief Send a cellular_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status  Cellular modem status
 * @param failure_reason  Failure reason when status in in CELLUAR_STATUS_FAILED
 * @param type  Cellular network radio type: gsm, cdma, lte...
 * @param quality  Signal quality in percent. If unknown, set to UINT8_MAX
 * @param mcc  Mobile country code. If unknown, set to UINT16_MAX
 * @param mnc  Mobile network code. If unknown, set to UINT16_MAX
 * @param lac  Location area code. If unknown, set to 0
 * @param id  Cellular instance number
 * @param download_rate  download rate in kbits/s
 * @param upload_rate  upload rate in kbits/s
 * @param ber  bit rate error measurement
 * @param rx_level  rx level.
 * @param tx_level  rx level.
 * @param signal_to_noise  signal to noise.
 * @param cell_tower_id  signal to noise.
 * @param band_number  band number.
 * @param band_frequency  band number. 
 * @param arfcn  Absolute radio-frequency channel number.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cellular_status_send(mavlink_channel_t chan, uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac, uint8_t id, uint64_t download_rate, uint64_t upload_rate, uint64_t ber, float rx_level, float tx_level, float signal_to_noise, const char *cell_tower_id, uint8_t band_number, float band_frequency, float arfcn)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CELLULAR_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint64_t(buf, 11, download_rate);
    _mav_put_uint64_t(buf, 19, upload_rate);
    _mav_put_uint64_t(buf, 27, ber);
    _mav_put_float(buf, 35, rx_level);
    _mav_put_float(buf, 39, tx_level);
    _mav_put_float(buf, 43, signal_to_noise);
    _mav_put_uint8_t(buf, 56, band_number);
    _mav_put_float(buf, 57, band_frequency);
    _mav_put_float(buf, 61, arfcn);
    _mav_put_char_array(buf, 47, cell_tower_id, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, buf, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#else
    mavlink_cellular_status_t packet;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.lac = lac;
    packet.status = status;
    packet.failure_reason = failure_reason;
    packet.type = type;
    packet.quality = quality;
    packet.id = id;
    packet.download_rate = download_rate;
    packet.upload_rate = upload_rate;
    packet.ber = ber;
    packet.rx_level = rx_level;
    packet.tx_level = tx_level;
    packet.signal_to_noise = signal_to_noise;
    packet.band_number = band_number;
    packet.band_frequency = band_frequency;
    packet.arfcn = arfcn;
    mav_array_memcpy(packet.cell_tower_id, cell_tower_id, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#endif
}

/**
 * @brief Send a cellular_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cellular_status_send_struct(mavlink_channel_t chan, const mavlink_cellular_status_t* cellular_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cellular_status_send(chan, cellular_status->status, cellular_status->failure_reason, cellular_status->type, cellular_status->quality, cellular_status->mcc, cellular_status->mnc, cellular_status->lac, cellular_status->id, cellular_status->download_rate, cellular_status->upload_rate, cellular_status->ber, cellular_status->rx_level, cellular_status->tx_level, cellular_status->signal_to_noise, cellular_status->cell_tower_id, cellular_status->band_number, cellular_status->band_frequency, cellular_status->arfcn);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, (const char *)cellular_status, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CELLULAR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cellular_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac, uint8_t id, uint64_t download_rate, uint64_t upload_rate, uint64_t ber, float rx_level, float tx_level, float signal_to_noise, const char *cell_tower_id, uint8_t band_number, float band_frequency, float arfcn)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, mcc);
    _mav_put_uint16_t(buf, 2, mnc);
    _mav_put_uint16_t(buf, 4, lac);
    _mav_put_uint8_t(buf, 6, status);
    _mav_put_uint8_t(buf, 7, failure_reason);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, quality);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint64_t(buf, 11, download_rate);
    _mav_put_uint64_t(buf, 19, upload_rate);
    _mav_put_uint64_t(buf, 27, ber);
    _mav_put_float(buf, 35, rx_level);
    _mav_put_float(buf, 39, tx_level);
    _mav_put_float(buf, 43, signal_to_noise);
    _mav_put_uint8_t(buf, 56, band_number);
    _mav_put_float(buf, 57, band_frequency);
    _mav_put_float(buf, 61, arfcn);
    _mav_put_char_array(buf, 47, cell_tower_id, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, buf, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#else
    mavlink_cellular_status_t *packet = (mavlink_cellular_status_t *)msgbuf;
    packet->mcc = mcc;
    packet->mnc = mnc;
    packet->lac = lac;
    packet->status = status;
    packet->failure_reason = failure_reason;
    packet->type = type;
    packet->quality = quality;
    packet->id = id;
    packet->download_rate = download_rate;
    packet->upload_rate = upload_rate;
    packet->ber = ber;
    packet->rx_level = rx_level;
    packet->tx_level = tx_level;
    packet->signal_to_noise = signal_to_noise;
    packet->band_number = band_number;
    packet->band_frequency = band_frequency;
    packet->arfcn = arfcn;
    mav_array_memcpy(packet->cell_tower_id, cell_tower_id, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CELLULAR_STATUS, (const char *)packet, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CELLULAR_STATUS UNPACKING


/**
 * @brief Get field status from cellular_status message
 *
 * @return  Cellular modem status
 */
static inline uint8_t mavlink_msg_cellular_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field failure_reason from cellular_status message
 *
 * @return  Failure reason when status in in CELLUAR_STATUS_FAILED
 */
static inline uint8_t mavlink_msg_cellular_status_get_failure_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field type from cellular_status message
 *
 * @return  Cellular network radio type: gsm, cdma, lte...
 */
static inline uint8_t mavlink_msg_cellular_status_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field quality from cellular_status message
 *
 * @return  Signal quality in percent. If unknown, set to UINT8_MAX
 */
static inline uint8_t mavlink_msg_cellular_status_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field mcc from cellular_status message
 *
 * @return  Mobile country code. If unknown, set to UINT16_MAX
 */
static inline uint16_t mavlink_msg_cellular_status_get_mcc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field mnc from cellular_status message
 *
 * @return  Mobile network code. If unknown, set to UINT16_MAX
 */
static inline uint16_t mavlink_msg_cellular_status_get_mnc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field lac from cellular_status message
 *
 * @return  Location area code. If unknown, set to 0
 */
static inline uint16_t mavlink_msg_cellular_status_get_lac(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field id from cellular_status message
 *
 * @return  Cellular instance number
 */
static inline uint8_t mavlink_msg_cellular_status_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field download_rate from cellular_status message
 *
 * @return  download rate in kbits/s
 */
static inline uint64_t mavlink_msg_cellular_status_get_download_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  11);
}

/**
 * @brief Get field upload_rate from cellular_status message
 *
 * @return  upload rate in kbits/s
 */
static inline uint64_t mavlink_msg_cellular_status_get_upload_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  19);
}

/**
 * @brief Get field ber from cellular_status message
 *
 * @return  bit rate error measurement
 */
static inline uint64_t mavlink_msg_cellular_status_get_ber(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  27);
}

/**
 * @brief Get field rx_level from cellular_status message
 *
 * @return  rx level.
 */
static inline float mavlink_msg_cellular_status_get_rx_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  35);
}

/**
 * @brief Get field tx_level from cellular_status message
 *
 * @return  rx level.
 */
static inline float mavlink_msg_cellular_status_get_tx_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  39);
}

/**
 * @brief Get field signal_to_noise from cellular_status message
 *
 * @return  signal to noise.
 */
static inline float mavlink_msg_cellular_status_get_signal_to_noise(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  43);
}

/**
 * @brief Get field cell_tower_id from cellular_status message
 *
 * @return  signal to noise.
 */
static inline uint16_t mavlink_msg_cellular_status_get_cell_tower_id(const mavlink_message_t* msg, char *cell_tower_id)
{
    return _MAV_RETURN_char_array(msg, cell_tower_id, 9,  47);
}

/**
 * @brief Get field band_number from cellular_status message
 *
 * @return  band number.
 */
static inline uint8_t mavlink_msg_cellular_status_get_band_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Get field band_frequency from cellular_status message
 *
 * @return  band number. 
 */
static inline float mavlink_msg_cellular_status_get_band_frequency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  57);
}

/**
 * @brief Get field arfcn from cellular_status message
 *
 * @return  Absolute radio-frequency channel number.
 */
static inline float mavlink_msg_cellular_status_get_arfcn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  61);
}

/**
 * @brief Decode a cellular_status message into a struct
 *
 * @param msg The message to decode
 * @param cellular_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_cellular_status_decode(const mavlink_message_t* msg, mavlink_cellular_status_t* cellular_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cellular_status->mcc = mavlink_msg_cellular_status_get_mcc(msg);
    cellular_status->mnc = mavlink_msg_cellular_status_get_mnc(msg);
    cellular_status->lac = mavlink_msg_cellular_status_get_lac(msg);
    cellular_status->status = mavlink_msg_cellular_status_get_status(msg);
    cellular_status->failure_reason = mavlink_msg_cellular_status_get_failure_reason(msg);
    cellular_status->type = mavlink_msg_cellular_status_get_type(msg);
    cellular_status->quality = mavlink_msg_cellular_status_get_quality(msg);
    cellular_status->id = mavlink_msg_cellular_status_get_id(msg);
    cellular_status->download_rate = mavlink_msg_cellular_status_get_download_rate(msg);
    cellular_status->upload_rate = mavlink_msg_cellular_status_get_upload_rate(msg);
    cellular_status->ber = mavlink_msg_cellular_status_get_ber(msg);
    cellular_status->rx_level = mavlink_msg_cellular_status_get_rx_level(msg);
    cellular_status->tx_level = mavlink_msg_cellular_status_get_tx_level(msg);
    cellular_status->signal_to_noise = mavlink_msg_cellular_status_get_signal_to_noise(msg);
    mavlink_msg_cellular_status_get_cell_tower_id(msg, cellular_status->cell_tower_id);
    cellular_status->band_number = mavlink_msg_cellular_status_get_band_number(msg);
    cellular_status->band_frequency = mavlink_msg_cellular_status_get_band_frequency(msg);
    cellular_status->arfcn = mavlink_msg_cellular_status_get_arfcn(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CELLULAR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CELLULAR_STATUS_LEN;
        memset(cellular_status, 0, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
    memcpy(cellular_status, _MAV_PAYLOAD(msg), len);
#endif
}
