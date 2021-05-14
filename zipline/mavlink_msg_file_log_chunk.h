#pragma once
// MESSAGE FILE_LOG_CHUNK PACKING

#define MAVLINK_MSG_ID_FILE_LOG_CHUNK 7009


typedef struct __mavlink_file_log_chunk_t {
 uint16_t chunks; /*< [chunks] Number of chunks in this file.*/
 uint16_t chunk_index; /*< [chunks] Current chunk (zero-indexed.)*/
 uint8_t filename[20]; /*< [string] Name of the file, as a string.*/
 uint8_t len; /*< [bytes] Number of bytes in this chunk. Should be max (200) unless this is the last chunk.*/
 uint8_t data[200]; /*< [binary] Data.*/
} mavlink_file_log_chunk_t;

#define MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN 225
#define MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN 225
#define MAVLINK_MSG_ID_7009_LEN 225
#define MAVLINK_MSG_ID_7009_MIN_LEN 225

#define MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC 78
#define MAVLINK_MSG_ID_7009_CRC 78

#define MAVLINK_MSG_FILE_LOG_CHUNK_FIELD_FILENAME_LEN 20
#define MAVLINK_MSG_FILE_LOG_CHUNK_FIELD_DATA_LEN 200

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FILE_LOG_CHUNK { \
    7009, \
    "FILE_LOG_CHUNK", \
    5, \
    {  { "filename", NULL, MAVLINK_TYPE_UINT8_T, 20, 4, offsetof(mavlink_file_log_chunk_t, filename) }, \
         { "chunks", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_file_log_chunk_t, chunks) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_file_log_chunk_t, chunk_index) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_file_log_chunk_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 200, 25, offsetof(mavlink_file_log_chunk_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FILE_LOG_CHUNK { \
    "FILE_LOG_CHUNK", \
    5, \
    {  { "filename", NULL, MAVLINK_TYPE_UINT8_T, 20, 4, offsetof(mavlink_file_log_chunk_t, filename) }, \
         { "chunks", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_file_log_chunk_t, chunks) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_file_log_chunk_t, chunk_index) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_file_log_chunk_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 200, 25, offsetof(mavlink_file_log_chunk_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a file_log_chunk message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param filename [string] Name of the file, as a string.
 * @param chunks [chunks] Number of chunks in this file.
 * @param chunk_index [chunks] Current chunk (zero-indexed.)
 * @param len [bytes] Number of bytes in this chunk. Should be max (200) unless this is the last chunk.
 * @param data [binary] Data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_log_chunk_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *filename, uint16_t chunks, uint16_t chunk_index, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN];
    _mav_put_uint16_t(buf, 0, chunks);
    _mav_put_uint16_t(buf, 2, chunk_index);
    _mav_put_uint8_t(buf, 24, len);
    _mav_put_uint8_t_array(buf, 4, filename, 20);
    _mav_put_uint8_t_array(buf, 25, data, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN);
#else
    mavlink_file_log_chunk_t packet;
    packet.chunks = chunks;
    packet.chunk_index = chunk_index;
    packet.len = len;
    mav_array_memcpy(packet.filename, filename, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FILE_LOG_CHUNK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC);
}

/**
 * @brief Pack a file_log_chunk message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param filename [string] Name of the file, as a string.
 * @param chunks [chunks] Number of chunks in this file.
 * @param chunk_index [chunks] Current chunk (zero-indexed.)
 * @param len [bytes] Number of bytes in this chunk. Should be max (200) unless this is the last chunk.
 * @param data [binary] Data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_log_chunk_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *filename,uint16_t chunks,uint16_t chunk_index,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN];
    _mav_put_uint16_t(buf, 0, chunks);
    _mav_put_uint16_t(buf, 2, chunk_index);
    _mav_put_uint8_t(buf, 24, len);
    _mav_put_uint8_t_array(buf, 4, filename, 20);
    _mav_put_uint8_t_array(buf, 25, data, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN);
#else
    mavlink_file_log_chunk_t packet;
    packet.chunks = chunks;
    packet.chunk_index = chunk_index;
    packet.len = len;
    mav_array_memcpy(packet.filename, filename, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FILE_LOG_CHUNK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC);
}

/**
 * @brief Encode a file_log_chunk struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param file_log_chunk C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_log_chunk_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_file_log_chunk_t* file_log_chunk)
{
    return mavlink_msg_file_log_chunk_pack(system_id, component_id, msg, file_log_chunk->filename, file_log_chunk->chunks, file_log_chunk->chunk_index, file_log_chunk->len, file_log_chunk->data);
}

/**
 * @brief Encode a file_log_chunk struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param file_log_chunk C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_log_chunk_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_file_log_chunk_t* file_log_chunk)
{
    return mavlink_msg_file_log_chunk_pack_chan(system_id, component_id, chan, msg, file_log_chunk->filename, file_log_chunk->chunks, file_log_chunk->chunk_index, file_log_chunk->len, file_log_chunk->data);
}

/**
 * @brief Send a file_log_chunk message
 * @param chan MAVLink channel to send the message
 *
 * @param filename [string] Name of the file, as a string.
 * @param chunks [chunks] Number of chunks in this file.
 * @param chunk_index [chunks] Current chunk (zero-indexed.)
 * @param len [bytes] Number of bytes in this chunk. Should be max (200) unless this is the last chunk.
 * @param data [binary] Data.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_file_log_chunk_send(mavlink_channel_t chan, const uint8_t *filename, uint16_t chunks, uint16_t chunk_index, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN];
    _mav_put_uint16_t(buf, 0, chunks);
    _mav_put_uint16_t(buf, 2, chunk_index);
    _mav_put_uint8_t(buf, 24, len);
    _mav_put_uint8_t_array(buf, 4, filename, 20);
    _mav_put_uint8_t_array(buf, 25, data, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_LOG_CHUNK, buf, MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC);
#else
    mavlink_file_log_chunk_t packet;
    packet.chunks = chunks;
    packet.chunk_index = chunk_index;
    packet.len = len;
    mav_array_memcpy(packet.filename, filename, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_LOG_CHUNK, (const char *)&packet, MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC);
#endif
}

/**
 * @brief Send a file_log_chunk message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_file_log_chunk_send_struct(mavlink_channel_t chan, const mavlink_file_log_chunk_t* file_log_chunk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_file_log_chunk_send(chan, file_log_chunk->filename, file_log_chunk->chunks, file_log_chunk->chunk_index, file_log_chunk->len, file_log_chunk->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_LOG_CHUNK, (const char *)file_log_chunk, MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC);
#endif
}

#if MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_file_log_chunk_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *filename, uint16_t chunks, uint16_t chunk_index, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, chunks);
    _mav_put_uint16_t(buf, 2, chunk_index);
    _mav_put_uint8_t(buf, 24, len);
    _mav_put_uint8_t_array(buf, 4, filename, 20);
    _mav_put_uint8_t_array(buf, 25, data, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_LOG_CHUNK, buf, MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC);
#else
    mavlink_file_log_chunk_t *packet = (mavlink_file_log_chunk_t *)msgbuf;
    packet->chunks = chunks;
    packet->chunk_index = chunk_index;
    packet->len = len;
    mav_array_memcpy(packet->filename, filename, sizeof(uint8_t)*20);
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_LOG_CHUNK, (const char *)packet, MAVLINK_MSG_ID_FILE_LOG_CHUNK_MIN_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN, MAVLINK_MSG_ID_FILE_LOG_CHUNK_CRC);
#endif
}
#endif

#endif

// MESSAGE FILE_LOG_CHUNK UNPACKING


/**
 * @brief Get field filename from file_log_chunk message
 *
 * @return [string] Name of the file, as a string.
 */
static inline uint16_t mavlink_msg_file_log_chunk_get_filename(const mavlink_message_t* msg, uint8_t *filename)
{
    return _MAV_RETURN_uint8_t_array(msg, filename, 20,  4);
}

/**
 * @brief Get field chunks from file_log_chunk message
 *
 * @return [chunks] Number of chunks in this file.
 */
static inline uint16_t mavlink_msg_file_log_chunk_get_chunks(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field chunk_index from file_log_chunk message
 *
 * @return [chunks] Current chunk (zero-indexed.)
 */
static inline uint16_t mavlink_msg_file_log_chunk_get_chunk_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field len from file_log_chunk message
 *
 * @return [bytes] Number of bytes in this chunk. Should be max (200) unless this is the last chunk.
 */
static inline uint8_t mavlink_msg_file_log_chunk_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field data from file_log_chunk message
 *
 * @return [binary] Data.
 */
static inline uint16_t mavlink_msg_file_log_chunk_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 200,  25);
}

/**
 * @brief Decode a file_log_chunk message into a struct
 *
 * @param msg The message to decode
 * @param file_log_chunk C-struct to decode the message contents into
 */
static inline void mavlink_msg_file_log_chunk_decode(const mavlink_message_t* msg, mavlink_file_log_chunk_t* file_log_chunk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    file_log_chunk->chunks = mavlink_msg_file_log_chunk_get_chunks(msg);
    file_log_chunk->chunk_index = mavlink_msg_file_log_chunk_get_chunk_index(msg);
    mavlink_msg_file_log_chunk_get_filename(msg, file_log_chunk->filename);
    file_log_chunk->len = mavlink_msg_file_log_chunk_get_len(msg);
    mavlink_msg_file_log_chunk_get_data(msg, file_log_chunk->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN? msg->len : MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN;
        memset(file_log_chunk, 0, MAVLINK_MSG_ID_FILE_LOG_CHUNK_LEN);
    memcpy(file_log_chunk, _MAV_PAYLOAD(msg), len);
#endif
}
