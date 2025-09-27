#ifndef DRV_COMM_H
#define DRV_COMM_H

#include <stdint.h>
#include <string.h>

// Protocol definitions
#define FRAME_HEADER_LENGTH 5
#define CMD_ID_LENGTH 2
#define DATA_LENGTH 30
#define FRAME_TAIL_LENGTH 2
#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH)
#define CONTROLLER_CMD_ID 0x0302

#pragma pack(push, 1)

// Data structure (27 bytes)
typedef struct {
    float angle_data[6];     // 4*6 = 24 bytes
    uint8_t button_status[3]; // 3 bytes
} AnglesAndButtons_t;

// Frame structure (39 bytes total)
typedef struct {
    struct {
        uint8_t sof;         // 0xA5 (1 byte)
        uint16_t data_length; // DATA_LENGTH (2 bytes)
        uint8_t seq;         // sequence number (1 byte)
        uint8_t crc8;        // header CRC (1 byte) - total 5 bytes
    } frame_header;

    uint16_t cmd_id;         // CONTROLLER_CMD_ID (2 bytes)
    uint8_t data[DATA_LENGTH]; // payload (30 bytes)
    uint16_t frame_tail;     // CRC16 (2 bytes)
} ControllerFrame_t;

#pragma pack(pop)

// Function declarations
void protocol_init(void);
void protocol_concatenate_data(const AnglesAndButtons_t* sensor_data, ControllerFrame_t* tx_frame);
uint8_t get_CRC8_check_sum(uint8_t *pch_message, uint32_t dw_length, uint8_t ucCRC8);
void append_CRC8_check_sum(uint8_t *pch_message, uint32_t dw_length);
uint16_t get_CRC16_check_sum(uint8_t *pch_message, uint32_t dw_length, uint16_t wCRC);
void append_CRC16_check_sum(uint8_t *pch_message, uint32_t dw_length);

// External constants
extern const uint8_t CRC8_INIT;
extern const uint16_t CRC16_INIT;

#endif
