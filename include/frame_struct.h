#pragma once

#include <stdint.h>

#define FRAME_BEGIN_FLAG (0xFF00)
#define FRAME_END_FLAG (0xDD)

#define FRAME_HEAD_SIZE (20)
#define FRAME_HEAD_DATA_SIZE (16)
#define FRAME_CHECKSUM_SIZE (1)
#define FRAME_END_SIZE (1)

struct a010_frame_head_t
{
    uint16_t frame_begin_flag;
    uint16_t frame_data_len;
    uint8_t reserved1;    // fixed to 0xff
    uint8_t output_mode;  // 0:depth only, 1:depth+ir
    uint8_t senser_temp;
    uint8_t driver_temp;
    uint8_t exposure_time[4];
    uint8_t error_code;
    uint8_t reserved2;  // fixed to 0x00
    uint8_t resolution_rows;
    uint8_t resolution_cols;
    uint16_t frame_id;  // 12-bit, 0~4095
    uint8_t isp_version;
    uint8_t reserved3;  // fixed to 0xff
};

struct a010_frame_tail_t
{
    uint8_t checksum;
    uint8_t frame_end_flag;  // fixed to 0xdd
};

struct a010_frame_t
{
    a010_frame_head_t frame_head;
    a010_frame_tail_t frame_tail;
    uint8_t payload[625];
};
