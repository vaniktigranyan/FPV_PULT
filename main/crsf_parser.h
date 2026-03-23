#ifndef FPV_CRSF_PARSER_H
#define FPV_CRSF_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef CRSF_PARSER_STRICT_SYNC
#define CRSF_PARSER_STRICT_SYNC 0
#endif

#define CRSF_MAX_FRAME_LENGTH_FIELD    62U
#define CRSF_MIN_FRAME_LENGTH_FIELD    2U
#define CRSF_PARSER_MAX_PAYLOAD_SIZE   60U
#define CRSF_PARSER_MAX_PACKET_SIZE    (CRSF_MAX_FRAME_LENGTH_FIELD + 2U)

typedef struct {
    uint8_t address;
    uint8_t length;
    uint8_t type;
    bool is_extended;
    uint8_t destination;
    uint8_t origin;
    uint8_t payload[CRSF_PARSER_MAX_PAYLOAD_SIZE];
    uint8_t payload_len;
} crsf_frame_t;

typedef struct {
    uint8_t state;
    uint8_t buffer[CRSF_PARSER_MAX_PACKET_SIZE];
    uint8_t index;
    uint8_t expected_total;
} crsf_parser_t;

void crsf_parser_init(crsf_parser_t *p);
bool crsf_parser_process_byte(crsf_parser_t *p, uint8_t b, crsf_frame_t *frame_out);

#endif /* FPV_CRSF_PARSER_H */
