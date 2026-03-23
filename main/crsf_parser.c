#include "crsf_parser.h"

#include <string.h>

#include "esp_log.h"

#include "crsf.h"

typedef enum {
    CRSF_PARSER_STATE_WAIT_ADDRESS = 0,
    CRSF_PARSER_STATE_WAIT_LENGTH,
    CRSF_PARSER_STATE_READ_BODY,
} crsf_parser_state_t;

static const char *TAG = "crsf_parser";

static void parser_reset(crsf_parser_t *p)
{
    p->state = (uint8_t)CRSF_PARSER_STATE_WAIT_ADDRESS;
    p->index = 0U;
    p->expected_total = 0U;
}

static bool is_extended_type(uint8_t type)
{
    /* According to CRSF docs, types >= 0x28 use extended header (dest+origin). */
    return (type >= 0x28);
}

void crsf_parser_init(crsf_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    parser_reset(p);
}

bool crsf_parser_process_byte(crsf_parser_t *p, uint8_t b, crsf_frame_t *frame_out)
{
    if ((p == NULL) || (frame_out == NULL)) {
        return false;
    }

    switch ((crsf_parser_state_t)p->state) {
    case CRSF_PARSER_STATE_WAIT_ADDRESS:
#if CRSF_PARSER_STRICT_SYNC
        if ((b != 0xC8) && (b != 0x00)) {
            return false;
        }
#endif
        p->buffer[0] = b;
        p->index = 1U;
        p->state = (uint8_t)CRSF_PARSER_STATE_WAIT_LENGTH;
        break;

    case CRSF_PARSER_STATE_WAIT_LENGTH:
        p->buffer[1] = b;
        if ((b < CRSF_MIN_FRAME_LENGTH_FIELD) || (b > CRSF_MAX_FRAME_LENGTH_FIELD)) {
            ESP_LOGW(TAG, "Invalid CRSF length byte: %u", b);
            parser_reset(p);
            break;
        }

        p->expected_total = (uint8_t)(b + 2U);
        p->index = 2U;
        p->state = (uint8_t)CRSF_PARSER_STATE_READ_BODY;
        break;

    case CRSF_PARSER_STATE_READ_BODY:
        if (p->index >= CRSF_PARSER_MAX_PACKET_SIZE) {
            ESP_LOGW(TAG, "Parser buffer overflow, resetting");
            parser_reset(p);
            break;
        }

        p->buffer[p->index++] = b;

        if (p->index == p->expected_total) {
            const uint8_t length = p->buffer[1];
            const uint8_t type = p->buffer[2];
            const uint8_t rx_crc = p->buffer[p->expected_total - 1U];
            const uint8_t calc_crc = crsf_crc8(&p->buffer[2], (size_t)(length - 1U));

            if (rx_crc != calc_crc) {
                ESP_LOGW(TAG, "CRC mismatch type=0x%02X rx=0x%02X calc=0x%02X",
                         type, rx_crc, calc_crc);
                parser_reset(p);
                return false;
            }

            frame_out->address = p->buffer[0];
            frame_out->length = length;
            frame_out->type = type;
            frame_out->is_extended = false;
            frame_out->destination = 0U;
            frame_out->origin = 0U;

            if (is_extended_type(type) && (length >= 4U)) {
                const uint8_t payload_len = (uint8_t)(length - 4U);
                frame_out->is_extended = true;
                frame_out->destination = p->buffer[3];
                frame_out->origin = p->buffer[4];
                frame_out->payload_len = payload_len;
                if (payload_len > 0U) {
                    memcpy(frame_out->payload, &p->buffer[5], payload_len);
                }
            } else {
                const uint8_t payload_len = (uint8_t)(length - 2U);
                frame_out->payload_len = payload_len;
                if (payload_len > 0U) {
                    memcpy(frame_out->payload, &p->buffer[3], payload_len);
                }
            }

            parser_reset(p);
            return true;
        }
        break;

    default:
        parser_reset(p);
        break;
    }

    return false;
}
