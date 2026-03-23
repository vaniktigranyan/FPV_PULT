#include "crsf.h"

#include <string.h>

uint8_t crsf_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00U;

    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ CRSF_CRC_POLY);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

bool crsf_build_rc_channels_packet(const uint16_t ch[CRSF_NUM_CHANNELS],
                                   uint8_t *out,
                                   size_t out_size,
                                   size_t *out_len)
{
    if ((ch == NULL) || (out == NULL) || (out_len == NULL)) {
        return false;
    }

    if (out_size < CRSF_RC_PACKET_SIZE) {
        return false;
    }

    memset(out, 0, CRSF_RC_PACKET_SIZE);

    out[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    out[1] = CRSF_RC_FRAME_LENGTH;
    out[2] = CRSF_TYPE_RC_CHANNELS_PACKED;

    uint8_t *payload = &out[3];
    uint32_t bit_index = 0U;

    for (uint32_t ch_idx = 0; ch_idx < CRSF_NUM_CHANNELS; ++ch_idx) {
        uint16_t value = (uint16_t)(ch[ch_idx] & 0x07FFU);

        for (uint32_t bit = 0; bit < CRSF_CHANNEL_BITS; ++bit) {
            if ((value & (1U << bit)) != 0U) {
                uint32_t abs_bit = bit_index + bit;
                uint32_t byte_index = abs_bit / 8U;
                uint32_t bit_in_byte = abs_bit % 8U;
                payload[byte_index] |= (uint8_t)(1U << bit_in_byte);
            }
        }

        bit_index += CRSF_CHANNEL_BITS;
    }

    const size_t crc_data_len = 1U + CRSF_RC_PAYLOAD_SIZE;
    out[CRSF_RC_PACKET_SIZE - 1U] = crsf_crc8(&out[2], crc_data_len);
    *out_len = CRSF_RC_PACKET_SIZE;
    return true;
}
