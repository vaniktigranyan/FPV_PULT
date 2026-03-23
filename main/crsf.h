#ifndef FPV_CRSF_H
#define FPV_CRSF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * CRSF frame format (byte stream):
 *   [0] address
 *   [1] length  = bytes from type to CRC (type + payload + crc)
 *   [2] type
 *   [3..N] payload
 *   [last] crc8(type + payload), poly 0xD5
 */

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_CRC_POLY                  0xD5

#define CRSF_TYPE_RC_CHANNELS_PACKED   0x16

#define CRSF_NUM_CHANNELS              16U
#define CRSF_CHANNEL_BITS              11U
#define CRSF_RC_PAYLOAD_SIZE           22U
#define CRSF_RC_FRAME_LENGTH           24U
#define CRSF_RC_PACKET_SIZE            26U

#define CRSF_CH_MIN                    172U
#define CRSF_CH_MID                    992U
#define CRSF_CH_MAX                    1811U

uint8_t crsf_crc8(const uint8_t *data, size_t len);
bool crsf_build_rc_channels_packet(const uint16_t ch[CRSF_NUM_CHANNELS],
                                   uint8_t *out,
                                   size_t out_size,
                                   size_t *out_len);

#endif /* FPV_CRSF_H */
