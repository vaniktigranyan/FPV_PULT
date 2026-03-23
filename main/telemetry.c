#include "telemetry.h"

#include <inttypes.h>

#include "esp_log.h"

static const char *TAG = "telemetry";

static const char *crsf_type_to_name(uint8_t type)
{
    switch (type) {
    case 0x02:
        return "GPS";
    case 0x03:
        return "GPS_TIME";
    case 0x06:
        return "GPS_EXTENDED";
    case 0x07:
        return "VARIO";
    case 0x08:
        return "BATTERY_SENSOR";
    case 0x09:
        return "BARO_ALTITUDE";
    case 0x0A:
        return "AIRSPEED";
    case 0x0B:
        return "HEARTBEAT";
    case 0x0F:
        return "DISCONTINUED";
    case 0x10:
        return "VTX_TELEMETRY";
    case 0x11:
        return "BAROMETER";
    case 0x12:
        return "MAGNETOMETER";
    case 0x13:
        return "ACCEL_GYRO";
    case 0x14:
        return "LINK_STATISTICS";
    case 0x16:
        return "RC_CHANNELS_PACKED";
    case 0x17:
        return "SUBSET_RC_CHANNELS_PACKED";
    case 0x1C:
        return "LINK_STATISTICS_RX";
    case 0x1D:
        return "LINK_STATISTICS_TX";
    case 0x1E:
        return "ATTITUDE";
    case 0x1F:
        return "MAVLINK_FC";
    case 0x21:
        return "FLIGHT_MODE";
    case 0x22:
        return "ESP_NOW_MESSAGES";
    case 0x28:
        return "PARAMETER_PING_DEVICES";
    case 0x29:
        return "PARAMETER_DEVICE_INFO";
    case 0x2B:
        return "PARAMETER_SETTINGS_ENTRY";
    case 0x2C:
        return "PARAMETER_READ";
    case 0x2D:
        return "PARAMETER_WRITE";
    case 0x32:
        return "DIRECT_COMMANDS";
    case 0x34:
        return "LOGGING";
    case 0x3A:
        return "REMOTE_RELATED";
    case 0x7A:
        return "MSP_REQUEST";
    case 0x7B:
        return "MSP_RESPONSE";
    case 0x80:
        return "ARDUPILOT_PASSTHROUGH";
    case 0x88:
        return "ROTORFLIGHT_TELEM_ENVELOPE";
    case 0xAA:
        return "MAVLINK_ENVELOPE";
    case 0xAC:
        return "MAVLINK_SYS_STATUS_SENSOR";
    default:
        return "UNKNOWN";
    }
}

void telemetry_handle_frame(const crsf_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    if (frame->is_extended) {
        ESP_LOGI(TAG, "RX telemetry: type=%s (0x%02X) payload_len=%" PRIu8
                      " addr=0x%02X dst=0x%02X src=0x%02X",
                 crsf_type_to_name(frame->type),
                 frame->type,
                 frame->payload_len,
                 frame->address,
                 frame->destination,
                 frame->origin);
    } else {
        ESP_LOGI(TAG, "RX telemetry: type=%s (0x%02X) payload_len=%" PRIu8 " addr=0x%02X",
                 crsf_type_to_name(frame->type), frame->type, frame->payload_len, frame->address);
    }
}
