#ifndef FPV_TELEMETRY_H
#define FPV_TELEMETRY_H

#include "crsf_parser.h"

/*
 * Telemetry module:
 * - Receives validated CRSF frames from parser.
 * - Decodes known telemetry frame types to names.
 * - Logs frame type and payload length.
 */

void telemetry_handle_frame(const crsf_frame_t *frame);

#endif /* FPV_TELEMETRY_H */
