#ifndef OD_MOTION_H
#define OD_MOTION_H
#include "od.h"
/* Submission/heartbeat/status are foreground operations. The high-frequency
 * consumer applies a prepared, bounded command before this cycle's control. */
OD_Result od_motion_submit(const uint8_t *payload, size_t length);
OD_Result od_motion_heartbeat(void);
void od_motion_cancel(void);
void od_motion_tick(bool can_apply);
void od_motion_status(uint8_t out[16]);
#endif
