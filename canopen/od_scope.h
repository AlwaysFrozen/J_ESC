#ifndef OD_SCOPE_H
#define OD_SCOPE_H
#include "od.h"
#include <stdatomic.h>

/* Initial conservative budget: 16 KiB sample storage; tune after hardware timing.
 * Only <=32-bit runtime RO scalars are accepted, avoiding torn 64-bit reads. */
#define OD_SCOPE_MAX_CHANNELS 16U
#define OD_SCOPE_BLOCK_BYTES 1024U
#define OD_SCOPE_BLOCK_COUNT 16U
enum { OD_SCOPE_CONTINUOUS = 1, OD_SCOPE_BURST = 2 };
enum { OD_SCOPE_IDLE = 0, OD_SCOPE_RUNNING = 1, OD_SCOPE_DRAINING = 2, OD_SCOPE_OVERFLOW = 3 };

typedef struct { uint16_t index; uint8_t subindex; OD_Type type; } OD_ScopeChannel;
typedef struct {
    atomic_bool ready;
    uint64_t first_tick;
    uint16_t count;
    uint8_t data[OD_SCOPE_BLOCK_BYTES];
} OD_ScopeBlock;
typedef struct {
    OD_ReadPlan plans[OD_SCOPE_MAX_CHANNELS];
    OD_ScopeBlock blocks[OD_SCOPE_BLOCK_COUNT];
    atomic_uint state;
    uint32_t divisor, countdown, config_id, segment, block_sequence;
    uint32_t captured, overflows;
    uint64_t tick;
    uint16_t stride, samples_per_block, fill;
    uint8_t channels, mode, write_block, read_block;
} OD_Scope;

/* Initialize before the control ISR starts. Foreground configure/start/stop are
 * serialized and called with the sampling ISR excluded. Never reset in-flight
 * blocks unless the transport has already copied their data. */
void od_scope_init(OD_Scope *s);
OD_Result od_scope_configure(OD_Scope *, const OD_Dictionary *, const OD_ScopeChannel *,
                             uint8_t count, uint32_t divisor, uint8_t mode);
OD_Result od_scope_start(OD_Scope *);
void od_scope_stop(OD_Scope *);
/* Called once per high-frequency control cycle, at the documented sample point. */
void od_scope_tick(OD_Scope *);
/* Foreground consumer; burst blocks are invisible until capture is frozen. */
const OD_ScopeBlock *od_scope_peek(const OD_Scope *);
void od_scope_release(OD_Scope *);
bool od_scope_empty(const OD_Scope *);

#endif
