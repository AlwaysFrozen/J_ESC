#include "od_scope.h"

_Static_assert(ATOMIC_INT_LOCK_FREE == 2 && ATOMIC_BOOL_LOCK_FREE == 2,
               "sampling requires lock-free state and ownership flags");

void od_scope_init(OD_Scope *s)
{
    atomic_init(&s->state, OD_SCOPE_IDLE);
    for (unsigned i = 0; i < OD_SCOPE_BLOCK_COUNT; ++i) atomic_init(&s->blocks[i].ready, false);
    s->channels = 0; s->stride = 0; s->fill = 0; s->write_block = 0; s->read_block = 0;
    s->tick = 0; s->config_id = 0; s->segment = 0; s->block_sequence = 0;
    s->captured = 0; s->overflows = 0; s->divisor = 1; s->countdown = 0;
    s->mode = OD_SCOPE_BURST; s->samples_per_block = 0;
}

bool od_scope_empty(const OD_Scope *s)
{
    for (unsigned i = 0; i < OD_SCOPE_BLOCK_COUNT; ++i)
        if (atomic_load_explicit(&s->blocks[i].ready, memory_order_acquire)) return false;
    return true;
}

OD_Result od_scope_configure(OD_Scope *s, const OD_Dictionary *d, const OD_ScopeChannel *channels,
                             uint8_t count, uint32_t divisor, uint8_t mode)
{
    if (atomic_load_explicit(&s->state, memory_order_acquire) == OD_SCOPE_RUNNING ||
        !od_scope_empty(s) || s->fill != 0) return OD_BUSY;
    if (channels == NULL || count == 0 || count > OD_SCOPE_MAX_CHANNELS || divisor == 0 ||
        (mode != OD_SCOPE_CONTINUOUS && mode != OD_SCOPE_BURST)) return OD_BAD_ARGUMENT;
    OD_ReadPlan plans[OD_SCOPE_MAX_CHANNELS];
    uint16_t stride = 0;
    for (unsigned i = 0; i < count; ++i) {
        /* Explicit allowlist: published runtime groups only, not config/commands. */
        if (channels[i].index < 0x2100 || channels[i].index > 0x2109 || channels[i].subindex == 0)
            return OD_ACCESS_DENIED;
        OD_Result result = od_prepare_read(d, channels[i].index, channels[i].subindex, &plans[i]);
        if (result != OD_OK) return result;
        if (plans[i].entry->access != OD_RO) return OD_ACCESS_DENIED;
        if (plans[i].entry->type != channels[i].type) return OD_TYPE_MISMATCH;
        if (plans[i].entry->size != 1 && plans[i].entry->size != 2 && plans[i].entry->size != 4)
            return OD_UNSUPPORTED;
        for (unsigned j = 0; j < i; ++j)
            if (plans[j].entry == plans[i].entry) return OD_BAD_ARGUMENT;
        stride = (uint16_t)(stride + plans[i].entry->size);
    }
    for (unsigned i = 0; i < count; ++i) s->plans[i] = plans[i];
    s->channels = count; s->stride = stride; s->divisor = divisor; s->mode = mode;
    s->samples_per_block = (uint16_t)(OD_SCOPE_BLOCK_BYTES / stride);
    ++s->config_id;
    atomic_store_explicit(&s->state, OD_SCOPE_IDLE, memory_order_release);
    return OD_OK;
}

OD_Result od_scope_start(OD_Scope *s)
{
    if (s->channels == 0) return OD_NOT_READY;
    if (atomic_load_explicit(&s->state, memory_order_acquire) == OD_SCOPE_RUNNING ||
        !od_scope_empty(s) || s->fill != 0) return OD_BUSY;
    s->write_block = 0; s->read_block = 0; s->fill = 0; s->countdown = 0;
    s->captured = 0; s->block_sequence = 0; ++s->segment;
    atomic_store_explicit(&s->state, OD_SCOPE_RUNNING, memory_order_release);
    return OD_OK;
}

static void publish(OD_Scope *s)
{
    OD_ScopeBlock *block = &s->blocks[s->write_block];
    block->count = s->fill;
    atomic_store_explicit(&block->ready, true, memory_order_release);
    s->fill = 0;
    s->write_block = (uint8_t)((s->write_block + 1U) % OD_SCOPE_BLOCK_COUNT);
}

void od_scope_stop(OD_Scope *s)
{
    if (s->fill != 0) publish(s);
    atomic_store_explicit(&s->state, OD_SCOPE_IDLE, memory_order_release);
}

void od_scope_tick(OD_Scope *s)
{
    uint64_t tick = s->tick++;
    if (atomic_load_explicit(&s->state, memory_order_acquire) != OD_SCOPE_RUNNING) return;
    if (s->countdown != 0) { --s->countdown; return; }
    s->countdown = s->divisor - 1U;
    OD_ScopeBlock *block = &s->blocks[s->write_block];
    if (atomic_load_explicit(&block->ready, memory_order_acquire)) {
        ++s->overflows;
        atomic_store_explicit(&s->state, OD_SCOPE_OVERFLOW, memory_order_release);
        return;
    }
    if (s->fill == 0) block->first_tick = tick;
    uint8_t *out = block->data + (size_t)s->fill * s->stride;
    for (unsigned i = 0; i < s->channels; ++i) {
        uint8_t size = s->plans[i].entry->size;
        /* Prepared, immutable plans were validated before enabling capture. */
        (void)od_read_prepared(&s->plans[i], out, size);
        out += size;
    }
    ++s->fill; ++s->captured;
    if (s->fill == s->samples_per_block) {
        publish(s);
        if (s->mode == OD_SCOPE_BURST && s->write_block == 0)
            atomic_store_explicit(&s->state, OD_SCOPE_DRAINING, memory_order_release);
    }
}

const OD_ScopeBlock *od_scope_peek(const OD_Scope *s)
{
    if (s->mode == OD_SCOPE_BURST &&
        atomic_load_explicit(&s->state, memory_order_acquire) == OD_SCOPE_RUNNING) return NULL;
    const OD_ScopeBlock *block = &s->blocks[s->read_block];
    return atomic_load_explicit(&block->ready, memory_order_acquire) ? block : NULL;
}

void od_scope_release(OD_Scope *s)
{
    if (od_scope_peek(s) == NULL) return;
    atomic_store_explicit(&s->blocks[s->read_block].ready, false, memory_order_release);
    s->read_block = (uint8_t)((s->read_block + 1U) % OD_SCOPE_BLOCK_COUNT);
    ++s->block_sequence;
}
