/*
 * pamodbus-disco — master discovery state machine
 * MIT License — see LICENSE file for details.
 *
 * Implements the master-side automatic MODBUS slave ID discovery protocol:
 *   1. Broadcast a reset to all slaves
 *   2. Broadcast a discovery query (read registers at PA_DISCO_REG_START)
 *   3. Wait for responses within a timing window
 *   4. Assign slave IDs via broadcast write
 *   5. Verify the assigned slave ID via direct read
 *   6. Repeat until no more slaves respond
 *
 * All hardware/OS dependencies are provided via callbacks.
 */

#include "pamodbus-disco-internal.h"
#include <string.h>  /* memset */

/* ---------------------------------------------------------------------------
 * State name strings
 * ------------------------------------------------------------------------- */

static const char * const state_names[] = {
    "PA_DISCO_STATE_IDLE",
    "PA_DISCO_STATE_START",
    "PA_DISCO_STATE_REPEAT",
    "PA_DISCO_STATE_WAIT",
    "PA_DISCO_STATE_VERIFY",
    "PA_DISCO_STATE_RESET_START",
    "PA_DISCO_STATE_RESET_REPEAT",
    "PA_DISCO_STATE_RESET_WAIT",
    "PA_DISCO_STATE_FINISH",
};

/* ---------------------------------------------------------------------------
 * Forward declarations of state handlers
 * ------------------------------------------------------------------------- */

static void do_state_idle(pa_disco_master_t *ctx);
static void do_state_start(pa_disco_master_t *ctx);
static void do_state_repeat(pa_disco_master_t *ctx);
static void do_state_wait(pa_disco_master_t *ctx);
static void do_state_verify(pa_disco_master_t *ctx);
static void do_state_reset_start(pa_disco_master_t *ctx);
static void do_state_reset_repeat(pa_disco_master_t *ctx);
static void do_state_reset_wait(pa_disco_master_t *ctx);
static void do_state_finish(pa_disco_master_t *ctx);

/* ---------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static inline uint8_t inc_slave_id(pa_disco_master_t *ctx)
{
    if (++ctx->slave_id == 0)
        ctx->slave_id = 1;
    return ctx->slave_id;
}

static bool broadcast_query(pa_disco_master_t *ctx)
{
    pa_modbus_t *pam = ctx->pam;
    int rc;

    /* Build a Read Holding Registers request to PA_DISCO_ADDR (0xFF)
     * for PA_DISCO_REG_COUNT (7) registers starting at PA_DISCO_REG_START (23).
     * The pamodbus framer will add the slave address + CRC.
     */
    pa_modbus_set_slave(pam, PA_DISCO_ADDR);
    rc = pa_modbus_build_read_holding_registers(pam, PA_DISCO_REG_START,
                                                PA_DISCO_REG_COUNT);
    if (rc < 0) return false;

    rc = pa_modbus_send(pam);
    return (rc == PA_OK);
}

static bool receive_response(pa_disco_master_t *ctx)
{
    pa_modbus_t *pam = ctx->pam;
    int rc;

    /* Try to receive a response */
    rc = pa_modbus_recv(pam);
    if (rc != PA_OK)
        return false;

    /* Extract the response data */
    uint16_t reg_count = 0;
    for (int i = 0; i < PA_DISCO_REG_COUNT && i < 128; i++) {
        uint16_t val = pa_modbus_get_register(pam, (uint16_t)i);
        if (i == 0) {
            ctx->rsp_slave_id = (uint8_t)(val & 0xFF);
            ctx->rsp_valid = 1;
        } else if (i >= 1 && i <= 6) {
            /* Serial number is 12 bytes in 6 registers */
            ctx->rsp_serialno.hword[i - 1] = val;
        }
        reg_count++;
    }

    return (reg_count >= 1);
}

static bool assign_slave(pa_disco_master_t *ctx, uint8_t slave_id)
{
    pa_modbus_t *pam = ctx->pam;
    int rc;

    /* Build a Write Multiple Registers request to PA_DISCO_ADDR (0xFF)
     * Write PA_DISCO_REG_COUNT registers starting at PA_DISCO_REG_START (23):
     *   reg[0] = slave_id (low byte)
     *   reg[1..6] = serial number (12 bytes in 6 registers)
     *   reg[7..9] = padding (0)
     */
    uint16_t regs[PA_DISCO_REG_COUNT] = {0};
    regs[0] = (uint16_t)slave_id;
    regs[1] = ctx->rsp_serialno.hword[0];
    regs[2] = ctx->rsp_serialno.hword[1];
    regs[3] = ctx->rsp_serialno.hword[2];
    regs[4] = ctx->rsp_serialno.hword[3];
    regs[5] = ctx->rsp_serialno.hword[4];
    regs[6] = ctx->rsp_serialno.hword[5];
    /* regs[7..9] remain 0 */

    /* Flush before sending */
    if (ctx->flush_cb)
        ctx->flush_cb(ctx->userdata);

    pa_modbus_set_slave(pam, PA_DISCO_ADDR);
    rc = pa_modbus_build_write_multiple_registers(pam, PA_DISCO_REG_START,
                                                  regs, PA_DISCO_REG_COUNT);
    if (rc < 0) return false;

    rc = pa_modbus_send(pam);
    return (rc == PA_OK);
}

static bool reset_slaves(pa_disco_master_t *ctx)
{
    pa_modbus_t *pam = ctx->pam;
    int rc;

    /* Build a Write Multiple Registers request to PA_DISCO_ADDR (0xFF)
     * Write PA_DISCO_REG_COUNT registers with slave_id=0 and zero serial number
     */
    uint16_t regs[PA_DISCO_REG_COUNT] = {0};

    if (ctx->flush_cb)
        ctx->flush_cb(ctx->userdata);

    pa_modbus_set_slave(pam, PA_DISCO_ADDR);
    rc = pa_modbus_build_write_multiple_registers(pam, PA_DISCO_REG_START,
                                                  regs, PA_DISCO_REG_COUNT);
    if (rc < 0) return false;

    rc = pa_modbus_send(pam);
    return (rc == PA_OK);
}

static bool verify_slave(pa_disco_master_t *ctx, uint8_t slave_id)
{
    pa_modbus_t *pam = ctx->pam;
    int rc;

    if (ctx->flush_cb)
        ctx->flush_cb(ctx->userdata);

    /* Read the verify register from the assigned slave */
    pa_modbus_set_slave(pam, slave_id);
    rc = pa_modbus_build_read_holding_registers(pam, PA_DISCO_VERIFY_REG,
                                                PA_DISCO_VERIFY_NREG);
    if (rc < 0) return false;

    rc = pa_modbus_send(pam);
    if (rc != PA_OK) return false;

    rc = pa_modbus_recv(pam);
    return (rc == PA_OK);
}

static pa_disco_slave_t *update_slave_list(pa_disco_master_t *ctx)
{
    pa_disco_slave_t *slave = NULL;
    int index;

    /* Check if this serial number already exists in the list */
    index = pa_disco_list_find_serialno(&ctx->list,
                                        ctx->rsp_serialno.hword);
    if (index >= 0) {
        /* Update existing slave ID */
        pa_disco_list_set_slave_id(&ctx->list, index,
                                   pa_disco_master_slave_id(ctx));
        slave = pa_disco_list_at(&ctx->list, index);
    } else {
        /* Add new slave */
        slave = pa_disco_slave_new(
            pa_disco_master_slave_id(ctx),
            ctx->rsp_serialno.hword,
            NULL);
        if (slave) {
            pa_disco_list_append(&ctx->list, slave);
        }
    }
    return slave;
}

/* ---------------------------------------------------------------------------
 * Initialization
 * ------------------------------------------------------------------------- */

void pa_disco_master_init(pa_disco_master_t *ctx, pa_modbus_t *pam)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->pam = pam;
    ctx->reset = true;  /* On first startup, do a full bus reset */
}

void pa_disco_master_set_settings(pa_disco_master_t *ctx,
                                  const pa_disco_settings_t *settings)
{
    ctx->settings = *settings;
}

void pa_disco_master_set_callbacks(pa_disco_master_t *ctx,
    pa_disco_get_ticks_fn     get_ticks,
    pa_disco_flush_fn         flush,
    pa_disco_mutex_lock_fn    lock,
    pa_disco_mutex_unlock_fn  unlock,
    pa_disco_mutex_trylock_fn trylock,
    pa_disco_list_notify_fn   notify,
    void                     *userdata)
{
    ctx->get_ticks_cb = get_ticks;
    ctx->flush_cb     = flush;
    ctx->lock_cb      = lock;
    ctx->unlock_cb    = unlock;
    ctx->trylock_cb   = trylock;
    ctx->notify_cb    = notify;
    ctx->userdata     = userdata;
}

void pa_disco_master_reset(pa_disco_master_t *ctx)
{
    ctx->reset = true;
}

/* ---------------------------------------------------------------------------
 * Public state accessors
 * ------------------------------------------------------------------------- */

uint8_t pa_disco_master_slave_id(const pa_disco_master_t *ctx)
{
    return ctx->slave_id;
}

pa_disco_state_t pa_disco_master_state(const pa_disco_master_t *ctx)
{
    return ctx->state;
}

const char *pa_disco_master_state_str(const pa_disco_master_t *ctx)
{
    if ((size_t)ctx->state < sizeof(state_names) / sizeof(state_names[0]))
        return state_names[ctx->state];
    return "UNKNOWN";
}

pa_disco_list_t *pa_disco_master_list(pa_disco_master_t *ctx)
{
    return &ctx->list;
}

/* ---------------------------------------------------------------------------
 * Service — main entry point, call periodically
 * ------------------------------------------------------------------------- */

void pa_disco_master_service(pa_disco_master_t *ctx)
{
    switch (ctx->state) {
    case PA_DISCO_STATE_IDLE:
        do_state_idle(ctx);
        break;
    case PA_DISCO_STATE_START:
        do_state_start(ctx);
        break;
    case PA_DISCO_STATE_REPEAT:
        do_state_repeat(ctx);
        break;
    case PA_DISCO_STATE_WAIT:
        do_state_wait(ctx);
        break;
    case PA_DISCO_STATE_VERIFY:
        do_state_verify(ctx);
        break;
    case PA_DISCO_STATE_RESET_START:
        do_state_reset_start(ctx);
        break;
    case PA_DISCO_STATE_RESET_REPEAT:
        do_state_reset_repeat(ctx);
        break;
    case PA_DISCO_STATE_RESET_WAIT:
        do_state_reset_wait(ctx);
        break;
    case PA_DISCO_STATE_FINISH:
        do_state_finish(ctx);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * State handlers
 * ------------------------------------------------------------------------- */

static void do_state_idle(pa_disco_master_t *ctx)
{
    bool refresh_expired = false;

    if (ctx->settings.refresh_period != 0 && ctx->get_ticks_cb) {
        uint32_t now = ctx->get_ticks_cb(ctx->userdata);
        uint32_t elapsed = now - ctx->refresh_start;
        if (elapsed >= ctx->settings.refresh_period)
            refresh_expired = true;
    }

    if (ctx->reset || refresh_expired) {
        /* Try to acquire the bus mutex */
        if (ctx->trylock_cb) {
            if (ctx->trylock_cb(ctx->userdata) == 0) {
                ctx->state = PA_DISCO_STATE_RESET_START;
            }
        } else {
            ctx->state = PA_DISCO_STATE_RESET_START;
        }
    }
}

static void do_state_start(pa_disco_master_t *ctx)
{
    ctx->slave_id = 0;
    ctx->reset = false;
    ctx->refresh_start = ctx->get_ticks_cb
        ? ctx->get_ticks_cb(ctx->userdata) : 0;
    ctx->repeat = ctx->settings.repeat_cycles;
    ctx->state = PA_DISCO_STATE_REPEAT;
}

static void do_state_repeat(pa_disco_master_t *ctx)
{
    if (--ctx->repeat >= 0) {
        /* Flush before sending */
        if (ctx->flush_cb)
            ctx->flush_cb(ctx->userdata);

        if (broadcast_query(ctx)) {
            ctx->received = false;
            ctx->wait_start = ctx->get_ticks_cb
                ? ctx->get_ticks_cb(ctx->userdata) : 0;
            ctx->state = PA_DISCO_STATE_WAIT;
        }
    } else {
        ctx->state = PA_DISCO_STATE_FINISH;
    }
}

static void do_state_wait(pa_disco_master_t *ctx)
{
    /* Try to receive a response if we haven't already */
    if (!ctx->received) {
        if (receive_response(ctx)) {
            inc_slave_id(ctx);
            ctx->received = true;
        }
    }

    /* Wait for the window + guard time to expire */
    if (ctx->get_ticks_cb) {
        uint32_t now = ctx->get_ticks_cb(ctx->userdata);
        uint32_t elapsed = now - ctx->wait_start;
        uint32_t window_total = (uint32_t)ctx->settings.window_time +
                                (uint32_t)ctx->settings.window_guard_time;

        if (elapsed >= window_total) {
            if (ctx->received) {
                /* Assign the slave ID */
                if (assign_slave(ctx, ctx->slave_id)) {
                    ctx->repeat++;  /* Allow one more broadcast */
                    ctx->verify_repeat = ctx->settings.verify_repeat_cycles;
                    ctx->state = PA_DISCO_STATE_VERIFY;
                    return;
                }
            }
            ctx->state = PA_DISCO_STATE_REPEAT;
        }
    }
}

static void do_state_verify(pa_disco_master_t *ctx)
{
    if (--ctx->verify_repeat >= 0) {
        bool verified = verify_slave(ctx, ctx->slave_id);

        if (verified) {
            /* Update the slave list */
            pa_disco_slave_t *slave = update_slave_list(ctx);

            /* Notify application of the new slave */
            if (slave && ctx->notify_cb) {
                ctx->notify_cb(&ctx->list, slave, ctx->userdata);
            }

            ctx->verify_repeat = 0;  /* Success, skip remaining retries */
        }
    } else {
        ctx->state = PA_DISCO_STATE_REPEAT;
    }
}

static void do_state_reset_start(pa_disco_master_t *ctx)
{
    ctx->reset_repeat = ctx->settings.reset_repeat_cycles;
    pa_disco_list_clear(&ctx->list);
    ctx->state = PA_DISCO_STATE_RESET_REPEAT;
}

static void do_state_reset_repeat(pa_disco_master_t *ctx)
{
    if (--ctx->reset_repeat >= 0) {
        if (reset_slaves(ctx)) {
            ctx->wait_reset_start = ctx->get_ticks_cb
                ? ctx->get_ticks_cb(ctx->userdata) : 0;
            ctx->state = PA_DISCO_STATE_RESET_WAIT;
        }
    } else {
        ctx->state = PA_DISCO_STATE_START;
    }
}

static void do_state_reset_wait(pa_disco_master_t *ctx)
{
    if (ctx->get_ticks_cb) {
        uint32_t now = ctx->get_ticks_cb(ctx->userdata);
        uint32_t elapsed = now - ctx->wait_reset_start;
        if (elapsed >= (uint32_t)ctx->settings.window_guard_time * 2) {
            ctx->state = PA_DISCO_STATE_RESET_REPEAT;
        }
    }
}

static void do_state_finish(pa_disco_master_t *ctx)
{
    ctx->refresh_start = ctx->get_ticks_cb
        ? ctx->get_ticks_cb(ctx->userdata) : 0;
    ctx->state = PA_DISCO_STATE_IDLE;

    /* Notify application that the discovery cycle is complete */
    if (ctx->notify_cb) {
        ctx->notify_cb(&ctx->list, NULL, ctx->userdata);
    }

    /* Release the bus mutex */
    if (ctx->unlock_cb) {
        ctx->unlock_cb(ctx->userdata);
    }
}