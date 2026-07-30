/*
 * pamodbus-disco — slave discovery logic
 * MIT License — see LICENSE file for details.
 *
 * Handles the slave-side discovery protocol:
 *   - Random window delay before responding to discovery queries
 *   - Slave ID assignment from master broadcast writes
 *   - Serial number matching for assignment acceptance
 *
 * All hardware/OS dependencies are provided via callbacks.
 */

#include "pamodbus-disco-internal.h"
#include <string.h>  /* memset */

/* ---------------------------------------------------------------------------
 * Init
 * ------------------------------------------------------------------------- */

void pa_disco_slave_init(pa_disco_slave_dev_t *ctx, pa_modbus_t *pam)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->pam = pam;
}

void pa_disco_slave_set_callbacks(pa_disco_slave_dev_t *ctx,
    pa_disco_get_ticks_fn     get_ticks,
    pa_disco_delay_ms_fn      delay_ms,
    pa_disco_flush_fn         flush,
    pa_disco_random_fn        random,
    pa_disco_slave_notify_fn  notify,
    void                     *userdata)
{
    ctx->get_ticks_cb = get_ticks;
    ctx->delay_ms_cb  = delay_ms;
    ctx->flush_cb     = flush;
    ctx->random_cb    = random;
    ctx->notify_cb    = notify;
    ctx->userdata     = userdata;
}

void pa_disco_slave_set_window(pa_disco_slave_dev_t *ctx,
                               uint16_t min_ms, uint16_t max_ms)
{
    ctx->disco_window_min = min_ms;
    ctx->disco_window_max = max_ms;
}

void pa_disco_slave_set_serialno(pa_disco_slave_dev_t *ctx,
                                 const uint16_t *serialno)
{
    ctx->self_serialno.hword[0] = serialno[0];
    ctx->self_serialno.hword[1] = serialno[1];
    ctx->self_serialno.hword[2] = serialno[2];
}

/* ---------------------------------------------------------------------------
 * Service
 * ------------------------------------------------------------------------- */

bool pa_disco_slave_service(pa_disco_slave_dev_t *ctx)
{
    /* While the window period is active, flush characters from the rx queue */
    if (ctx->disco_window_wait) {
        if (ctx->flush_cb)
            ctx->flush_cb(ctx->userdata);

        /* Is the window period over? */
        if (ctx->get_ticks_cb) {
            uint32_t now = ctx->get_ticks_cb(ctx->userdata);
            uint32_t elapsed = now - ctx->disco_window_start;
            if (elapsed >= (uint32_t)ctx->disco_window_max) {
                ctx->disco_window_wait = false;
            }
        }
        return false;
    }
    return true;
}

/* ---------------------------------------------------------------------------
 * Read callback — called by register map on read request
 * ------------------------------------------------------------------------- */

int pa_disco_slave_read_cb(void *arg, int reg_index, uint16_t nregs)
{
    pa_disco_slave_dev_t *ctx = (pa_disco_slave_dev_t *)arg;
    pa_modbus_t *pam = ctx->pam;

    /* Is it a read of the discovery register file? */
    if (reg_index >= PA_DISCO_REG_START &&
        reg_index <= (PA_DISCO_REG_START + (int)PA_DISCO_REG_COUNT)) {
        /* Is it a full discovery read? */
        if (reg_index == PA_DISCO_REG_START && nregs == PA_DISCO_REG_COUNT) {
            /* Check if we're being addressed by broadcast */
            uint8_t slave_addr = pa_modbus_get_slave(pam);
            if (slave_addr == 0) {
                /* Enter window wait state */
                ctx->disco_window_start = ctx->get_ticks_cb
                    ? ctx->get_ticks_cb(ctx->userdata) : 0;
                ctx->disco_window_wait = true;

                /* Insert a random delay before responding */
                if (ctx->delay_ms_cb && ctx->random_cb) {
                    uint32_t rand_delay = ctx->random_cb(
                        ctx->disco_window_min, ctx->disco_window_max,
                        ctx->userdata);
                    ctx->delay_ms_cb(rand_delay, ctx->userdata);
                }
                return 1;  /* Allow response */
            }
        }
        return 0;  /* Suppress response for partial reads */
    }
    return 1;  /* Allow default handler for other registers */
}

/* ---------------------------------------------------------------------------
 * Write callback — called by register map on write request
 * ------------------------------------------------------------------------- */

int pa_disco_slave_write_cb(void *arg, int reg_index, uint16_t nregs)
{
    pa_disco_slave_dev_t *ctx = (pa_disco_slave_dev_t *)arg;
    pa_modbus_t *pam = ctx->pam;

    /* Is it a write to the discovery register file? */
    if (reg_index >= PA_DISCO_REG_START &&
        reg_index <= (PA_DISCO_REG_START + (int)PA_DISCO_REG_COUNT)) {
        /* Is it a full write? */
        if (reg_index == PA_DISCO_REG_START && nregs == PA_DISCO_REG_COUNT) {
            /* Get the write data from pamodbus */
            const uint16_t *reg_data = pa_modbus_slave_reg_data(pam);
            uint8_t req_slave_id = (uint8_t)(reg_data[0] & 0xFF);

            /* Serial number is in reg_data[1..3] (3 uint16_t = 6 bytes) */
            pa_disco_serialno_t other_serialno;
            other_serialno.hword[0] = reg_data[1];
            other_serialno.hword[1] = reg_data[2];
            other_serialno.hword[2] = reg_data[3];

            /* Determine if the ID needs to be reset or assigned:
             * - If requested serial number is zero (reset)
             * - If requested serial number matches our serial number (assign)
             */
            bool serial_no_is_zero =
                (other_serialno.hword[0] == 0 &&
                 other_serialno.hword[1] == 0 &&
                 other_serialno.hword[2] == 0);
            bool serial_no_is_me =
                (other_serialno.hword[0] == ctx->self_serialno.hword[0] &&
                 other_serialno.hword[1] == ctx->self_serialno.hword[1] &&
                 other_serialno.hword[2] == ctx->self_serialno.hword[2]);

            if (serial_no_is_me || serial_no_is_zero) {
                /* Set the new modbus slave ID */
                if (pa_modbus_get_slave(pam) != req_slave_id) {
                    pa_modbus_set_slave(pam, req_slave_id);

                    /* Notify the application */
                    if (ctx->notify_cb) {
                        ctx->notify_cb(req_slave_id, ctx->userdata);
                    }
                }
            }

            /* Suppress response for broadcast writes */
            return 0;
        }
    }
    return 1;  /* Allow default handler for other registers */
}