/*
 * pamodbus-disco — register map implementation
 * MIT License — see LICENSE file for details.
 *
 * Provides a holding register storage with auto MODBUS request/response
 * handling via pamodbus. All I/O is done through the pamodbus callbacks.
 */

#include "pamodbus-disco-internal.h"
#include <string.h>  /* memset, memcpy */

/* ---------------------------------------------------------------------------
 * Type conversion helpers (big-endian float32 in two registers)
 * ------------------------------------------------------------------------- */

typedef union {
    char     c[4];
    float    f;
    uint32_t i;
    struct {
        uint16_t h;  /* high word (big-endian) */
        uint16_t l;  /* low word (big-endian) */
    } s;
} pa_disco_convert32_t;

/* ---------------------------------------------------------------------------
 * pamodbus callbacks for register access
 * ------------------------------------------------------------------------- */

static int pam_read_holding_regs(uint16_t addr, uint16_t count,
                                 uint16_t *values, void *userdata)
{
    pa_disco_register_map_t *map = (pa_disco_register_map_t *)userdata;
    uint16_t offset = addr - map->holding_start;
    if (offset + count > PA_DISCO_HOLDING_NREGS)
        return -1;
    for (uint16_t i = 0; i < count; i++)
        values[i] = map->holding_regs[offset + i];
    return 0;
}

static int pam_write_single_reg(uint16_t addr, uint16_t value,
                                void *userdata)
{
    pa_disco_register_map_t *map = (pa_disco_register_map_t *)userdata;
    uint16_t offset = addr - map->holding_start;
    if (offset >= PA_DISCO_HOLDING_NREGS)
        return -1;
    map->holding_regs[offset] = value;
    return 0;
}

static int pam_write_multiple_regs(uint16_t addr, uint16_t count,
                                   const uint16_t *values, void *userdata)
{
    pa_disco_register_map_t *map = (pa_disco_register_map_t *)userdata;
    uint16_t offset = addr - map->holding_start;
    if (offset + count > PA_DISCO_HOLDING_NREGS)
        return -1;
    for (uint16_t i = 0; i < count; i++)
        map->holding_regs[offset + i] = values[i];
    return 0;
}

/* ---------------------------------------------------------------------------
 * Init
 * ------------------------------------------------------------------------- */

void pa_disco_register_map_init(pa_disco_register_map_t *map,
                                pa_modbus_t *pam)
{
    memset(map, 0, sizeof(*map));
    map->pam = pam;

    /* Register pamodbus holding register callbacks */
    pa_modbus_set_read_holding_registers_cb(pam, pam_read_holding_regs, map);
    pa_modbus_set_write_single_register_cb(pam, pam_write_single_reg, map);
    pa_modbus_set_write_multiple_registers_cb(pam, pam_write_multiple_regs, map);
}

void pa_disco_register_map_set_req_cb(pa_disco_register_map_t *map,
                                      pa_disco_req_cb_t cb,
                                      void *arg)
{
    map->req_cb = cb;
    map->req_cb_arg = arg;
}

void pa_disco_register_map_set_holding(pa_disco_register_map_t *map,
                                       uint16_t *regs,
                                       uint16_t start,
                                       uint16_t count)
{
    uint16_t copy_count = count;
    if (copy_count > PA_DISCO_HOLDING_NREGS)
        copy_count = PA_DISCO_HOLDING_NREGS;
    for (uint16_t i = 0; i < copy_count; i++)
        map->holding_regs[i] = regs[i];
    map->holding_start = start;
}

/* ---------------------------------------------------------------------------
 * Service
 * ------------------------------------------------------------------------- */

int pa_disco_register_map_service(pa_disco_register_map_t *map)
{
    pa_modbus_t *pam = map->pam;
    int ret;

    /* Receive data and feed it to the pamodbus slave parser */
    ret = pa_modbus_recv(pam);
    if (ret < 0)
        return ret;   /* No data, timeout, or error */
    if (ret > 0)
        return 0;     /* Still waiting for more data */

    /* A complete request has been parsed. Call the request callback. */
    bool do_reply = true;
    if (map->req_cb) {
        do_reply = (map->req_cb(map->req_cb_arg) != 0);
    }

    if (do_reply) {
        /* Build the response using registered callbacks */
        ret = pa_modbus_slave_respond(pam);
        if (ret < 0)
            return ret;

        /* Send the response */
        ret = pa_modbus_send(pam);
        if (ret != PA_OK)
            return -1;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Accessors
 * ------------------------------------------------------------------------- */

uint16_t *pa_disco_register_map_get_ptr(pa_disco_register_map_t *map,
                                        uint16_t reg)
{
    uint16_t offset = reg - map->holding_start;
    if (offset >= PA_DISCO_HOLDING_NREGS)
        return NULL;
    return &map->holding_regs[offset];
}

uint16_t pa_disco_register_map_get(pa_disco_register_map_t *map,
                                   uint16_t reg)
{
    uint16_t offset = reg - map->holding_start;
    if (offset >= PA_DISCO_HOLDING_NREGS)
        return 0;
    return map->holding_regs[offset];
}

float pa_disco_register_map_get_float32(pa_disco_register_map_t *map,
                                        uint16_t reg)
{
    pa_disco_convert32_t u;
    uint16_t offset = reg - map->holding_start;
    if (offset + 1 >= PA_DISCO_HOLDING_NREGS)
        return 0.0f;

    u.s.h = map->holding_regs[offset + 0];  /* big end */
    u.s.l = map->holding_regs[offset + 1];  /* little end */
    return u.f;
}

void pa_disco_register_map_put(pa_disco_register_map_t *map,
                               uint16_t reg, uint16_t val)
{
    uint16_t offset = reg - map->holding_start;
    if (offset >= PA_DISCO_HOLDING_NREGS)
        return;
    map->holding_regs[offset] = val;
}

void pa_disco_register_map_put_float32(pa_disco_register_map_t *map,
                                       uint16_t reg, float val)
{
    pa_disco_convert32_t u;
    uint16_t offset = reg - map->holding_start;
    if (offset + 1 >= PA_DISCO_HOLDING_NREGS)
        return;

    u.f = val;
    map->holding_regs[offset + 0] = u.s.h;  /* big end */
    map->holding_regs[offset + 1] = u.s.l;  /* little end */
}