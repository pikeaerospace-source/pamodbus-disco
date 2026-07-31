/*
 * modbus-disco-list.h — Compatibility header for modbus-disco list API
 *
 * This header is provided for backward compatibility with code that uses
 * the old modbus-disco library types (modbus_slave_t, modbus_disco_list_t).
 * It maps the old API onto the new pamodbus-disco library.
 *
 * MIT License — see LICENSE file for details.
 */

#ifndef MODBUS_DISCO_LIST_H_
#define MODBUS_DISCO_LIST_H_

#include <board.h>
#include <serialno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Old type definitions (ABI-compatible with original modbus-disco)
 *
 * These types have the same layout as the original modbus-disco library.
 * Note: modbus_slave_t layout DIFFERS from pa_disco_slave_t due to
 * serialno_t alignment requirements (4-byte vs 2-byte). The pamodbus-disco
 * library uses pa_disco_slave_t internally; this compatibility layer
 * maintains the original layout for existing code that embeds modbus_slave_t
 * in other structures (e.g., xduc_param_t).
 * ------------------------------------------------------------------------- */

typedef struct _modbus_slave_t_ {
    uint8_t     slave_id;       /**< The modbus slave id */
    uint8_t     _pad0[3];       /**< Padding for serialno_t alignment */
    serialno_t  serialno;       /**< The unique identifier (serialno) */
    void*       arg;            /**< Application data argument */
} modbus_slave_t;

typedef struct _modbus_disco_list_t_ {
    modbus_slave_t** slave;     /**< List of slave record pointers */
    int              count;     /**< number of slaves in the list */
} modbus_disco_list_t;

/* ---------------------------------------------------------------------------
 * Old API functions
 *
 * These functions are implemented in modbus-disco-list-compat.c and
 * provide the same interface as the original modbus-disco library.
 * ------------------------------------------------------------------------- */

extern modbus_slave_t* modbus_disco_slave_new(  uint8_t slave_id,
                                                serialno_t*  serialno,
                                                void* arg  );
extern void modbus_disco_slave_delete(modbus_slave_t* slave );
extern void modbus_disco_list_new   (modbus_disco_list_t* list);
extern void modbus_disco_list_delete(modbus_disco_list_t* list);
extern int  modbus_disco_list_count (modbus_disco_list_t* list);
extern bool modbus_disco_list_insert(modbus_disco_list_t* list,modbus_slave_t* slave, int index);
extern bool modbus_disco_list_append(modbus_disco_list_t* list,modbus_slave_t* slave);
extern modbus_slave_t* modbus_disco_list_at(modbus_disco_list_t* list, int index);
extern int modbus_disco_list_find_serialno(modbus_disco_list_t* list, serialno_t* serialno);
extern int modbus_disco_list_set_slave_id(modbus_disco_list_t* list, int index, uint8_t slave_id);

#ifdef __cplusplus
}
#endif

#endif