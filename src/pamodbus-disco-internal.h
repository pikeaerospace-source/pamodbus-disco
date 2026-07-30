/*
 * pamodbus-disco — internal definitions
 * MIT License — see LICENSE file for details.
 *
 * This header defines the internal structures for the pamodbus-disco library.
 * Consumers should only include pamodbus-disco.h.
 */

#ifndef PAMODBUS_DISCO_INTERNAL_H
#define PAMODBUS_DISCO_INTERNAL_H

#include "pamodbus-disco.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Error codes (internal)
 * ------------------------------------------------------------------------- */

#define PA_DISCO_ERR_UNKNOWN  -1

/* ---------------------------------------------------------------------------
 * Serial number helpers (6-byte serial number in 3 uint16_t's)
 * ------------------------------------------------------------------------- */

/** Serial number type (6 bytes). */
typedef struct {
    uint16_t hword[3];
} pa_disco_serialno_t;

/* The consumer may typedef their own serialno_t that is ABI-compatible.
 * pa_disco_serialno_t is used internally for storage. */

/* ---------------------------------------------------------------------------
 * Slave record (internal list entry)
 * ------------------------------------------------------------------------- */

struct pa_disco_slave {
    uint8_t              slave_id;   /**< Modbus slave ID. */
    pa_disco_serialno_t  serialno;   /**< Unique serial number. */
    void                *arg;        /**< Application data. */
};

/* ---------------------------------------------------------------------------
 * Slave list (internal)
 * ------------------------------------------------------------------------- */

struct pa_disco_list {
    pa_disco_slave_t **slave;    /**< Array of pointers to slave records. */
    int                count;    /**< Number of slaves in the list. */
    int                capacity; /**< Allocated capacity. */
};

/* ---------------------------------------------------------------------------
 * Master discovery state machine (internal)
 * ------------------------------------------------------------------------- */

struct pa_disco_master {
    /* ---- pamodbus context ---- */
    pa_modbus_t *pam;               /**< pamodbus context (borrowed). */

    /* ---- Settings ---- */
    pa_disco_settings_t settings;   /**< Discovery settings. */

    /* ---- Callbacks ---- */
    pa_disco_get_ticks_fn     get_ticks_cb;
    pa_disco_flush_fn         flush_cb;
    pa_disco_mutex_lock_fn    lock_cb;
    pa_disco_mutex_unlock_fn  unlock_cb;
    pa_disco_mutex_trylock_fn trylock_cb;
    pa_disco_list_notify_fn   notify_cb;
    void                     *userdata;  /**< Passed to all callbacks. */

    /* ---- State machine ---- */
    pa_disco_state_t  state;            /**< Current state. */
    int               repeat;           /**< Broadcast repeat counter. */
    int               reset_repeat;     /**< Reset repeat counter. */
    int               verify_repeat;    /**< Verify repeat counter. */
    uint32_t          refresh_start;    /**< Ticks at start of refresh period. */
    uint32_t          wait_start;       /**< Ticks at start of wait period. */
    uint32_t          wait_reset_start; /**< Ticks at start of reset wait. */
    uint8_t           slave_id;         /**< Next slave ID to assign. */
    bool              received;         /**< Response received in current window. */
    bool              reset;            /**< Reset was asserted. */

    /* ---- Response data ---- */
    pa_disco_serialno_t rsp_serialno;   /**< Serial number from last response. */
    uint8_t             rsp_slave_id;   /**< Slave ID from last response. */
    uint8_t             rsp_valid;      /**< Non-zero if response data valid. */

    /* ---- Discovered slaves ---- */
    pa_disco_list_t  list;             /**< List of discovered slaves. */
};

/* ---------------------------------------------------------------------------
 * Slave discovery state (internal)
 * ------------------------------------------------------------------------- */

struct pa_disco_slave_dev {
    /* ---- pamodbus context ---- */
    pa_modbus_t *pam;               /**< pamodbus context (borrowed). */

    /* ---- Callbacks ---- */
    pa_disco_get_ticks_fn     get_ticks_cb;
    pa_disco_delay_ms_fn      delay_ms_cb;
    pa_disco_flush_fn         flush_cb;
    pa_disco_random_fn        random_cb;
    pa_disco_slave_notify_fn  notify_cb;
    void                     *userdata;

    /* ---- Discovery window state ---- */
    uint16_t  disco_window_min;     /**< Minimum window delay (ms). */
    uint16_t  disco_window_max;     /**< Maximum window delay (ms). */
    bool      disco_window_wait;    /**< Currently in window wait period. */
    uint32_t  disco_window_start;   /**< Ticks when window wait started. */

    /* ---- Identity ---- */
    pa_disco_serialno_t  zero_serialno;
    pa_disco_serialno_t  self_serialno;
};

/* ---------------------------------------------------------------------------
 * Register map (internal)
 * ------------------------------------------------------------------------- */

struct pa_disco_register_map {
    pa_modbus_t *pam;               /**< pamodbus context (borrowed). */

    uint16_t  holding_regs[PA_DISCO_HOLDING_NREGS]; /**< Register storage. */
    uint16_t  holding_start;        /**< Starting register address. */

    /* Callback to inspect/handle requests before auto-respond */
    pa_disco_req_cb_t  req_cb;
    void              *req_cb_arg;
};

#ifdef __cplusplus
}
#endif

#endif /* PAMODBUS_DISCO_INTERNAL_H */