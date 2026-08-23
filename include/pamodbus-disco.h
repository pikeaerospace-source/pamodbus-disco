/*
 * pamodbus-disco — Lightweight MODBUS Discovery Protocol Library
 *
 * MIT License — see LICENSE file for details.
 *
 * pamodbus-disco is a pure-protocol MODBUS discovery library that builds on
 * pamodbus. It provides master and slave discovery state machines for
 * automatically assigning MODBUS slave IDs to devices on a bus.
 *
 * No hardware dependencies — all I/O, timing, mutex, and random operations
 * are provided via callbacks.
 */

#ifndef PAMODBUS_DISCO_H
#define PAMODBUS_DISCO_H

#include <stddef.h>   /* size_t */
#include <stdint.h>   /* uint8_t, uint16_t, uint32_t */
#include <stdbool.h>  /* bool */
#include <pamodbus.h> /* pa_modbus_t, pa_error_t, etc. */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------------- */

/** Opaque master discovery context. */
typedef struct pa_disco_master pa_disco_master_t;

/** Opaque slave discovery context. */
typedef struct pa_disco_slave_dev pa_disco_slave_dev_t;

/** Opaque register map context. */
typedef struct pa_disco_register_map pa_disco_register_map_t;

/** Discovered slave record. */
typedef struct pa_disco_slave pa_disco_slave_t;

/** List of discovered slaves. */
typedef struct pa_disco_list pa_disco_list_t;

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */

/** Discovery broadcast address (standard MODBUS reserved range). */
#define PA_DISCO_ADDR              0xFF

/** Number of registers in the discovery register file. */
#define PA_DISCO_REG_COUNT         10

/** Starting register address for discovery register file. */
#define PA_DISCO_REG_START         23

/** Verify read register address. */
#define PA_DISCO_VERIFY_REG        0

/** Number of verify registers to read. */
#define PA_DISCO_VERIFY_NREG       1

/** Number of holding registers in the register map.
 *
 * Must cover reg 0 (verify read, PA_DISCO_VERIFY_REG) and the full discovery
 * block (PA_DISCO_REG_START .. PA_DISCO_REG_START + PA_DISCO_REG_COUNT = 23..32)
 * within a single contiguous window, i.e. >= PA_DISCO_REG_START +
 * PA_DISCO_REG_COUNT (33). 32 was too small to serve both. */
#define PA_DISCO_HOLDING_NREGS     64

/** Verify read response timeout (ms). The master sends the verify read and
 * then polls (non-blocking) for the response across pa_disco_master_service()
 * calls; this is how long it waits before a verification is assumed to have
 * failed and the attempt is retried. */
#define PA_DISCO_VERIFY_TIMEOUT_MS 100

/* ---------------------------------------------------------------------------
 * Enums
 * ------------------------------------------------------------------------- */

/** Discovery master state machine states. */
typedef enum {
    PA_DISCO_STATE_IDLE         = 0,  /**< Doing nothing. */
    PA_DISCO_STATE_START,             /**< Start a discovery cycle. */
    PA_DISCO_STATE_REPEAT,            /**< Repeat discovery broadcast. */
    PA_DISCO_STATE_WAIT,              /**< Wait for replies. */
    PA_DISCO_STATE_VERIFY,            /**< Verify a slave ID. */
    PA_DISCO_STATE_RESET_START,       /**< Start bus reset. */
    PA_DISCO_STATE_RESET_REPEAT,      /**< Repeat reset broadcast. */
    PA_DISCO_STATE_RESET_WAIT,        /**< Wait after reset broadcast. */
    PA_DISCO_STATE_FINISH,            /**< Discovery cycle complete. */
} pa_disco_state_t;

/* ---------------------------------------------------------------------------
 * Callback types
 * ------------------------------------------------------------------------- */

/**
 * Get current time in milliseconds.
 * @param userdata  User-supplied pointer.
 * @return Current time in milliseconds (monotonic).
 */
typedef uint32_t (*pa_disco_get_ticks_fn)(void *userdata);

/**
 * Blocking delay in milliseconds.
 * @param ms        Delay in milliseconds.
 * @param userdata  User-supplied pointer.
 */
typedef void (*pa_disco_delay_ms_fn)(uint32_t ms, void *userdata);

/**
 * Flush the receive buffer (discard all pending received data).
 * @param userdata  User-supplied pointer.
 */
typedef void (*pa_disco_flush_fn)(void *userdata);

/**
 * Lock a mutex (blocking).
 * @param userdata  User-supplied pointer.
 */
typedef void (*pa_disco_mutex_lock_fn)(void *userdata);

/**
 * Unlock a mutex.
 * @param userdata  User-supplied pointer.
 */
typedef void (*pa_disco_mutex_unlock_fn)(void *userdata);

/**
 * Try to lock a mutex (non-blocking).
 * @param userdata  User-supplied pointer.
 * @return 0 if lock acquired, non-zero if busy.
 */
typedef int (*pa_disco_mutex_trylock_fn)(void *userdata);

/**
 * Generate a random number in [min, max].
 * @param min       Minimum value (inclusive).
 * @param max       Maximum value (inclusive).
 * @param userdata  User-supplied pointer.
 * @return Random number in [min, max].
 */
typedef uint32_t (*pa_disco_random_fn)(uint32_t min, uint32_t max, void *userdata);

/**
 * Notification callback for slave ID changes (slave mode).
 * @param slave_id  The new slave ID.
 * @param userdata  User-supplied pointer.
 */
typedef void (*pa_disco_slave_notify_fn)(uint8_t slave_id, void *userdata);

/**
 * Notification callback for slave list changes (master mode).
 * Called once per discovered slave, then once with slave==NULL when
 * the discovery cycle is complete.
 * @param list      The list of all discovered slaves.
 * @param slave     The newly discovered slave, or NULL if cycle complete.
 * @param userdata  User-supplied pointer.
 */
typedef void (*pa_disco_list_notify_fn)(pa_disco_list_t *list,
                                        pa_disco_slave_t *slave,
                                        void *userdata);

/**
 * Request inspection callback (register map).
 * Called when a complete MODBUS request has been parsed.
 * Return true to allow auto-response, false to suppress it.
 * @param arg  User-supplied pointer.
 * @return true to respond, false to suppress response.
 */
typedef int (*pa_disco_req_cb_t)(void *arg);

/* ---------------------------------------------------------------------------
 * Settings
 * ------------------------------------------------------------------------- */

/** Discovery master settings. */
typedef struct {
    uint16_t  window_time;           /**< Window time for replies (ms). */
    uint32_t  refresh_period;        /**< Period between refresh cycles (ms). 0 = no refresh. */
    int       repeat_cycles;         /**< Number of broadcast iterations with zero replies. */
    uint16_t  window_guard_time;     /**< Guard time after window (ms). */
    int       reset_repeat_cycles;   /**< Number of reset broadcast iterations. */
    int       verify_repeat_cycles;  /**< Number of verify attempts per slave. */
} pa_disco_settings_t;

/* ---------------------------------------------------------------------------
 * Master API
 * ------------------------------------------------------------------------- */

/**
 * Initialize the master discovery context.
 * @param ctx   Master discovery context (allocated by caller).
 * @param pam   pamodbus context (must already be initialized with buffers and I/O callbacks).
 */
void pa_disco_master_init(pa_disco_master_t *ctx, pa_modbus_t *pam);

/**
 * Set discovery settings.
 * @param ctx       Master discovery context.
 * @param settings  Discovery settings (copied internally).
 */
void pa_disco_master_set_settings(pa_disco_master_t *ctx,
                                  const pa_disco_settings_t *settings);

/**
 * Set callbacks for hardware/OS abstraction.
 * All callbacks except notify may be NULL (default: no-op or error).
 * @param ctx       Master discovery context.
 * @param get_ticks Get current time in ms callback.
 * @param flush     Flush RX buffer callback.
 * @param lock      Mutex lock callback.
 * @param unlock    Mutex unlock callback.
 * @param trylock   Mutex trylock callback.
 * @param notify    Slave list change notification callback.
 * @param userdata  Passed to all callbacks.
 */
void pa_disco_master_set_callbacks(pa_disco_master_t *ctx,
    pa_disco_get_ticks_fn     get_ticks,
    pa_disco_flush_fn         flush,
    pa_disco_mutex_lock_fn    lock,
    pa_disco_mutex_unlock_fn  unlock,
    pa_disco_mutex_trylock_fn trylock,
    pa_disco_list_notify_fn   notify,
    void                     *userdata);

/**
 * Force a bus reset on the next idle cycle.
 * @param ctx  Master discovery context.
 */
void pa_disco_master_reset(pa_disco_master_t *ctx);

/**
 * Run one iteration of the master discovery state machine.
 * Call this periodically from the main loop.
 * @param ctx  Master discovery context.
 */
void pa_disco_master_service(pa_disco_master_t *ctx);

/**
 * Get the current slave ID being assigned.
 * @param ctx  Master discovery context.
 * @return Current slave ID.
 */
uint8_t pa_disco_master_slave_id(const pa_disco_master_t *ctx);

/**
 * Get the current state of the master discovery state machine.
 * @param ctx  Master discovery context.
 * @return Current state.
 */
pa_disco_state_t pa_disco_master_state(const pa_disco_master_t *ctx);

/**
 * Get a printable string for the current state.
 * @param ctx  Master discovery context.
 * @return String representation of the current state.
 */
const char *pa_disco_master_state_str(const pa_disco_master_t *ctx);

/**
 * Get the list of discovered slaves.
 * @param ctx  Master discovery context.
 * @return Pointer to the slave list.
 */
pa_disco_list_t *pa_disco_master_list(pa_disco_master_t *ctx);

/* ---------------------------------------------------------------------------
 * Slave API
 * ------------------------------------------------------------------------- */

/**
 * Initialize the slave discovery context.
 * @param ctx   Slave discovery context (allocated by caller).
 * @param pam   pamodbus context (must already be initialized with buffers and I/O callbacks).
 */
void pa_disco_slave_init(pa_disco_slave_dev_t *ctx, pa_modbus_t *pam);

/**
 * Set callbacks for hardware/OS abstraction.
 * @param ctx        Slave discovery context.
 * @param get_ticks  Get current time in ms callback.
 * @param delay_ms   Blocking delay callback.
 * @param flush      Flush RX buffer callback.
 * @param random     Random number generator callback.
 * @param notify     Slave ID change notification callback.
 * @param userdata   Passed to all callbacks.
 */
void pa_disco_slave_set_callbacks(pa_disco_slave_dev_t *ctx,
    pa_disco_get_ticks_fn     get_ticks,
    pa_disco_delay_ms_fn      delay_ms,
    pa_disco_flush_fn         flush,
    pa_disco_random_fn        random,
    pa_disco_slave_notify_fn  notify,
    void                     *userdata);

/**
 * Set the discovery window timing parameters.
 * @param ctx       Slave discovery context.
 * @param min_ms    Minimum window delay in milliseconds.
 * @param max_ms    Maximum window delay in milliseconds.
 */
void pa_disco_slave_set_window(pa_disco_slave_dev_t *ctx,
                               uint16_t min_ms, uint16_t max_ms);

/**
 * Set the self serial number (unique identifier).
 * @param ctx       Slave discovery context.
 * @param serialno  Pointer to 12-byte serial number (6 uint16_t values).
 */
void pa_disco_slave_set_serialno(pa_disco_slave_dev_t *ctx,
                                 const uint16_t *serialno);

/**
 * Service the slave discovery state machine.
 * Call this periodically from the main loop.
 * @param ctx  Slave discovery context.
 * @return true if the register map should be serviced, false if
 *         the slave is in a window wait state.
 */
bool pa_disco_slave_service(pa_disco_slave_dev_t *ctx);

/**
 * Slave read callback — called by the register map when a read request
 * is received. Handles discovery window timing.
 * @param ctx        Slave discovery context.
 * @param reg_index  The register index being read.
 * @param nregs      Number of registers being read.
 * @return true if the register map should auto-respond, false to suppress.
 */
int pa_disco_slave_read_cb(void *ctx, int reg_index, uint16_t nregs);

/**
 * Slave write callback — called by the register map when a write request
 * is received. Handles slave ID assignment from the master.
 * @param ctx        Slave discovery context.
 * @param reg_index  The register index being written.
 * @param nregs      Number of registers being written.
 * @return true if the register map should auto-respond, false to suppress.
 */
int pa_disco_slave_write_cb(void *ctx, int reg_index, uint16_t nregs);

/* ---------------------------------------------------------------------------
 * Register Map API
 * ------------------------------------------------------------------------- */

/**
 * Initialize the register map.
 * @param map   Register map context (allocated by caller).
 * @param pam   pamodbus context (must already be initialized with buffers
 *              and register callbacks).
 */
void pa_disco_register_map_init(pa_disco_register_map_t *map,
                                pa_modbus_t *pam);

/**
 * Set the request inspection callback.
 * Called when a complete MODBUS request is parsed.
 * @param map   Register map context.
 * @param cb    Callback function.
 * @param arg   User-supplied pointer passed to callback.
 */
void pa_disco_register_map_set_req_cb(pa_disco_register_map_t *map,
                                      pa_disco_req_cb_t cb,
                                      void *arg);

/**
 * Set the holding register storage.
 * @param map    Register map context.
 * @param regs   Pointer to register storage array.
 * @param start  Starting register address.
 * @param count  Number of registers.
 */
void pa_disco_register_map_set_holding(pa_disco_register_map_t *map,
                                       uint16_t *regs,
                                       uint16_t start,
                                       uint16_t count);

/**
 * Service the register map: receive data, parse request, auto-respond.
 * @param map  Register map context.
 * @return 0 on success, negative on error.
 */
int pa_disco_register_map_service(pa_disco_register_map_t *map);

/**
 * Get a pointer to a register in the map.
 * @param map  Register map context.
 * @param reg  Register address.
 * @return Pointer to the register value, or NULL if out of range.
 */
uint16_t *pa_disco_register_map_get_ptr(pa_disco_register_map_t *map,
                                        uint16_t reg);

/**
 * Get a register value.
 * @param map  Register map context.
 * @param reg  Register address.
 * @return Register value, or 0 if out of range.
 */
uint16_t pa_disco_register_map_get(pa_disco_register_map_t *map,
                                   uint16_t reg);

/**
 * Get a float32 value from two consecutive registers (big-endian).
 * @param map  Register map context.
 * @param reg  Starting register address.
 * @return Float value, or 0.0f if out of range.
 */
float pa_disco_register_map_get_float32(pa_disco_register_map_t *map,
                                        uint16_t reg);

/**
 * Set a register value.
 * @param map  Register map context.
 * @param reg  Register address.
 * @param val  Value to write.
 */
void pa_disco_register_map_put(pa_disco_register_map_t *map,
                               uint16_t reg, uint16_t val);

/**
 * Set a float32 value into two consecutive registers (big-endian).
 * @param map  Register map context.
 * @param reg  Starting register address.
 * @param val  Float value to write.
 */
void pa_disco_register_map_put_float32(pa_disco_register_map_t *map,
                                       uint16_t reg, float val);

/* ---------------------------------------------------------------------------
 * Slave List API
 * ------------------------------------------------------------------------- */

/**
 * Create a new slave record.
 * @param slave_id  The slave ID.
 * @param serialno  Pointer to serial number (6 uint16_t values).
 * @param arg       Application data pointer (may be NULL).
 * @return Pointer to new slave record, or NULL on allocation failure.
 */
pa_disco_slave_t *pa_disco_slave_new(uint8_t slave_id,
                                     const uint16_t *serialno,
                                     void *arg);

/**
 * Delete a slave record (free memory).
 * @param slave  Slave record to delete.
 */
void pa_disco_slave_delete(pa_disco_slave_t *slave);

/**
 * Initialize an empty slave list.
 * @param list  Slave list to initialize.
 */
void pa_disco_list_init(pa_disco_list_t *list);

/**
 * Clear all slaves from the list and free memory.
 * @param list  Slave list to clear.
 */
void pa_disco_list_clear(pa_disco_list_t *list);

/**
 * Get the number of slaves in the list.
 * @param list  Slave list.
 * @return Number of slaves.
 */
int pa_disco_list_count(const pa_disco_list_t *list);

/**
 * Insert a slave at a specific index.
 * @param list   Slave list.
 * @param slave  Slave record to insert.
 * @param index  Insertion index.
 * @return true on success, false on failure.
 */
bool pa_disco_list_insert(pa_disco_list_t *list, pa_disco_slave_t *slave,
                          int index);

/**
 * Append a slave to the end of the list.
 * @param list   Slave list.
 * @param slave  Slave record to append.
 * @return true on success, false on failure.
 */
bool pa_disco_list_append(pa_disco_list_t *list, pa_disco_slave_t *slave);

/**
 * Get a slave at a specific index.
 * @param list   Slave list.
 * @param index  Index.
 * @return Pointer to slave record, or NULL if out of range.
 */
pa_disco_slave_t *pa_disco_list_at(const pa_disco_list_t *list, int index);

/**
 * Find a slave by serial number.
 * @param list      Slave list.
 * @param serialno  Pointer to serial number (6 uint16_t values).
 * @return Index of the matching slave, or -1 if not found.
 */
int pa_disco_list_find_serialno(const pa_disco_list_t *list,
                                const uint16_t *serialno);

/**
 * Set the slave ID of a slave at a specific index.
 * @param list      Slave list.
 * @param index     Index.
 * @param slave_id  New slave ID.
 * @return 0 on success, -1 on error.
 */
int pa_disco_list_set_slave_id(pa_disco_list_t *list, int index,
                               uint8_t slave_id);

/**
 * Get the slave ID of a slave record.
 * @param slave  Slave record.
 * @return Slave ID.
 */
uint8_t pa_disco_slave_get_id(const pa_disco_slave_t *slave);

/**
 * Get the serial number of a slave record.
 * @param slave  Slave record.
 * @return Pointer to serial number (6 uint16_t values).
 */
const uint16_t *pa_disco_slave_get_serialno(const pa_disco_slave_t *slave);

/**
 * Get the application data pointer of a slave record.
 * @param slave  Slave record.
 * @return Application data pointer.
 */
void *pa_disco_slave_get_arg(const pa_disco_slave_t *slave);

#ifdef __cplusplus
}
#endif

#endif /* PAMODBUS_DISCO_H */