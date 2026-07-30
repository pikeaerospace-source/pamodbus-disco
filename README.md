# pamodbus-disco — MODBUS Automatic Slave ID Discovery Library

**pamodbus-disco** is a lightweight, portable MODBUS discovery protocol library written in C, built on top of [pamodbus](https://github.com/8bitgeek/pamodbus). It implements an automatic slave ID assignment protocol for MODBUS RTU networks, allowing a master to discover and assign unique IDs to unconfigured slave devices on a shared bus.

Like pamodbus, this library is **pure protocol** — it handles the discovery state machine, message construction, and response parsing, while all hardware interactions (UART, timers, mutexes, random number generation) are bridged through simple callbacks. No threads, no HAL dependencies, no OS coupling.

## Architecture

```
┌──────────────────────────────────────────────────┐
│               Application (Consumer)              │
│  Provides: UART, timers, mutex, random callbacks  │
├──────────────────────────────────────────────────┤
│              pamodbus-disco Library               │
│  Master: Discovery state machine, slave list mgmt │
│  Slave:  Window timing, ID assignment, reg map    │
├──────────────────────────────────────────────────┤
│                  pamodbus Library                  │
│  PDU build/parse, RTU/TCP framing, CRC-16         │
├──────────────────────────────────────────────────┤
│              Hardware (UART, RS-485)               │
└──────────────────────────────────────────────────┘
```

## Design Principles

- **Pure protocol** — no hardware dependencies in the library. All I/O, timing, mutex, and random operations are provided via callbacks.
- **Callbacks for notifications** — slave ID changes and discovery cycle completion are reported to the application through callbacks.
- **Public prefix `pa_disco_`** — consistent with the pamodbus naming convention.
- **User-supplied memory** — the consumer provides all buffers and context structures. The library performs no heap allocations (except the slave list, which uses `malloc`/`realloc`).
- **C99 compatible** — broad compiler support from GCC to IAR to ARMCC.

## Discovery Protocol

The protocol uses standard MODBUS function codes over the broadcast address `0xFF`:

1. **Reset** — Master broadcasts a write of 7 registers at address 23 with `slave_id=0` and zero serial number. All slaves reset their assigned ID to 0 (unassigned).
2. **Query** — Master broadcasts a read of 7 registers at address 23. Unassigned slaves (ID=0) respond after a random delay.
3. **Assign** — Master sends a write of 7 registers at address 23 with the new slave ID and the responding slave's serial number. The matching slave adopts the new ID.
4. **Verify** — Master reads register 0 from the newly assigned slave to confirm the ID is active.
5. **Repeat** — Steps 2-4 repeat until no more responses are received within the window.

### Register Layout

| Offset | Field | Size |
|--------|-------|------|
| 0 | Slave ID (assigned by master) | uint16 |
| 1-3 | Serial Number (6 bytes) | 3 × uint16 |
| 4-6 | Padding (reserved) | 3 × uint16 |

## Types

### `pa_disco_master_t`

Opaque master discovery context. All state is internal. The consumer allocates the struct and initializes it with `pa_disco_master_init()`.

### `pa_disco_slave_dev_t`

Opaque slave discovery context. All state is internal. The consumer allocates the struct and initializes it with `pa_disco_slave_init()`.

### `pa_disco_register_map_t`

Opaque register map context. Provides holding register storage with automatic MODBUS request/response handling via pamodbus.

### `pa_disco_slave_t`

Discovered slave record. Contains slave ID, serial number, and an application data pointer.

### `pa_disco_list_t`

Dynamic list of discovered slave records. Managed by the master discovery state machine.

### `pa_disco_state_t`

```c
typedef enum {
    PA_DISCO_STATE_IDLE         = 0,  /* Doing nothing */
    PA_DISCO_STATE_START,              /* Start a discovery cycle */
    PA_DISCO_STATE_REPEAT,             /* Repeat discovery broadcast */
    PA_DISCO_STATE_WAIT,               /* Wait for replies */
    PA_DISCO_STATE_VERIFY,             /* Verify a slave ID */
    PA_DISCO_STATE_RESET_START,        /* Start bus reset */
    PA_DISCO_STATE_RESET_REPEAT,       /* Repeat reset broadcast */
    PA_DISCO_STATE_RESET_WAIT,         /* Wait after reset broadcast */
    PA_DISCO_STATE_FINISH,             /* Discovery cycle complete */
} pa_disco_state_t;
```

### `pa_disco_settings_t`

```c
typedef struct {
    uint16_t  window_time;           /* Window time for replies (ms) */
    uint32_t  refresh_period;        /* Period between refresh cycles (ms) */
    int       repeat_cycles;         /* Broadcast iterations with zero replies */
    uint16_t  window_guard_time;     /* Guard time after window (ms) */
    int       reset_repeat_cycles;   /* Reset broadcast iterations */
    int       verify_repeat_cycles;  /* Verify attempts per slave */
} pa_disco_settings_t;
```

## Callback Types

### I/O and Timing

```c
/* Get current time in milliseconds */
typedef uint32_t (*pa_disco_get_ticks_fn)(void *userdata);

/* Blocking delay in milliseconds */
typedef void (*pa_disco_delay_ms_fn)(uint32_t ms, void *userdata);

/* Flush receive buffer */
typedef void (*pa_disco_flush_fn)(void *userdata);
```

### Mutex (RS-485 Bus Access)

```c
typedef void (*pa_disco_mutex_lock_fn)(void *userdata);
typedef void (*pa_disco_mutex_unlock_fn)(void *userdata);
typedef int  (*pa_disco_mutex_trylock_fn)(void *userdata);
```

### Random Number (Slave Window Randomization)

```c
typedef uint32_t (*pa_disco_random_fn)(uint32_t min, uint32_t max, void *userdata);
```

### Notifications

```c
/* Slave ID changed (slave mode) */
typedef void (*pa_disco_slave_notify_fn)(uint8_t slave_id, void *userdata);

/* Slave list changed (master mode) */
typedef void (*pa_disco_list_notify_fn)(pa_disco_list_t *list,
                                        pa_disco_slave_t *slave,
                                        void *userdata);
```

## API Reference

### Master API

```c
void pa_disco_master_init(pa_disco_master_t *ctx, pa_modbus_t *pam);
void pa_disco_master_set_settings(pa_disco_master_t *ctx,
                                  const pa_disco_settings_t *settings);
void pa_disco_master_set_callbacks(pa_disco_master_t *ctx,
    pa_disco_get_ticks_fn     get_ticks,
    pa_disco_flush_fn         flush,
    pa_disco_mutex_lock_fn    lock,
    pa_disco_mutex_unlock_fn  unlock,
    pa_disco_mutex_trylock_fn trylock,
    pa_disco_list_notify_fn   notify,
    void                     *userdata);
void pa_disco_master_reset(pa_disco_master_t *ctx);
void pa_disco_master_service(pa_disco_master_t *ctx);
uint8_t pa_disco_master_slave_id(const pa_disco_master_t *ctx);
pa_disco_state_t pa_disco_master_state(const pa_disco_master_t *ctx);
const char *pa_disco_master_state_str(const pa_disco_master_t *ctx);
pa_disco_list_t *pa_disco_master_list(pa_disco_master_t *ctx);
```

### Slave API

```c
void pa_disco_slave_init(pa_disco_slave_dev_t *ctx, pa_modbus_t *pam);
void pa_disco_slave_set_callbacks(pa_disco_slave_dev_t *ctx,
    pa_disco_get_ticks_fn     get_ticks,
    pa_disco_delay_ms_fn      delay_ms,
    pa_disco_flush_fn         flush,
    pa_disco_random_fn        random,
    pa_disco_slave_notify_fn  notify,
    void                     *userdata);
void pa_disco_slave_set_window(pa_disco_slave_dev_t *ctx,
                               uint16_t min_ms, uint16_t max_ms);
void pa_disco_slave_set_serialno(pa_disco_slave_dev_t *ctx,
                                 const uint16_t *serialno);
bool pa_disco_slave_service(pa_disco_slave_dev_t *ctx);
int  pa_disco_slave_read_cb(void *ctx, int reg_index, uint16_t nregs);
int  pa_disco_slave_write_cb(void *ctx, int reg_index, uint16_t nregs);
```

### Register Map API

```c
void pa_disco_register_map_init(pa_disco_register_map_t *map, pa_modbus_t *pam);
void pa_disco_register_map_set_req_cb(pa_disco_register_map_t *map,
                                      pa_disco_req_cb_t cb, void *arg);
void pa_disco_register_map_set_holding(pa_disco_register_map_t *map,
                                       uint16_t *regs, uint16_t start, uint16_t count);
int  pa_disco_register_map_service(pa_disco_register_map_t *map);
uint16_t *pa_disco_register_map_get_ptr(pa_disco_register_map_t *map, uint16_t reg);
uint16_t  pa_disco_register_map_get(pa_disco_register_map_t *map, uint16_t reg);
float     pa_disco_register_map_get_float32(pa_disco_register_map_t *map, uint16_t reg);
void      pa_disco_register_map_put(pa_disco_register_map_t *map, uint16_t reg, uint16_t val);
void      pa_disco_register_map_put_float32(pa_disco_register_map_t *map, uint16_t reg, float val);
```

### Slave List API

```c
pa_disco_slave_t *pa_disco_slave_new(uint8_t slave_id, const uint16_t *serialno, void *arg);
void pa_disco_slave_delete(pa_disco_slave_t *slave);
void pa_disco_list_init(pa_disco_list_t *list);
void pa_disco_list_clear(pa_disco_list_t *list);
int  pa_disco_list_count(const pa_disco_list_t *list);
bool pa_disco_list_insert(pa_disco_list_t *list, pa_disco_slave_t *slave, int index);
bool pa_disco_list_append(pa_disco_list_t *list, pa_disco_slave_t *slave);
pa_disco_slave_t *pa_disco_list_at(const pa_disco_list_t *list, int index);
int  pa_disco_list_find_serialno(const pa_disco_list_t *list, const uint16_t *serialno);
int  pa_disco_list_set_slave_id(pa_disco_list_t *list, int index, uint8_t slave_id);
```

## Usage Examples

### Master: Discover Slaves on an RS-485 Bus

```c
#include "pamodbus-disco.h"
#include "pamodbus.h"

static pa_modbus_t     pam;
static pa_disco_master_t disco;
static uint8_t txbuf[256];
static uint8_t rxbuf[256];

/* Hardware abstraction callbacks */
static uint32_t get_ticks(void *userdata) {
    return (uint32_t)jiffies();  /* Platform-specific tick counter */
}

static void flush(void *userdata) {
    hw_uart_rx_queue_flush(RS485_UART_0);
}

static int trylock(void *userdata) {
    return caribou_mutex_try_lock(&rs485_mutex) ? 0 : -1;
}

static void unlock(void *userdata) {
    caribou_mutex_unlock(&rs485_mutex);
}

/* Notification: called per slave discovered, then with slave==NULL when done */
static void on_slave_discovered(pa_disco_list_t *list,
                                 pa_disco_slave_t *slave, void *userdata) {
    if (slave) {
        printf("Discovered: ID=%d S/N=%04X%04X%04X\n",
               pa_disco_slave_get_id(slave),
               pa_disco_slave_get_serialno(slave)[0],
               pa_disco_slave_get_serialno(slave)[1],
               pa_disco_slave_get_serialno(slave)[2]);
    } else {
        printf("Discovery cycle complete — %d slaves found\n",
               pa_disco_list_count(list));
    }
}

void master_setup(void) {
    /* Initialize pamodbus */
    pa_modbus_init(&pam);
    pa_modbus_set_framer(&pam, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&pam, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&pam, rxbuf, sizeof(rxbuf));
    pa_modbus_set_send_cb(&pam, uart_send, &device);
    pa_modbus_set_recv_cb(&pam, uart_recv, &device);

    /* Set discovery settings */
    pa_disco_settings_t settings = {
        .window_time        = 100,    /* 100ms response window */
        .refresh_period     = 30000,  /* Refresh every 30s */
        .repeat_cycles      = 3,      /* 3 broadcasts with no response = done */
        .window_guard_time  = 50,     /* 50ms guard time */
        .reset_repeat_cycles = 3,     /* 3 reset broadcasts */
        .verify_repeat_cycles = 2,    /* 2 verify attempts */
    };

    /* Initialize discovery master */
    pa_disco_master_init(&disco, &pam);
    pa_disco_master_set_settings(&disco, &settings);
    pa_disco_master_set_callbacks(&disco,
        get_ticks, flush, NULL, NULL, trylock,
        on_slave_discovered, NULL);
}

void master_run(void) {
    pa_disco_master_service(&disco);
}
```

### Slave: Respond to Discovery on an RS-485 Bus

```c
#include "pamodbus-disco.h"
#include "pamodbus.h"

static pa_disco_slave_dev_t   disco;
static pa_disco_register_map_t map;
static uint8_t txbuf[256], rxbuf[256];

/* Hardware abstraction callbacks */
static uint32_t get_ticks(void *userdata) {
    return (uint32_t)jiffies();
}

static void delay_ms(uint32_t ms, void *userdata) {
    msdelay((int)ms);
}

static void flush(void *userdata) {
    hw_uart_rx_queue_flush(RS485_UART_0);
}

static uint32_t random_range(uint32_t min, uint32_t max, void *userdata) {
    return (uint32_t)random((int)min, (int)max);
}

static void on_id_assigned(uint8_t slave_id, void *userdata) {
    printf("Assigned slave ID: %d\n", slave_id);
}

/* Register map request callback: dispatches to slave discovery handlers */
static int req_cb(void *arg) {
    pa_disco_slave_dev_t *d = (pa_disco_slave_dev_t *)arg;
    pa_modbus_t *pam = d->pam;  /* Requires pamodbus-disco-internal.h access */
    uint8_t fn = pa_modbus_slave_function(pam);
    uint16_t reg = pa_modbus_slave_addr(pam);
    uint16_t cnt = pa_modbus_slave_count(pam);

    switch (fn) {
        case 0x03: return pa_disco_slave_read_cb(d, (int)reg, cnt);
        case 0x06:
        case 0x10: return pa_disco_slave_write_cb(d, (int)reg, cnt);
        default:   return 1;
    }
}

void slave_setup(void) {
    uint16_t serialno[3] = {0x1234, 0x5678, 0x9ABC};

    /* Initialize pamodbus */
    pa_modbus_t *pam = &map.pam;
    pa_modbus_init(pam);
    pa_modbus_set_framer(pam, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(pam, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(pam, rxbuf, sizeof(rxbuf));
    pa_modbus_set_slave(pam, 0);              /* Unassigned */
    pa_modbus_set_discovery_addr(pam, 0xFF);  /* Listen on broadcast */
    pa_modbus_set_send_cb(pam, uart_send, &device);
    pa_modbus_set_recv_cb(pam, uart_recv, &device);

    /* Initialize register map */
    pa_disco_register_map_init(&map, pam);

    /* Initialize slave discovery */
    pa_disco_slave_init(&disco, pam);
    pa_disco_slave_set_callbacks(&disco, get_ticks, delay_ms, flush,
                                  random_range, on_id_assigned, NULL);
    pa_disco_slave_set_window(&disco, 10, 100);  /* 10-100ms random delay */
    pa_disco_slave_set_serialno(&disco, serialno);

    /* Wire up the request callback */
    pa_disco_register_map_set_req_cb(&map, req_cb, &disco);
}

void slave_run(void) {
    if (pa_disco_slave_service(&disco)) {
        pa_disco_register_map_service(&map);
    }
}
```

## File Structure

```
pamodbus-disco/
├── include/
│   └── pamodbus-disco.h          # Public API header
├── src/
│   ├── pamodbus-disco-internal.h # Internal structs and helpers
│   ├── pa_disco_master.c         # Master discovery state machine
│   ├── pa_disco_slave.c          # Slave discovery logic
│   ├── pa_disco_list.c           # Slave list management
│   └── pa_disco_register_map.c   # Register map with auto MODBUS handling
├── test/
│   └── test_pamodbus-disco.c     # Unit tests
├── _src.mk                       # Build system integration
├── LICENSE                       # MIT license
└── README.md                     # This file
```

## License

MIT — see LICENSE file for details.