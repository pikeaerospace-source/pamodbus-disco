# pamodbus-disco — Unit Test Suite

This directory contains the unit test suite for the **pamodbus-disco** library — a lightweight, portable MODBUS automatic slave ID discovery protocol library built on top of [pamodbus](https://github.com/8bitgeek/pamodbus).

## Test File

- **`test_pamodbus-disco.c`** — The complete test suite (218 tests)

## Test Architecture

The test suite follows the same patterns as `_src/pamodbus/test/test_pamodbus.c`:

- **No external test framework** — plain C with `TEST_ASSERT` macros, `tests_passed`/`tests_failed` counters
- **Direct byte-level MODBUS frame construction** — raw RTU frames with correct CRC-16 are built and fed to the parser
- **Mock callbacks** — all hardware/OS dependency callbacks (send, recv, ticks, flush, mutex, random, delay, notify) are implemented as controllable mocks
- **Deterministic timing** — a mock tick counter is advanced manually between state machine calls, eliminating timing dependencies
- **Standalone host executable** — compiled with `gcc` on the host, no embedded hardware required

## Test Coverage

### 1. Slave List Tests (`pa_disco_list.c`) — 18 tests

| Test | Description |
|------|-------------|
| `pa_disco_list_init` | Empty list has count=0 |
| `pa_disco_slave_new` / `pa_disco_slave_delete` | Create and destroy slave records |
| `pa_disco_slave_get_id` / `pa_disco_slave_get_serialno` / `pa_disco_slave_get_arg` | Accessor round-trip |
| `pa_disco_list_append` | Append 3 slaves, verify order |
| `pa_disco_list_at` out-of-range | Negative index, index beyond end |
| `pa_disco_list_insert` | Insert at beginning, middle, end |
| `pa_disco_list_insert` invalid index | Negative, beyond end |
| `pa_disco_list_find_serialno` | Find existing, non-existent, empty list |
| `pa_disco_list_set_slave_id` | Valid and invalid index |
| `pa_disco_list_clear` | Clear, double clear, re-append after clear |

### 2. Register Map Tests (`pa_disco_register_map.c`) — 18 tests

| Test | Description |
|------|-------------|
| Init | All registers zero after init |
| Get/put single register | Round-trip verification |
| Get pointer | Valid, out-of-range |
| Float32 conversion | Positive, zero, negative, out-of-range |
| Set holding | Copy from external array, start address change |
| MODBUS FC03 read request | Feed RTU frame, verify auto-response |
| MODBUS FC06 write single | Feed RTU frame, verify register written |
| MODBUS FC10 write multiple | Feed RTU frame, verify registers written |
| Request callback suppression | Callback returns 0, response suppressed |

### 3. Slave Discovery Tests (`pa_disco_slave.c`) — 18 tests

| Test | Description |
|------|-------------|
| Init | Context zeroed, pam pointer set |
| Set callbacks | All 6 callbacks stored correctly |
| Set window | Min/max stored correctly |
| Set serial number | 6 uint16 values copied |
| Service (no window wait) | Returns true |
| Service (in window wait) | Returns false, flush called |
| Service (window expires) | Window wait cleared, returns false |
| Read callback — discovery trigger | Window wait entered, slave=0 |
| Read callback — partial read | Returns 0 (suppress) |
| Read callback — non-discovery | Returns 1 (allow) |
| Read callback — assigned slave | Returns 0 (suppress), no window |
| Write callback — reset | All-zero serialno, slave ID=0, notify |
| Write callback — assign matching | Serialno matches, ID updated, notify |
| Write callback — same ID | No notify when ID unchanged |
| Write callback — non-matching | No change, no notify |
| Write callback — partial write | Returns 1 (allow default) |
| Write callback — non-discovery | Returns 1 (allow) |
| Random delay during discovery | delay_ms callback invoked |

### 4. Master Discovery Tests (`pa_disco_master.c`) — 48 tests

| Test | Description |
|------|-------------|
| Init | State=IDLE, reset flag set |
| Set settings | All 6 fields stored |
| Set callbacks | All 7 callbacks stored |
| State strings | All 9 states + unknown |
| Slave ID / list accessors | Initial values |
| Reset sets flag | |
| **State Machine Transitions** | |
| IDLE → RESET_START | Reset flag + trylock success |
| IDLE stays IDLE | Trylock fails |
| RESET_START → RESET_REPEAT | List cleared |
| RESET_REPEAT → RESET_WAIT | Reset broadcast sent |
| RESET_WAIT → RESET_REPEAT | Guard time expired |
| RESET_WAIT stays in RESET_WAIT | Guard time not expired |
| RESET_REPEAT → START | Cycles exhausted |
| START → REPEAT | Slave ID reset to 0 |
| REPEAT → WAIT | Broadcast query sent (FC03, addr 0xFF) |
| WAIT → REPEAT | No response, window expires |
| WAIT stays in WAIT | Window not expired |
| WAIT → VERIFY | Response received |
| VERIFY → REPEAT | Verify succeeds, slave added |
| REPEAT → FINISH | Cycles exhausted |
| FINISH → IDLE | Notify with NULL, unlock |
| Full cycle (no slaves) | Complete cycle, 0 slaves |
| Full cycle (one slave) | Complete cycle, 1 slave discovered |

### 5. Integration Test — 2 tests

| Test | Description |
|------|-------------|
| Full discovery cycle | End-to-end: reset → broadcast → receive → verify → repeat → finish → idle |
| Cycle completion | Verifies the cycle terminates without crashing |

## Build & Run

### Prerequisites

- GCC (or any C99-compatible compiler)
- pamodbus library source files (in `_src/pamodbus/`)

### Compilation

```bash
gcc -o test_pamodbus-disco \
    -I_src/pamodbus/include \
    -I_src/pamodbus/src \
    -I_src/pamodbus-disco/include \
    -I_src/pamodbus-disco/src \
    _src/pamodbus-disco/test/test_pamodbus-disco.c \
    _src/pamodbus-disco/src/pa_disco_master.c \
    _src/pamodbus-disco/src/pa_disco_slave.c \
    _src/pamodbus-disco/src/pa_disco_list.c \
    _src/pamodbus-disco/src/pa_disco_register_map.c \
    _src/pamodbus/src/pamodbus.c \
    _src/pamodbus/src/pdu.c \
    _src/pamodbus/src/framer_rtu.c \
    _src/pamodbus/src/framer_tcp.c \
    _src/pamodbus/src/crc16.c \
    -lm
```

### Run

```bash
./test_pamodbus-disco
```

### Expected Output

```
pamodbus-disco Unit Tests
========================
=== Slave List Tests ===
  PASS: Empty list after init
  ...
=== Register Map Tests ===
  ...
=== Slave Discovery Tests ===
  ...
=== Master Discovery Tests ===
  ...
=== Integration Tests ===
  ...
========================
Results: 218 passed, 0 failed
```

Exit code `0` indicates all tests passed. Exit code `1` indicates one or more failures.

## Mock Architecture

The test suite uses a shared `test_mock_t` struct that provides controllable mock implementations for all hardware/OS dependency callbacks:

| Callback | Mock Behavior |
|----------|---------------|
| `mock_send` | Captures TX bytes into `tx_buf`/`tx_len` |
| `mock_recv` | Returns pre-staged response from `rx_buf`/`rx_len` when `rx_pending` is true; returns 0 (timeout) otherwise |
| `mock_get_ticks` | Returns current value of `ticks` counter |
| `mock_flush` | Sets `flush_called` flag |
| `mock_lock` / `mock_unlock` | Tracks nesting count in `mutex_locked` |
| `mock_trylock` | Returns `mutex_trylock_result` (0 = success) |
| `mock_random` | Returns deterministic `random_value` |
| `mock_delay` | Advances `ticks` by the delay amount |
| `mock_slave_notify` | Increments `slave_notify_count`, records `last_assigned_slave_id` |
| `mock_master_notify` | Increments count on slave discovery, sets `discovery_done` on cycle complete |

### RTU Frame Helpers

Two helper functions simplify building valid MODBUS RTU frames for test scenarios:

- **`mock_stage_read_resp()`** — Builds a complete read holding registers response with correct CRC-16
- **`mock_stage_frame()`** — Builds a generic RTU frame with arbitrary data

These are used by tests to stage responses that the mock recv callback returns when `pa_modbus_recv()` is called.

## Key Implementation Details

### Master State Machine Timing

The master discovery state machine uses callbacks for timing (`get_ticks`). In tests, the mock tick counter is advanced manually between `pa_disco_master_service()` calls to trigger time-based state transitions (window expiration, guard time, refresh period).

### VERIFY → REPEAT Transition

The `do_state_verify()` function requires two service calls to transition from VERIFY to REPEAT:
1. **First call**: `verify_slave()` succeeds, slave is added to the list, `verify_repeat` is set to 0, state stays VERIFY
2. **Second call**: `verify_repeat` decrements to -1, `else` branch transitions to REPEAT

This is a property of the current library implementation and is accounted for in the tests.

### Staged Response Consumption

The `mock_recv` callback consumes the staged response on the first call (`rx_pending` is set to `false`). This means tests must stage a new response before each `pa_modbus_recv()` call that needs to receive data.