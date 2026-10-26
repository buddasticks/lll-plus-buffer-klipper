// ~/klipper/src/buffer.c  (ADDED file, survives git pull)
// LLL Buffer Plus, Klipper MCU (STM32F072CB)
// Logic ported from Fly3DTeam buffer v2.0.9 (lib/buffer/buffer.cpp).
// Adds USB reporting to the host: runout, forward timeout, manual pause.
// Adds auto-load: feeds a set length in mm after a stable filament insert.
// Auto-load settings come from printer.cfg via the lll_autoload config command.
// PB14 stays the summary fault line (active low, forward timeout, as in v2.0.9).
// Not ported: TMC CRC/driver-status checks, MDM blockage detection, dual-key mode switch.

#include "autoconf.h"      // CONFIG_CLOCK_FREQ
#include "board/gpio.h"    // gpio_in_setup / gpio_out_setup
#include "board/misc.h"    // timer_read_time
#include "command.h"       // sendf, DECL_COMMAND
#include "sched.h"         // DECL_INIT / DECL_TASK

// ------------------------------------------------------------------
// Pins (GPIO(PORT,NUM) = (PORT-'A')*16 + NUM)
// ------------------------------------------------------------------
#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))

#define HALL1_PIN     GPIO('B', 2)   // light sensor 3 -> Back
#define HALL2_PIN     GPIO('B', 3)   // light sensor 2 -> Stop
#define HALL3_PIN     GPIO('B', 4)   // light sensor 1 -> Forward
#define FIL_PIN       GPIO('B', 7)   // filament switch, 1 = no filament
#define KEY1_PIN      GPIO('B', 13)  // KEY1 back
#define KEY2_PIN      GPIO('B', 12)  // KEY2 forward
#define FRONT_SIG_PIN GPIO('B', 5)   // mainboard forward, active low
#define BACK_SIG_PIN  GPIO('B', 6)   // mainboard back, active low
#define EXT1_PIN      GPIO('A', 2)   // EXTENSION_PIN1 (inform)
#define EXT2_PIN      GPIO('A', 3)   // EXTENSION_PIN2 (inform)
#define EN_PIN        GPIO('A', 6)   // TMC enable, active low
#define UART_PIN      GPIO('B', 1)   // TMC2208 single-wire UART
#define DUANLIAO_PIN  GPIO('B', 15)  // runout output
#define FAULT_PIN     GPIO('B', 14)  // summary fault, active low

// ------------------------------------------------------------------
// UART baud. 9600 matches the Mellow firmware timing. 115200 is faster
// and gives less CPU time per write, but it is untested for motor
// behaviour on this board. Change to 115200 only after a feed test.
// ------------------------------------------------------------------
#define UART_BAUD        9600
#define BIT_TICKS        (CONFIG_CLOCK_FREQ / UART_BAUD)
#define GAP_TICKS        (CONFIG_CLOCK_FREQ / 5000U)   // 200 us between datagrams

// ------------------------------------------------------------------
// Parameters (v2.0.9 defaults)
// ------------------------------------------------------------------
#define SPEED_RPM        260
#define MICROSTEPS_DIV   64
#define VACTUAL_MAG      ((int32_t)((int64_t)SPEED_RPM * MICROSTEPS_DIV * 200 / 60 * 1000 / 715))
#define FEED_TIMEOUT_MS  60000   // forward timeout
#define RUNOUT_DELAY_MS  10000   // FILAMENT_RUNOUT_DELAY_MS
#define LONG_PRESS_MS    500
#define CLICK_MAX_MS     500
#define INFORM_MS        3000
#define AUTOLOAD_MAX_MS  60000   // cap auto-load time, matches forward timeout
#define AL_DEBOUNCE_MS   150     // entrance switch must hold a new state this long
#define AL_REARM_MS      500     // entrance switch must read empty this long to re-arm

#define MS_TICKS(ms)     ((uint32_t)(ms) * (CONFIG_CLOCK_FREQ / 1000))

// ------------------------------------------------------------------
// TMC2208 registers (write-only use)
// ------------------------------------------------------------------
#define TMC_GCONF        0x00
#define TMC_IHOLD_IRUN   0x10
#define TMC_VACTUAL      0x22
#define TMC_CHOPCONF     0x6C
#define TMC_PWMCONF      0x70
#define TMC_WRITE_FLAG   0x80
#define TMC_SLAVE_ADDR   0x00
#define TMC_SYNC         0x05

#define GCONF_VAL        0x000001C4UL
#define GCONF_SHAFT      0x00000008UL
#define CHOPCONF_VAL     0x12020055UL
#define IHOLD_IRUN_VAL   0x00010F07UL
#define PWMCONF_VAL      0xC10D0024UL

// ------------------------------------------------------------------
// State
// ------------------------------------------------------------------
enum { FAULT_NONE = 0, FAULT_TIMEOUT = 1, FAULT_PAUSE = 2 };
enum { ST_STOP = 0, ST_FORWARD, ST_BACK };

struct key_state {
    uint8_t prev;          // 1 = released, 0 = pressed
    uint8_t clicks;
    uint8_t pending;
    uint32_t press_t;
    uint32_t release_t;
};

static struct gpio_in hall1, hall2, hall3, fil, key1, key2, front_sig, back_sig;
static struct gpio_out en_out, uart_out, ext1, ext2, duanliao_out, fault_out;
static struct key_state keys[2];        // [0] back, [1] forward
static uint8_t last_state = ST_STOP;
static uint8_t tmc_ready = 0;
static uint8_t fault = FAULT_NONE;
static uint8_t out_state = 0;           // DUANLIAO_OUT_STATE, v2.0.9 default
static uint8_t filament_had = 0;
static uint8_t runout_confirmed = 0;
static uint8_t nofil_timing = 0;
static uint32_t nofil_start = 0;
static uint8_t feeding = 0;
static uint32_t feed_start = 0;
static uint8_t inform_on = 0;
static uint32_t inform_start = 0;
static uint8_t reported_fault = 0xff;
static uint8_t reported_filament = 0xff;

// auto-load settings (set by the lll_autoload config command)
static uint8_t al_enable = 0;           // default off until the host sends config
static uint32_t al_length_mm = 100;
static uint32_t al_speed_x10 = 331;     // 0.1 mm/s units, 331 = 33.1 mm/s

// auto-load runtime state
static uint8_t al_init = 0;             // 0 = sample the switch on the next task
static uint8_t al_raw_last = 0;         // last raw reading, 1 = filament present
static uint8_t al_stable = 0;           // debounced reading
static uint32_t al_raw_t = 0;           // when the raw reading last changed
static uint8_t al_armed = 1;            // 1 = a stable insert may start a feed
static uint32_t al_absent_t = 0;        // when the switch last read empty
static uint8_t autoload_active = 0;
static uint32_t autoload_end = 0;

// ------------------------------------------------------------------
// TMC2208 UART (bit-bang)
// ------------------------------------------------------------------
static void
bit_delay(void)
{
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < BIT_TICKS)
        ;
}

static void
uart_send_byte(uint8_t b)
{
    gpio_out_write(uart_out, 0);
    bit_delay();
    for (uint8_t i = 0; i < 8; i++) {
        gpio_out_write(uart_out, (b >> i) & 0x01);
        bit_delay();
    }
    gpio_out_write(uart_out, 1);
    bit_delay();
}

// Short idle gap between datagrams. Replaces the old 2 ms blocking delays.
static void
uart_gap(void)
{
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < GAP_TICKS)
        ;
}

static uint8_t
tmc_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t cur = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (cur & 0x01))
                crc = (crc << 1) ^ 0x07;
            else
                crc = crc << 1;
            cur >>= 1;
        }
    }
    return crc;
}

static void
tmc_write(uint8_t reg, uint32_t val)
{
    uint8_t dg[8];
    dg[0] = TMC_SYNC;
    dg[1] = TMC_SLAVE_ADDR;
    dg[2] = reg | TMC_WRITE_FLAG;
    dg[3] = (val >> 24) & 0xFF;
    dg[4] = (val >> 16) & 0xFF;
    dg[5] = (val >> 8) & 0xFF;
    dg[6] = val & 0xFF;
    dg[7] = tmc_crc(dg, 7);
    for (uint8_t i = 0; i < 8; i++)
        uart_send_byte(dg[i]);
    uart_gap();
}

static void
tmc_delay_ms(uint32_t ms)
{
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < MS_TICKS(ms))
        ;
}

// Runs on first task call, so 24 V must be present before USB connects.
static void
tmc_configure(void)
{
    tmc_delay_ms(10);
    tmc_write(TMC_GCONF, GCONF_VAL);
    tmc_write(TMC_CHOPCONF, CHOPCONF_VAL);
    tmc_write(TMC_IHOLD_IRUN, IHOLD_IRUN_VAL);
    tmc_write(TMC_PWMCONF, PWMCONF_VAL);
    tmc_write(TMC_VACTUAL, 0);
    last_state = ST_STOP;
}

// Shaft bit sets direction, VACTUAL is applied after the direction change.
static void
apply_state(uint8_t state)
{
    if (state == last_state)
        return;
    switch (state) {
    case ST_FORWARD:
        gpio_out_write(en_out, 0);
        if (last_state == ST_BACK)
            tmc_write(TMC_VACTUAL, 0);
        tmc_write(TMC_GCONF, GCONF_VAL | GCONF_SHAFT);
        tmc_write(TMC_VACTUAL, (uint32_t)VACTUAL_MAG);
        break;
    case ST_BACK:
        gpio_out_write(en_out, 0);
        if (last_state == ST_FORWARD)
            tmc_write(TMC_VACTUAL, 0);
        tmc_write(TMC_GCONF, GCONF_VAL);
        tmc_write(TMC_VACTUAL, (uint32_t)VACTUAL_MAG);
        break;
    default:
        tmc_write(TMC_VACTUAL, 0);
        gpio_out_write(en_out, 1);
        break;
    }
    last_state = state;
}

// ------------------------------------------------------------------
// Keys: click counting and long press, polled (no interrupts)
// ------------------------------------------------------------------
// Returns the decided click count once the click window closes, else 0.
static uint8_t
key_poll(struct key_state *k, uint8_t val, uint32_t now)
{
    if (k->prev && !val) {
        k->press_t = now;
    } else if (!k->prev && val) {
        if (now - k->press_t <= MS_TICKS(CLICK_MAX_MS)) {
            k->clicks++;
            k->release_t = now;
            k->pending = 1;
        } else {
            k->clicks = 0;      // long press, not a click
        }
    }
    k->prev = val;
    if (k->pending && now - k->release_t >= MS_TICKS(CLICK_MAX_MS)) {
        uint8_t n = k->clicks;
        k->clicks = 0;
        k->pending = 0;
        return n;
    }
    return 0;
}

static uint8_t
key_held(const struct key_state *k, uint32_t now)
{
    return !k->prev && (now - k->press_t >= MS_TICKS(LONG_PRESS_MS));
}

// ------------------------------------------------------------------
// Main logic, ported from motor_control() in buffer.cpp
// ------------------------------------------------------------------
static void
buffer_logic(uint32_t now)
{
    // 1. Clicks: single = clear fault and inform, double = pause
    uint8_t n = key_poll(&keys[0], gpio_in_read(key1), now);
    if (n == 1) {
        fault = FAULT_NONE;
        gpio_out_write(ext2, 0);
        inform_on = 1; inform_start = now;
    } else if (n >= 2) {
        fault = FAULT_PAUSE;
    }

    n = key_poll(&keys[1], gpio_in_read(key2), now);
    if (n == 1) {
        fault = FAULT_NONE;
        gpio_out_write(ext1, 1);
        inform_on = 1; inform_start = now;
    } else if (n >= 2) {
        fault = FAULT_PAUSE;
    }

    if (inform_on && now - inform_start >= MS_TICKS(INFORM_MS)) {
        inform_on = 0;
        gpio_out_write(ext2, 1);
        gpio_out_write(ext1, 0);
    }

    // 2. Long press or mainboard signal overrides the halls (back has priority)
    uint8_t manual = 0;
    if (key_held(&keys[0], now) || !gpio_in_read(back_sig))
        manual = ST_BACK;
    else if (key_held(&keys[1], now) || !gpio_in_read(front_sig))
        manual = ST_FORWARD;

    if (manual) {
        fault = FAULT_NONE;     // long press clears timeout and pause, as v2.0.9
        feeding = 0;
        if (autoload_active) {
            autoload_active = 0;
            sendf("lll_al kind=%c reason=%c", 'e', '4');   // manual press cancelled auto-load
        }
        apply_state(manual);
        return;
    }

    // 3. Filament switch: 1 = no filament. Runout after 10 s, immediate if never loaded.
    if (gpio_in_read(fil)) {
        if (!filament_had) {
            runout_confirmed = 1;
        } else {
            if (!nofil_timing) { nofil_timing = 1; nofil_start = now; }
            if (now - nofil_start >= MS_TICKS(RUNOUT_DELAY_MS))
                runout_confirmed = 1;
        }
    } else {
        filament_had = 1;
        nofil_timing = 0;
        runout_confirmed = 0;
    }
    gpio_out_write(duanliao_out, runout_confirmed ? out_state : !out_state);

    // 4. Auto-load: feed al_length_mm after a stable filament insert
    if (al_enable) {
        uint8_t raw = !gpio_in_read(fil);

        if (!al_init) {
            al_init = 1;
            al_raw_last = raw;
            al_stable = raw;
            al_raw_t = now;
            al_absent_t = now;
            al_armed = raw ? 0 : 1;     // filament present at boot: no feed
        }

        // debounce the entrance switch
        if (raw != al_raw_last) {
            al_raw_last = raw;
            al_raw_t = now;
        } else if (raw != al_stable && now - al_raw_t >= MS_TICKS(AL_DEBOUNCE_MS)) {
            al_stable = raw;
            if (!al_stable)
                al_absent_t = now;
        }

        // re-arm only after the switch has read empty for a while
        if (!al_stable && !al_armed && now - al_absent_t >= MS_TICKS(AL_REARM_MS))
            al_armed = 1;

        if (autoload_active) {
            uint8_t reason = 0;
            if (!al_stable)
                reason = 2;                              // filament removed
            else if (fault != FAULT_NONE)
                reason = 3;                              // fault
            else if ((int32_t)(now - autoload_end) >= 0)
                reason = 1;                              // length reached
            if (reason) {
                autoload_active = 0;
                feeding = 0;
                apply_state(ST_STOP);
                sendf("lll_al kind=%c reason=%c", 'e', '0' + reason);
                return;
            }
            apply_state(ST_FORWARD);
            return;
        }

        if (al_stable && al_armed && fault == FAULT_NONE) {
            // time = length / speed, speed in 0.1 mm/s, so ms = length * 10000 / speed_x10
            uint32_t al_ms = (al_length_mm * 10000UL) / al_speed_x10;
            if (al_ms > AUTOLOAD_MAX_MS)
                al_ms = AUTOLOAD_MAX_MS;
            autoload_active = 1;
            al_armed = 0;
            autoload_end = now + MS_TICKS(al_ms);
            sendf("lll_al kind=%c reason=%c", 's', '0');
            apply_state(ST_FORWARD);
            return;
        }
    }

    // 5. Stop on runout or any fault
    if (runout_confirmed || fault != FAULT_NONE) {
        apply_state(ST_STOP);
        feeding = 0;
        return;
    }

    // 6. Halls: stop position first, then forward, then back
    uint8_t state;
    if (gpio_in_read(hall3))      state = ST_FORWARD;
    else if (gpio_in_read(hall2)) state = ST_STOP;
    else if (gpio_in_read(hall1)) state = ST_BACK;
    else                          state = last_state;

    // 7. Forward timeout: continuous feed longer than 60 s
    if (state == ST_FORWARD) {
        if (!feeding) {
            feeding = 1;
            feed_start = now;
        } else if (now - feed_start > MS_TICKS(FEED_TIMEOUT_MS)) {
            fault = FAULT_TIMEOUT;
            apply_state(ST_STOP);
            feeding = 0;
            return;
        }
    } else {
        feeding = 0;
    }

    apply_state(state);
}

// ------------------------------------------------------------------
// Host reporting: sent only when fault or filament state changes
// ------------------------------------------------------------------
static void
report_status(void)
{
    uint8_t filament = runout_confirmed ? 0 : 1;
    if (fault == reported_fault && filament == reported_filament)
        return;
    reported_fault = fault;
    reported_filament = filament;
    sendf("lll_status fault=%c filament=%c", fault, filament);
}

// Host sends this on connect so it gets the current state immediately.
void
command_lll_query(uint32_t *args)
{
    (void)args;
    reported_fault = 0xff;
    reported_filament = 0xff;
    report_status();
}
DECL_COMMAND(command_lll_query, "lll_query");

// Sent by the host from printer.cfg (lll_autoload config command).
// enable: 0 or 1, length_mm: feed length, speed_x10: feed speed in 0.1 mm/s.
void
command_lll_autoload(uint32_t *args)
{
    al_enable = args[0] ? 1 : 0;
    al_length_mm = args[1];
    al_speed_x10 = args[2] ? args[2] : 1;   // avoid divide by zero
    autoload_active = 0;
    al_init = 0;                            // resample the switch on the next task
}
DECL_COMMAND(command_lll_autoload,
    "lll_autoload enable=%c length_mm=%u speed_x10=%u");

// ------------------------------------------------------------------
// Init and task
// ------------------------------------------------------------------
void
buffer_init(void)
{
    hall1 = gpio_in_setup(HALL1_PIN, 0);
    hall2 = gpio_in_setup(HALL2_PIN, 0);
    hall3 = gpio_in_setup(HALL3_PIN, 0);
    fil   = gpio_in_setup(FIL_PIN, 0);
    key1  = gpio_in_setup(KEY1_PIN, 0);
    key2  = gpio_in_setup(KEY2_PIN, 0);
    front_sig = gpio_in_setup(FRONT_SIG_PIN, 1);
    back_sig  = gpio_in_setup(BACK_SIG_PIN, 1);

    en_out       = gpio_out_setup(EN_PIN, 1);
    uart_out     = gpio_out_setup(UART_PIN, 1);
    ext1         = gpio_out_setup(EXT1_PIN, 0);
    ext2         = gpio_out_setup(EXT2_PIN, 1);
    duanliao_out = gpio_out_setup(DUANLIAO_PIN, !out_state);
    fault_out    = gpio_out_setup(FAULT_PIN, 1);

    keys[0].prev = 1;
    keys[1].prev = 1;
}
DECL_INIT(buffer_init);

void
buffer_task(void)
{
    if (!tmc_ready) {
        tmc_configure();
        tmc_ready = 1;
        return;
    }
    buffer_logic(timer_read_time());
    gpio_out_write(fault_out, fault == FAULT_TIMEOUT ? 0 : 1);
    report_status();
}
DECL_TASK(buffer_task);
