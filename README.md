# LLL Buffer Plus, Klipper port (T2)

Klipper MCU firmware for the Mellow LLL Buffer Plus (STM32F072CB), with
auto-load and host runout/fault reporting. Based on Fly3DTeam buffer v2.0.9.

## Contents

| File | Install to | Purpose |
|------|-----------|---------|
| `firmware/buffer.c` | `~/klipper/src/buffer.c` (copied by the script) | MCU firmware |
| `klipper/extras/lll_autoload.py` | `~/klipper/klippy/extras/lll_autoload.py` | Sends auto-load settings at startup |
| `config/printer_lll_t2.cfg` | paste into `printer.cfg` or `[include]` it | MCU, sensors, auto-load, fault macro |
| `config/print_start_precheck.cfg` | paste into `macros/print_start.cfg` | Stops a print if the buffer is not ready |
| `scripts/flash_lll.sh` | run on the Pi | Build and flash the buffer MCU |

## Install

1. Copy the host extra:

```bash
cp ~/lll_buffer_project/klipper/extras/lll_autoload.py ~/klipper/klippy/extras/
```

2. Edit `config/printer_lll_t2.cfg` and set the serial path to the board's ID
   from `ls /dev/serial/by-id/`. Paste the file's contents into `printer.cfg`,
   or include it. Delete any `LLL_T0` sections.

3. Paste `config/print_start_precheck.cfg` into `macros/print_start.cfg`, as
   described in its header.

4. Flash the buffer board:

```bash
chmod +x ~/lll_buffer_project/scripts/flash_lll.sh
~/lll_buffer_project/scripts/flash_lll.sh /dev/serial/by-id/usb-Klipper_stm32f072xb_XXXX-if00
```

   In `make menuconfig`, set:
   - Micro-controller Architecture: STMicroelectronics STM32
   - Processor model: STM32F072
   - Bootloader offset: 8 KiB bootloader
   - Clock Reference: 8 MHz crystal
   - Communication interface: USB (on PA11/PA12)

5. In Mainsail, send `FIRMWARE_RESTART`.

## Check

```
QUERY_FILAMENT_SENSOR SENSOR=lll_t2_runout
QUERY_FILAMENT_SENSOR SENSOR=lll_t2_fault
```

Both read "detected" with filament loaded and the buffer healthy.

## Settings

| Setting | Where | Notes |
|---------|-------|-------|
| `auto_load` | `[lll_autoload T2]` | True feeds on insert |
| `load_length` | `[lll_autoload T2]` | mm, 1 to 500 |
| `load_speed` | `[lll_autoload T2]` | mm/s, calibrate (see below) |
| `UART_BAUD` | `firmware/buffer.c` | 9600 default, reflash to change |

Changing the `printer.cfg` values needs `RESTART`, not a reflash.

## Calibrate the feed speed

1. Mark the filament at the entrance.
2. Hold KEY2 for exactly 10 seconds.
3. Measure the distance moved in mm and divide by 10.
4. Set `load_speed` to that value and send `RESTART`.

## Test

- Empty buffer, insert filament: it should feed `load_length` mm, then the
  halls take over and feed on to the head.
- Manual feed (KEY2) and back (KEY1): the motor should run smoothly, with the
  correct direction.
- Read the events:

```bash
grep "lll_al" ~/printer_data/logs/klippy.log | tail -n 20
```

Reason codes: `0` start, `1` length reached, `2` filament removed,
`3` fault, `4` manual press cancelled auto-load.

## Load indicator

After flashing, check the MCU numbers at idle:

```bash
curl -s "http://localhost:7125/printer/objects/query?mcu%20LLL_T2" | python3 -m json.tool | grep -E "awake|stddev"
```

The stddev should be far below the earlier ~2.5 ms. Run a feed, then check
again to confirm the motor still behaves.

## Known gaps

Not ported from Mellow v2.0.9: TPU modes, quick mode, TMC CRC and driver
status checks, MDM blockage detection, the ERR LED (PA15), and the dual-key
mode switch. The manual PB12 button burst issue is still open.

## Revert

The stock firmware can be restored from the DFU backup
(`lll_backup_flash.bin`) taken before the first flash, or from the Mellow
v2.0.9 build (`dev-2.x.x` branch, needs the Fly bootloader).

## Warning

This is experimental, unverified community firmware. Flashing can brick the
board. Keep the DFU backup before you flash.
