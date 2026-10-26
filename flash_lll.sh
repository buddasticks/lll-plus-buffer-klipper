#!/bin/bash
# scripts/flash_lll.sh
# Builds Klipper with buffer.c for the STM32F072 buffer board and flashes it
# over USB via Katapult. Usage:
#   ./flash_lll.sh /dev/serial/by-id/usb-Klipper_stm32f072xb_XXXX-if00
# Run on the Pi, as the biqu user.

set -e

SERIAL="$1"
KLIPPER="$HOME/klipper"
SRC_BUFFER="$HOME/lll_buffer_project/firmware/buffer.c"
KLIPPY_ENV="$HOME/klippy-env/bin/python3"
FLASHTOOL="$HOME/katapult/scripts/flashtool.py"

if [ -z "$SERIAL" ]; then
    echo "Usage: $0 /dev/serial/by-id/<stm32f072 serial>"
    exit 1
fi

if [ ! -f "$SRC_BUFFER" ]; then
    echo "Missing $SRC_BUFFER"
    exit 1
fi

cd "$KLIPPER"

# 1. Keep the sv08 build config so the other MCUs can be rebuilt later
cp .config .config.sv08

# 2. Copy the buffer source into the tree
cp "$SRC_BUFFER" src/buffer.c

# 3. Add buffer.c to the build if it isn't there yet
grep -q "buffer.c" src/Makefile || \
    echo 'src-$(CONFIG_MACH_STM32F072) += buffer.c' >> src/Makefile

# 4. Build config for the F072 buffer board. Opens menuconfig: set the values
#    listed in the README, then save and exit.
echo "Opening menuconfig. Set: STM32 / STM32F072 / 8 KiB bootloader / 8 MHz / USB PA11/PA12"
make menuconfig

# 5. Build
make clean
make

# 6. The dictionary must contain the new commands, or buffer.c did not build
COUNT=$(grep -c "lll_autoload" out/klipper.dict || true)
if [ "$COUNT" -eq 0 ]; then
    echo "ERROR: lll_autoload not in klipper.dict. buffer.c was not built."
    cp .config.sv08 .config
    exit 1
fi
echo "Dictionary OK ($COUNT lll_autoload entries)"

# 7. Flash
"$KLIPPY_ENV" "$FLASHTOOL" -f out/klipper.bin -d "$SERIAL"

# 8. Restore the sv08 build config
cp .config.sv08 .config
echo "Flash done. Send FIRMWARE_RESTART in Mainsail."
