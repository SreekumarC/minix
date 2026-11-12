#!/usr/bin/env python3
"""Standalone OLED test — run this BEFORE wiring the display into app.py.

Isolates the hardware/library/bus/address from everything else, so if
this fails you know it's the display setup, not a bug somewhere in the
Flask app.

Usage:
    python3 test_oled.py            # uses I2C_BUS/I2C_ADDR from oled_display.py
    python3 test_oled.py 7 0x3c     # override bus/address for this run only
"""

import sys
import time

import oled_display


def main():
    bus = oled_display.I2C_BUS
    addr = oled_display.I2C_ADDR
    rotate = oled_display.ROTATE
    if len(sys.argv) >= 3:
        bus = int(sys.argv[1])
        addr = int(sys.argv[2], 0)  # accepts 0x3c or 60
    if len(sys.argv) >= 4:
        rotate = int(sys.argv[3])

    print(f"Trying I2C bus={bus}, address=0x{addr:02X}, rotate={rotate} ...")
    oled = oled_display.OLEDStatus(bus=bus, address=addr, rotate=rotate)

    if not oled.ok:
        print("FAILED to open the display. Things to check:")
        print("  1. Run `i2cdetect -l` to list buses on this Jetson.")
        print("  2. Run `sudo i2cdetect -y -r <bus>` for each one until")
        print("     you see a device respond (usually at 0x3c, sometimes")
        print("     0x3d) — that confirms wiring + the real bus number.")
        print("  3. Re-run this script with that bus/address, e.g.:")
        print("       python3 test_oled.py 1 0x3d")
        print("  4. If nothing responds on any bus: check VCC/GND aren't")
        print("     swapped, and that SDA/SCL aren't swapped with each other.")
        sys.exit(1)

    print("Display opened OK. Running through test patterns...")

    print("  [1/4] Static text")
    oled.update(state="TEST", dist=42, speed=5, aeb="clear",
                ldw="in lane", link_ok=True, fps=30)
    time.sleep(2)

    print("  [2/4] Different values (confirms it actually redraws, not stuck)")
    oled.update(state="FWD", dist=15, speed=9, aeb="braking",
                ldw="departure", link_ok=False, fps=12)
    time.sleep(2)

    print("  [3/4] Live counter for 5 seconds (watch for smooth updates, not lag)")
    for i in range(50):
        oled.update(state="MRM!", dist=999 - i, speed=i % 10,
                    aeb="caution", ldw="off", link_ok=True, fps=25 + (i % 5))
        time.sleep(0.1)

    print("  [4/4] Back to idle")
    oled.update(state="STOP", dist=None, speed=0, aeb="clear",
                ldw="---", link_ok=True, fps=0)

    print("Done. If the screen showed all four steps clearly, the display")
    print("is good to go — no changes needed in app.py.")
    oled.close()


if __name__ == "__main__":
    main()
