"""OLED status display (0.96" SSD1306, I2C) attached directly to the
Jetson's 40-pin header — not the Arduino.

Wiring: VCC->3.3V or 5V (check your module's rating), GND->GND,
SDA->pin 3, SCL->pin 5 on the 40-pin header.

IMPORTANT — verify before trusting this:
  1. Bus number varies by carrier board. Run `i2cdetect -l` to list
     buses, then `i2cdetect -y -r <bus>` for each candidate until you
     see a device respond. The official Jetson devkit's 40-pin header
     is commonly bus 7 or bus 1 depending on the JetPack/L4T version;
     a third-party GPIO extender (this build uses one) can renumber
     it entirely, so don't assume — check.
  2. Address is almost always 0x3C, occasionally 0x3D on clone boards.
     i2cdetect will show you which.
  Update I2C_BUS / I2C_ADDR below once confirmed.

Fails safe: if the display can't be opened (wrong bus, not wired yet,
permissions), this logs once and every update() call becomes a no-op
instead of crashing the Flask app.
"""

import logging

logger = logging.getLogger(__name__)

I2C_BUS = 7        # CHECK with `i2cdetect -l` — see note above
I2C_ADDR = 0x3C     # CHECK with `i2cdetect -y -r <bus>`
ROTATE = 2          # 0=normal, 1=90° CW, 2=180°, 3=270° CW (portrait: 1 or 3)


class OLEDStatus:
    def __init__(self, bus=I2C_BUS, address=I2C_ADDR, rotate=ROTATE):
        self.ok = False
        self._device = None
        self._font = None
        self._portrait = rotate in (1, 3)
        try:
            from luma.core.interface.serial import i2c
            from luma.core.render import canvas
            from luma.oled.device import ssd1306
            from PIL import ImageFont

            serial = i2c(port=bus, address=address)
            self._device = ssd1306(serial, rotate=rotate)
            self._canvas = canvas

            # Portrait is only ~64px wide — a 12pt font clips fast, so
            # drop to something that actually fits short labels.
            font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
            font_size = 16 if self._portrait else 12
            try:
                self._font = ImageFont.truetype(font_path, font_size)
            except Exception:
                self._font = ImageFont.load_default()
            self.ok = True
        except Exception as e:
            logger.warning("OLED not available (bus=%s addr=0x%02X): %s",
                            bus, address, e)

    def update(self, state="---", dist=None, speed=None, aeb="---",
               ldw="---", link_ok=True, fps=None):
        """speed/fps/link_ok are accepted but no longer shown on the OLED —
        kept in the signature so callers don't need to change. They're
        already visible in the browser UI; the display sticks to the
        4 fields that matter most for a quick glance: state, distance,
        AEB, LDW."""
        if not self.ok:
            return
        try:
            dist_txt = "---" if dist is None or dist >= 999 else f"{dist}cm"

            lines = [
                f"St: {state}",
                f"D: {dist_txt}",
                f"AEB: {aeb}",
                f"LDW: {ldw}",
            ]
            line_height = 30 if self._portrait else 16

            with self._canvas(self._device) as draw:
                y = 0
                for line in lines:
                    draw.text((0, y), line, font=self._font, fill="white")
                    y += line_height
        except Exception as e:
            logger.warning("OLED update failed, disabling: %s", e)
            self.ok = False

    def close(self):
        if self.ok and self._device is not None:
            try:
                self._device.cleanup()
            except Exception:
                pass
