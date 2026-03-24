# ssd1306-lite
Minimal SSD1306 driver

ssd1306-lite is a minimal SSD1306 driver with:
- internal framebuffer
- dirty page tracking
- cooperative async yielding

This driver is designed for deterministic behavior in async environments
(e.g. Embassy), ensuring long operations periodically yield execution.

The display is assumed to be **128x64** in page addressing mode with I2C interface.

The frame buffer can be updated directly using methods `set_pixel()`,
`unset_pixel()`, or `update_pixel()`.  The position of a pixel should be specified
by a coordinate. The top-left corner is the origin. The position goes right
as x grows while it goes down as y grows.

There are also severl high-level drawing function that helps following use cases:
- printing text
- drawing embedded-graphics styled
- rendering image
- drawing arc

The image on the frame buffer is sent to the display by method `flush()`
or `flush_full()`.
