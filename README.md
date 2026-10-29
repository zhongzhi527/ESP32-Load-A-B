# ESP32-Load-A-B — DUAL SCALE load-cell node

Firmware for the **load-cell ESP32** in the **DUAL SCALE** system. Each node
reads a single load cell through a SparkFun NAU7802 amplifier, shows the live
weight on a small on-board SSD1306 OLED, and streams the reading over
**Bluetooth Low Energy (BLE)** to the handheld display.

This is the **scale** half of the DUAL system. It pairs with the handheld
display firmware **[Display-Load-A-B](https://github.com/zhongzhi527/Display-Load-A-B)**.

> There are two load-cell systems. This repo is the **2-cell Bluetooth** one.
> The separate **4-cell ESP-NOW** system lives in
> `ESP32-Load-A-B-C-D` (scale) + `Display-Load-A-B-C-D` (display).

## Hardware

- ESP32 dev module
- SparkFun Qwiic Scale **NAU7802** load-cell amplifier (I²C: SDA=GPIO23, SCL=GPIO22)
- SSD1306 128×64 OLED (same I²C bus, address 0x3C or 0x3D)
- Status LEDs: green=GPIO16, red=GPIO33
- Push button on GPIO4 (short press = unlock / re-arm lock; hold ≥0.8 s = guided 1 kg calibration)

## Two units: A and B

One source builds both load cells. The build environment selects which unit a
given ESP32 becomes (it sets the `LOAD_CELL_UNIT` flag and the BLE name the
display pairs with):

| Build environment | Unit | BLE name  |
|-------------------|------|-----------|
| `load_cell_a`     | A    | LOAD_CELL_1 |
| `load_cell_b`     | B    | LOAD_CELL_2 |

Flash each physical board with the matching environment — Load Cell A gets
`load_cell_a`, Load Cell B gets `load_cell_b`.

## Build & flash (PlatformIO)

Plug in **one** board at a time so you know which unit it becomes.

```sh
# Load Cell A
pio run -e load_cell_a -t upload

# Load Cell B
pio run -e load_cell_b -t upload
```

In VS Code you can instead use the PlatformIO sidebar: expand the
`load_cell_a` (or `load_cell_b`) environment → **Upload**.

### Platform version is pinned on purpose

`platformio.ini` pins `platform = espressif32@6.5.0`. Leaving it unpinned pulls
the newest platform (8.0.0), whose ESP-IDF is incompatible with
`NimBLE-Arduino@2.3.2` and fails to build (`syscfg.h` / `nimconfig.h` not
found). If a build ever fails with those errors, confirm the pin is present and
run a clean build (delete the `.pio` folder, then rebuild).

## Pairing

Keep this scale and the handheld display on matching firmware — they share the
`DualScaleBleProtocol.h` packet definitions. Flashing only one side can leave
them unable to connect. After flashing, power-cycle both and give them ~20 s to
link; the OLED footer shows `BLE:ON` once the display connects.
