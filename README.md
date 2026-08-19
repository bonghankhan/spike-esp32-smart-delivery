# SPIKE Prime × ESP32 Smart Delivery Rover

A followable project where LEGO SPIKE Prime handles motion/sensing and an ESP32 provides BLE provisioning plus a Wi-Fi mission-control dashboard.

## Architecture

Phone/Tablet → Wi-Fi → ESP32 → BLE (official SPIKE App 3 protocol) → SPIKE Prime Hub → Motors + Distance Sensor

There is **no direct electrical connection** between the ESP32 and SPIKE hardware.

## Hardware

- LEGO Education SPIKE Prime Hub
- 2 SPIKE motors for a two-wheel driving base
- SPIKE Distance Sensor
- ESP32 or ESP32-S3 development board with BLE + Wi-Fi
- USB power for ESP32
- Phone/tablet/laptop for the dashboard

## LEGO ports used by the supplied code

- A: left motor
- B: right motor
- C: distance sensor

Change the constants/code if your build uses different ports.

## Recommended build

Use LEGO Education's official **Simple Driving Base** or a similar two-wheel base. Mount the Distance Sensor facing straight forward.

## Software setup

1. Install Arduino IDE 2.x.
2. Install ESP32 board support.
3. Install **NimBLE-Arduino 2.x** from Library Manager.
4. Open `code/esp32_spike_bridge.ino`.
5. Select your ESP32 board and upload.
6. Turn on the SPIKE Prime Hub and make it available for Bluetooth connection.
7. Close the LEGO SPIKE App before letting the ESP32 connect.
8. Open Serial Monitor at 115200 baud.
9. Wait for `SPIKE connected`, `Handshake OK`, and `Program uploaded`.
10. On your phone, join Wi-Fi `SPIKE-ESP32` with password `spikeprime`.
11. Browse to `http://192.168.4.1`.
12. Press **START**.

## Expected behavior

- The rover drives forward.
- When the Distance Sensor detects an object within ~180 mm, the rover stops, reverses slightly, turns, and continues.
- The ESP32 dashboard displays Hub battery and measured distance when device notifications are available.
- The dashboard START/STOP buttons control program slot 0.

## First test sequence

Before running on the floor:

1. Lift the rover so the wheels are off the ground.
2. Power the Hub and ESP32.
3. Confirm BLE connection in Serial Monitor.
4. Press START and verify both motors rotate forward.
5. Put a book 10–15 cm in front of the Distance Sensor and verify stop/reverse/turn behavior.
6. Press STOP and confirm all program motion ends.
7. Only then test on the floor at low speed.

## If motors run backward

Swap motor A/B physically, reverse the motor pair order, or invert the sign of velocity in the SPIKE program.

## If the Hub is not found

- Close the SPIKE App; another active connection can prevent the intended connection flow.
- Make sure the Hub is on and wireless/Bluetooth is enabled.
- Keep the ESP32 within 1–2 m for the first test.
- Verify the Serial Monitor shows a scan for service `FD02`.

## If program upload fails

The ESP32 example follows LEGO's documented sequence: InfoRequest → ClearSlot → StartFileUpload → TransferChunk(s) → ProgramFlow. The SPIKE protocol uses a custom COBS framing step, XOR with `0x03`, and CRC32 aligned to 4-byte boundaries. If your Hub firmware behavior differs, compare Serial logs with LEGO's official reference Python client.

## Hardware-test status

The project structure and protocol fields are based on LEGO's published SPIKE Prime protocol and LEGO Education Python examples. The ESP32 implementation in this package is a reference implementation and was **not physically bench-tested in this ChatGPT environment**. Treat the first run as a bring-up/debug session.

## Sources

- LEGO SPIKE Prime protocol: https://lego.github.io/spike-prime-docs/
- LEGO SPIKE Prime connection details: https://lego.github.io/spike-prime-docs/connect.html
- LEGO SPIKE Prime messages: https://lego.github.io/spike-prime-docs/messages.html
- LEGO official BLE reference client: https://github.com/LEGO/spike-prime-docs/tree/main/examples/python
- LEGO Education Cart Control / Distance Sensor lesson: https://education.lego.com/en-us/lessons/spike-python-u3-sensor-control/spike-python-u3l3/
- NimBLE-Arduino: https://github.com/h2zero/NimBLE-Arduino
