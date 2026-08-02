# WRO Senior Vision

Vision system for the AI Thinker ESP32-CAM and LEGO SPIKE Prime with Pybricks.
All image processing runs on the ESP32-CAM. The computer is only a graphical
debug display.

## Hardware

- AI Thinker ESP32-CAM with OV2640.
- USB-UART adapter with 3.3 V logic for DEBUG.
- Separate 3.3 V UART connection to a Powered Up port on SPIKE Prime.
- Common ground between the ESP32-CAM and SPIKE.
- Do not power the camera from a SPIKE signal pin. Use a suitable external
  supply and connect only logic signals and ground.

The camera UART uses GPIO1 and GPIO3. SPIKE UART uses GPIO13 and GPIO14:

```text
ESP32 GPIO14 TX -> SPIKE UART RX
ESP32 GPIO13 RX <- SPIKE UART TX
ESP32 GND       <-> SPIKE GND
```

The DEBUG USB-UART connection is:

```text
USB-UART TX -> ESP32 GPIO3 / RX0
USB-UART RX <- ESP32 GPIO1 / TX0
GND         <-> ESP32 GND
```

## Build ESP32

DEBUG firmware:

```text
pio run -e esp32cam
pio run -e esp32cam -t upload
```

COMPETITION firmware:

```text
pio run -e esp32cam_competition
pio run -e esp32cam_competition -t upload
```

The default PlatformIO environment is DEBUG. COMPETITION disables JPEG
encoding, overlays, metadata and video transmission.

## Debug Viewer

Build the independent Windows application:

```text
cd tools/WroVisionDebugger
dotnet run --configuration Release
```

The application opens a graphical window. It does not use Arduino Serial
Monitor, WiFi, a web server or OpenCV. Select the USB-UART COM port and
`921600` baud. `460800` remains available as a fallback for USB-UART adapters
that are unstable at high speed.

The window contains the annotated original image, the perspective-corrected
image, cell metadata, RGB, HSV, confidence, FPS and communication errors.

## ESP32 Debug Protocol

All values are little-endian:

```text
0xA5 0x5A
version          1 byte
type             1 byte
payload_length   2 bytes
sequence         4 bytes
payload          payload_length bytes
crc16            2 bytes
```

CRC16-CCITT starts at `0xFFFF`, uses polynomial `0x1021`, and covers version
through payload.

## SPIKE Software

Copy `spike/wro_vision_receiver.py` and `spike/example.py` to the Pybricks
project. The receiver uses `UARTDevice` at 115200 baud, checks the CRC, sends
ACK packets and exposes the twelve cells through `GridPattern`.

## Current Processing Pipeline

1. Capture QVGA RGB565.
2. Reduce luminance to 160 x 120.
3. Build a Sobel edge map.
4. Detect long candidate lines with a reduced Hough accumulator.
5. Select a quadrilateral compatible with the 4 x 3 frame.
6. Calculate a projective homography.
7. Sample the central region of each cell.
8. Classify HSV with normalized RGB verification.
9. Draw the frame, lines, corners, grid and samples in DEBUG.
10. Send the pattern with CRC16 to SPIKE.
