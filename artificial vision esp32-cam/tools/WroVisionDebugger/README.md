# WRO Vision Debugger

Standalone Windows viewer for the ESP32-CAM debug stream. It does not use the
Arduino Serial Monitor, WiFi, a web server, OpenCV, or image processing on the
computer.

## Build

From this directory:

```text
dotnet build --configuration Release
```

## Run

```text
dotnet run --configuration Release
```

Select the USB-UART COM port and the same baud rate configured in the ESP32.
The viewer receives binary packets and displays JPEG frames in its own window.

## Packet format

All integer values are little-endian.

```text
0xA5 0x5A
version          1 byte
type             1 byte
payload_length   2 bytes
sequence         4 bytes
payload          payload_length bytes
crc16            2 bytes
```

The CRC16-CCITT is calculated from `version` through the last payload byte,
with initial value `0xFFFF` and polynomial `0x1021`. The two CRC bytes are not
included in the CRC calculation.

Packet types:

```text
1 = original annotated JPEG
2 = corrected-perspective JPEG
3 = grid metadata
4 = statistics
5 = system status
```

Metadata type `3` currently uses a 114-byte payload:

```text
grid_valid             1 byte
overall_confidence     1 byte
processing_ms          2 bytes
fps_times_ten          2 bytes
12 cell records        9 bytes each
```

Each cell record contains color code, confidence, RGB, hue, saturation and
value. Color codes are 1 yellow, 2 blue, 3 green, and 4 white.
