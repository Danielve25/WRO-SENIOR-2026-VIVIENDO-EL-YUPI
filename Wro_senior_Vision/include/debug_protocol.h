#pragma once

#include <Arduino.h>
#include "esp_camera.h"

namespace debug_protocol {

constexpr uint8_t VERSION = 1;
constexpr uint8_t TYPE_ORIGINAL_JPEG = 1;
constexpr uint8_t TYPE_FLASH_COMMAND = 6;
constexpr size_t MAX_JPEG_PAYLOAD = 60000;

using CommandHandler = void (*)(uint8_t brightness);

struct JpegBuffer {
    uint8_t *data = nullptr;
    size_t capacity = 0;
    size_t length = 0;
};

bool encodeFrame(const camera_fb_t *frame, JpegBuffer &output, uint8_t quality);
bool sendPacket(HardwareSerial &serial, uint8_t type, uint32_t sequence,
                const uint8_t *payload, size_t payloadLength);
bool sendFrameType(HardwareSerial &serial, const camera_fb_t *frame,
                   uint8_t type, uint32_t sequence, JpegBuffer &jpegBuffer,
                   uint8_t quality);
bool sendFrame(HardwareSerial &serial, const camera_fb_t *frame,
               uint32_t sequence, JpegBuffer &jpegBuffer, uint8_t quality);
void service(HardwareSerial &serial, CommandHandler commandHandler);

}  // namespace debug_protocol
