#include "debug_protocol.h"

#include <cstring>

namespace {

struct JpegWriter {
    debug_protocol::JpegBuffer *output;
};

uint16_t updateCrc(uint16_t crc, uint8_t value)
{
    crc ^= static_cast<uint16_t>(value) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000U) != 0U
            ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
            : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

size_t jpegWriterCallback(void *argument, size_t index, const void *data, size_t length)
{
    JpegWriter *writer = static_cast<JpegWriter *>(argument);
    debug_protocol::JpegBuffer &output = *writer->output;

    if (index > output.capacity || length > output.capacity - index) {
        return 0;
    }

    std::memcpy(output.data + index, data, length);
    const size_t end = index + length;
    if (end > output.length) {
        output.length = end;
    }
    return length;
}

void writeLittleEndian16(HardwareSerial &serial, uint16_t value)
{
    const uint8_t bytes[2] = {
        static_cast<uint8_t>(value & 0xFFU),
        static_cast<uint8_t>((value >> 8) & 0xFFU)
    };
    serial.write(bytes, sizeof(bytes));
}

void writeLittleEndian32(HardwareSerial &serial, uint32_t value)
{
    const uint8_t bytes[4] = {
        static_cast<uint8_t>(value & 0xFFU),
        static_cast<uint8_t>((value >> 8) & 0xFFU),
        static_cast<uint8_t>((value >> 16) & 0xFFU),
        static_cast<uint8_t>((value >> 24) & 0xFFU)
    };
    serial.write(bytes, sizeof(bytes));
}

}  // namespace

namespace debug_protocol {

bool encodeFrame(const camera_fb_t *frame, JpegBuffer &output, uint8_t quality)
{
    if (frame == nullptr || output.data == nullptr || output.capacity == 0) {
        return false;
    }

    output.length = 0;
    JpegWriter writer{&output};
    return frame2jpg_cb(const_cast<camera_fb_t *>(frame), quality,
                        jpegWriterCallback, &writer);
}

bool sendPacket(HardwareSerial &serial, uint8_t type, uint32_t sequence,
                const uint8_t *payload, size_t payloadLength)
{
    if (payload == nullptr || payloadLength > UINT16_MAX) {
        return false;
    }

    uint16_t crc = 0xFFFFU;
    const uint8_t version = VERSION;
    const uint16_t length = static_cast<uint16_t>(payloadLength);

    const uint8_t magic[2] = {0xA5, 0x5A};
    serial.write(magic, sizeof(magic));

    serial.write(version);
    crc = updateCrc(crc, version);
    serial.write(type);
    crc = updateCrc(crc, type);

    const uint8_t lengthBytes[2] = {
        static_cast<uint8_t>(length & 0xFFU),
        static_cast<uint8_t>((length >> 8) & 0xFFU)
    };
    serial.write(lengthBytes, sizeof(lengthBytes));
    crc = updateCrc(crc, lengthBytes[0]);
    crc = updateCrc(crc, lengthBytes[1]);

    const uint8_t sequenceBytes[4] = {
        static_cast<uint8_t>(sequence & 0xFFU),
        static_cast<uint8_t>((sequence >> 8) & 0xFFU),
        static_cast<uint8_t>((sequence >> 16) & 0xFFU),
        static_cast<uint8_t>((sequence >> 24) & 0xFFU)
    };
    serial.write(sequenceBytes, sizeof(sequenceBytes));
    for (uint8_t value : sequenceBytes) {
        crc = updateCrc(crc, value);
    }

    serial.write(payload, payloadLength);
    for (size_t index = 0; index < payloadLength; ++index) {
        crc = updateCrc(crc, payload[index]);
    }

    writeLittleEndian16(serial, crc);
    return true;
}

bool sendFrameType(HardwareSerial &serial, const camera_fb_t *frame,
                   uint8_t type, uint32_t sequence, JpegBuffer &jpegBuffer,
                   uint8_t quality)
{
    if (!encodeFrame(frame, jpegBuffer, quality) ||
        jpegBuffer.length > MAX_JPEG_PAYLOAD) {
        return false;
    }

    return sendPacket(serial, type, sequence,
                      jpegBuffer.data, jpegBuffer.length);
}

bool sendFrame(HardwareSerial &serial, const camera_fb_t *frame,
               uint32_t sequence, JpegBuffer &jpegBuffer, uint8_t quality)
{
    return sendFrameType(serial, frame, TYPE_ORIGINAL_JPEG, sequence,
                         jpegBuffer, quality);
}

}  // namespace debug_protocol
