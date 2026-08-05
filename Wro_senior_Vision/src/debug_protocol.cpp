#include "debug_protocol.h"

#include <cstring>

namespace {

constexpr uint8_t MAX_COMMAND_PAYLOAD = 4;

enum class CommandParserState : uint8_t {
    WaitMagicFirst,
    WaitMagicSecond,
    Version,
    Type,
    LengthLow,
    LengthHigh,
    Sequence0,
    Sequence1,
    Sequence2,
    Sequence3,
    Payload,
    CrcLow,
    CrcHigh
};

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

class CommandParser {
public:
    void service(HardwareSerial &serial, debug_protocol::CommandHandler handler)
    {
        while (serial.available() > 0) {
            feed(static_cast<uint8_t>(serial.read()), handler);
        }
    }

private:
    void feed(uint8_t value, debug_protocol::CommandHandler handler)
    {
        switch (state) {
            case CommandParserState::WaitMagicFirst:
                if (value == 0xA5) state = CommandParserState::WaitMagicSecond;
                break;

            case CommandParserState::WaitMagicSecond:
                if (value == 0x5A) {
                    crc = 0xFFFFU;
                    payloadIndex = 0;
                    state = CommandParserState::Version;
                } else {
                    state = value == 0xA5
                        ? CommandParserState::WaitMagicSecond
                        : CommandParserState::WaitMagicFirst;
                }
                break;

            case CommandParserState::Version:
                version = value;
                crc = updateCrc(crc, value);
                state = CommandParserState::Type;
                break;

            case CommandParserState::Type:
                type = value;
                crc = updateCrc(crc, value);
                state = CommandParserState::LengthLow;
                break;

            case CommandParserState::LengthLow:
                length = value;
                crc = updateCrc(crc, value);
                state = CommandParserState::LengthHigh;
                break;

            case CommandParserState::LengthHigh:
                length |= static_cast<uint16_t>(value) << 8;
                crc = updateCrc(crc, value);
                if (length > MAX_COMMAND_PAYLOAD) {
                    reset();
                } else {
                    state = CommandParserState::Sequence0;
                }
                break;

            case CommandParserState::Sequence0:
                crc = updateCrc(crc, value);
                state = CommandParserState::Sequence1;
                break;

            case CommandParserState::Sequence1:
                crc = updateCrc(crc, value);
                state = CommandParserState::Sequence2;
                break;

            case CommandParserState::Sequence2:
                crc = updateCrc(crc, value);
                state = CommandParserState::Sequence3;
                break;

            case CommandParserState::Sequence3:
                crc = updateCrc(crc, value);
                state = length == 0 ? CommandParserState::CrcLow
                                    : CommandParserState::Payload;
                break;

            case CommandParserState::Payload:
                payload[payloadIndex++] = value;
                crc = updateCrc(crc, value);
                if (payloadIndex == length) state = CommandParserState::CrcLow;
                break;

            case CommandParserState::CrcLow:
                receivedCrc = value;
                state = CommandParserState::CrcHigh;
                break;

            case CommandParserState::CrcHigh:
                receivedCrc |= static_cast<uint16_t>(value) << 8;
                if (receivedCrc == crc && version == debug_protocol::VERSION &&
                    type == debug_protocol::TYPE_FLASH_COMMAND && length == 1U &&
                    handler != nullptr) {
                    handler(payload[0]);
                }
                reset();
                break;
        }
    }

    void reset()
    {
        state = CommandParserState::WaitMagicFirst;
        length = 0;
        payloadIndex = 0;
        receivedCrc = 0;
        crc = 0;
    }

    CommandParserState state = CommandParserState::WaitMagicFirst;
    uint8_t version = 0;
    uint8_t type = 0;
    uint16_t length = 0;
    uint16_t payloadIndex = 0;
    uint8_t payload[MAX_COMMAND_PAYLOAD] = {};
    uint16_t receivedCrc = 0;
    uint16_t crc = 0;
};

CommandParser commandParser;

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

void service(HardwareSerial &serial, CommandHandler commandHandler)
{
    commandParser.service(serial, commandHandler);
}

}  // namespace debug_protocol
