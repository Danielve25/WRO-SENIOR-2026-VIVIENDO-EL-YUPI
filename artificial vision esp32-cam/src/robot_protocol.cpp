#include "robot_protocol.h"

#include <cstring>

namespace {

constexpr uint8_t MAGIC_FIRST = 0xC3;
constexpr uint8_t MAGIC_SECOND = 0x3C;

uint16_t updateCrc(uint16_t crc, uint8_t value)
{
    crc ^= static_cast<uint16_t>(value) << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
        crc = (crc & 0x8000U) != 0U
            ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
            : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

}  // namespace

namespace robot_protocol {

void Link::begin()
{
    Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_GPIO, TX_GPIO);
    while (Serial2.available() > 0) {
        Serial2.read();
    }
}

void Link::service()
{
    while (Serial2.available() > 0) {
        feed(static_cast<uint8_t>(Serial2.read()));
    }
}

void Link::publish(const vision::Result &result)
{
    if (!result.valid) {
        return;
    }

    uint8_t colors[vision::CELL_COUNT];
    uint8_t confidence[vision::CELL_COUNT];
    bool changed = !hasPattern;
    for (uint8_t index = 0; index < vision::CELL_COUNT; index++) {
        colors[index] = static_cast<uint8_t>(result.cells[index].color);
        confidence[index] = result.cells[index].confidence;
        if (hasPattern && (colors[index] != lastColors[index] ||
                           confidence[index] != lastConfidence[index])) {
            changed = true;
        }
    }

    const uint32_t now = millis();
    if (!changed && now - lastPublishMillis < 1000U) {
        return;
    }

    uint8_t payload[PATTERN_PAYLOAD_LENGTH];
    payload[0] = 1;
    payload[1] = result.confidence;
    std::memcpy(payload + 2, colors, vision::CELL_COUNT);
    std::memcpy(payload + 2 + vision::CELL_COUNT, confidence,
                vision::CELL_COUNT);

    const uint16_t sequence = nextSequence++;
    sendPacket(TYPE_PATTERN, sequence, payload, sizeof(payload));
    std::memcpy(lastColors, colors, sizeof(lastColors));
    std::memcpy(lastConfidence, confidence, sizeof(lastConfidence));
    lastPublishMillis = now;
    hasPattern = true;
}

void Link::feed(uint8_t value)
{
    switch (parserState) {
        case ParserState::WaitMagicFirst:
            if (value == MAGIC_FIRST) parserState = ParserState::WaitMagicSecond;
            break;

        case ParserState::WaitMagicSecond:
            if (value == MAGIC_SECOND) {
                calculatedCrc = 0xFFFFU;
                receivedIndex = 0;
                parserState = ParserState::Version;
            } else {
                parserState = value == MAGIC_FIRST
                    ? ParserState::WaitMagicSecond
                    : ParserState::WaitMagicFirst;
            }
            break;

        case ParserState::Version:
            receivedVersion = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            parserState = ParserState::Type;
            break;

        case ParserState::Type:
            receivedType = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            parserState = ParserState::Length;
            break;

        case ParserState::Length:
            receivedLength = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            if (receivedLength > sizeof(receivedPayload)) {
                resetParser();
            } else {
                parserState = ParserState::SequenceLow;
            }
            break;

        case ParserState::SequenceLow:
            receivedSequence = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            parserState = ParserState::SequenceHigh;
            break;

        case ParserState::SequenceHigh:
            receivedSequence |= static_cast<uint16_t>(value) << 8;
            calculatedCrc = updateCrc(calculatedCrc, value);
            parserState = receivedLength == 0 ? ParserState::CrcLow : ParserState::Payload;
            break;

        case ParserState::Payload:
            receivedPayload[receivedIndex++] = value;
            calculatedCrc = updateCrc(calculatedCrc, value);
            if (receivedIndex == receivedLength) parserState = ParserState::CrcLow;
            break;

        case ParserState::CrcLow:
            receivedCrc = value;
            parserState = ParserState::CrcHigh;
            break;

        case ParserState::CrcHigh:
            receivedCrc |= static_cast<uint16_t>(value) << 8;
            if (receivedCrc == calculatedCrc && receivedVersion == VERSION &&
                receivedType == TYPE_ACK && receivedLength == 2) {
                const uint16_t acknowledged = static_cast<uint16_t>(receivedPayload[0]) |
                                              static_cast<uint16_t>(receivedPayload[1]) << 8;
                (void)acknowledged;
            }
            resetParser();
            break;
    }
}

void Link::resetParser()
{
    parserState = ParserState::WaitMagicFirst;
    receivedIndex = 0;
    receivedLength = 0;
    receivedSequence = 0;
    receivedCrc = 0;
    calculatedCrc = 0;
}

void Link::sendPacket(uint8_t type, uint16_t sequence,
                      const uint8_t *payload, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    const uint8_t header[2] = {MAGIC_FIRST, MAGIC_SECOND};
    Serial2.write(header, sizeof(header));

    Serial2.write(VERSION);
    crc = updateCrc(crc, VERSION);
    Serial2.write(type);
    crc = updateCrc(crc, type);
    Serial2.write(length);
    crc = updateCrc(crc, length);

    const uint8_t sequenceBytes[2] = {
        static_cast<uint8_t>(sequence & 0xFFU),
        static_cast<uint8_t>((sequence >> 8) & 0xFFU)
    };
    Serial2.write(sequenceBytes, sizeof(sequenceBytes));
    crc = updateCrc(crc, sequenceBytes[0]);
    crc = updateCrc(crc, sequenceBytes[1]);

    Serial2.write(payload, length);
    for (uint8_t index = 0; index < length; index++) {
        crc = updateCrc(crc, payload[index]);
    }

    const uint8_t crcBytes[2] = {
        static_cast<uint8_t>(crc & 0xFFU),
        static_cast<uint8_t>((crc >> 8) & 0xFFU)
    };
    Serial2.write(crcBytes, sizeof(crcBytes));
    Serial2.flush();
}

}  // namespace robot_protocol
