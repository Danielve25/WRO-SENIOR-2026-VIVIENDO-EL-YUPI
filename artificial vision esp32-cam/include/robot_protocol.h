#pragma once

#include <Arduino.h>

#include "vision_pipeline.h"

namespace robot_protocol {

constexpr uint32_t BAUD_RATE = 115200;
constexpr int RX_GPIO = 13;
constexpr int TX_GPIO = 14;
constexpr uint8_t VERSION = 1;
constexpr uint8_t TYPE_PATTERN = 1;
constexpr uint8_t TYPE_ACK = 2;
constexpr uint8_t PATTERN_PAYLOAD_LENGTH = 26;

class Link {
public:
    void begin();
    void service();
    void publish(const vision::Result &result);

private:
    void feed(uint8_t value);
    void resetParser();
    void sendPacket(uint8_t type, uint16_t sequence,
                    const uint8_t *payload, uint8_t length);

    enum class ParserState : uint8_t {
        WaitMagicFirst,
        WaitMagicSecond,
        Version,
        Type,
        Length,
        SequenceLow,
        SequenceHigh,
        Payload,
        CrcLow,
        CrcHigh
    };

    ParserState parserState = ParserState::WaitMagicFirst;
    uint8_t receivedVersion = 0;
    uint8_t receivedType = 0;
    uint8_t receivedLength = 0;
    uint8_t receivedIndex = 0;
    uint16_t receivedSequence = 0;
    uint16_t receivedCrc = 0;
    uint16_t calculatedCrc = 0;
    uint8_t receivedPayload[64] = {};

    bool hasPattern = false;
    uint8_t lastColors[vision::CELL_COUNT] = {};
    uint8_t lastConfidence[vision::CELL_COUNT] = {};
    uint32_t lastPublishMillis = 0;
    uint16_t nextSequence = 1;
};

}  // namespace robot_protocol
