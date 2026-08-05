#include "robot_protocol.h"

#include <cstring>

#include "app_config.h"

namespace
{

    constexpr uint8_t MAGIC_FIRST = 0xC3;
    constexpr uint8_t MAGIC_SECOND = 0x3C;
    HardwareSerial competitionSerial(2);

    uint16_t updateCrc(uint16_t crc, uint8_t value)
    {
        crc ^= static_cast<uint16_t>(value) << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1);
        }
        return crc;
    }

#if WRO_COMPETITION_DEBUG
    void logPacket(uint8_t type, uint16_t sequence,
                   const uint8_t *payload, uint8_t length, uint16_t crc)
    {
        Serial.printf("TX type=%u seq=%u len=%u payload=",
                      static_cast<unsigned>(type),
                      static_cast<unsigned>(sequence),
                      static_cast<unsigned>(length));
        for (uint8_t index = 0; index < length; ++index)
        {
            Serial.printf("%02X", static_cast<unsigned>(payload[index]));
        }
        Serial.printf(" crc=%04X\n", static_cast<unsigned>(crc));
    }
#endif

} // namespace

namespace robot_protocol
{

    void Link::begin(CommandHandler handler)
    {
        commandHandler = handler;
        competitionSerial.begin(BAUD_RATE, SERIAL_8N1, RX_GPIO, TX_GPIO);
        while (competitionSerial.available() > 0)
        {
            competitionSerial.read();
        }
    }

    void Link::service()
    {
        while (competitionSerial.available() > 0)
        {
            feed(static_cast<uint8_t>(competitionSerial.read()));
        }
    }

    void Link::publish(const vision::Result &result)
    {
        const uint32_t now = millis();
        if (!result.valid)
        {
#if WRO_COMPETITION_DEBUG
            static uint32_t lastInvalidLogMillis = 0;
            if (lastInvalidLogMillis == 0U ||
                now - lastInvalidLogMillis >= 1000U)
            {
                Serial.printf("NO_TX valid=0 confidence=%u border=%u grid=%u "
                              "colors=%u rejection=%u\n",
                              static_cast<unsigned>(result.confidence),
                              static_cast<unsigned>(result.borderScore),
                              static_cast<unsigned>(result.gridScore),
                              static_cast<unsigned>(result.validColorCells),
                              static_cast<unsigned>(result.rejectionCode));
                lastInvalidLogMillis = now;
            }
#endif
            return;
        }

        uint8_t colors[vision::CELL_COUNT];
        uint8_t confidence[vision::CELL_COUNT];
        bool changed = !hasPattern;
        for (uint8_t index = 0; index < vision::CELL_COUNT; index++)
        {
            colors[index] = static_cast<uint8_t>(result.cells[index].color);
            confidence[index] = result.cells[index].confidence;
            if (hasPattern && (colors[index] != lastColors[index] ||
                               confidence[index] != lastConfidence[index]))
            {
                changed = true;
            }
        }

        if (!changed && now - lastPublishMillis < 1000U)
        {
            return;
        }

        uint8_t payload[PATTERN_PAYLOAD_LENGTH];
        payload[0] = 1;
        payload[1] = result.confidence;
        std::memcpy(payload + 2, colors, vision::CELL_COUNT);
        std::memcpy(payload + 2 + vision::CELL_COUNT, confidence,
                    vision::CELL_COUNT);

        const uint16_t sequence = nextSequence++;

#if WRO_COMPETITION_DEBUG
        Serial.printf("PATTERN seq=%u valid=1 confidence=%u colors=",
                      static_cast<unsigned>(sequence),
                      static_cast<unsigned>(result.confidence));
        for (uint8_t index = 0; index < vision::CELL_COUNT; ++index)
        {
            if (index > 0U)
            {
                Serial.print(',');
            }
            Serial.print(static_cast<unsigned>(colors[index]));
        }
        Serial.print(" confidences=");
        for (uint8_t index = 0; index < vision::CELL_COUNT; ++index)
        {
            if (index > 0U)
            {
                Serial.print(',');
            }
            Serial.print(static_cast<unsigned>(confidence[index]));
        }
        Serial.println();
#endif

        sendPacket(TYPE_PATTERN, sequence, payload, sizeof(payload));
        std::memcpy(lastColors, colors, sizeof(lastColors));
        std::memcpy(lastConfidence, confidence, sizeof(lastConfidence));
        lastPublishMillis = now;
        hasPattern = true;
    }

    void Link::feed(uint8_t value)
    {
        switch (parserState)
        {
        case ParserState::WaitMagicFirst:
            if (value == MAGIC_FIRST)
                parserState = ParserState::WaitMagicSecond;
            break;

        case ParserState::WaitMagicSecond:
            if (value == MAGIC_SECOND)
            {
                calculatedCrc = 0xFFFFU;
                receivedIndex = 0;
                parserState = ParserState::Version;
            }
            else
            {
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
            if (receivedLength > sizeof(receivedPayload))
            {
                resetParser();
            }
            else
            {
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
            if (receivedIndex == receivedLength)
                parserState = ParserState::CrcLow;
            break;

        case ParserState::CrcLow:
            receivedCrc = value;
            parserState = ParserState::CrcHigh;
            break;

        case ParserState::CrcHigh:
            receivedCrc |= static_cast<uint16_t>(value) << 8;
            if (receivedCrc == calculatedCrc && receivedVersion == VERSION &&
                receivedType == TYPE_ACK && receivedLength == 2)
            {
                const uint16_t acknowledged = static_cast<uint16_t>(receivedPayload[0]) |
                                              static_cast<uint16_t>(receivedPayload[1]) << 8;
                (void)acknowledged;
            }
            else if (receivedCrc == calculatedCrc && receivedVersion == VERSION &&
                     receivedType == TYPE_COMMAND &&
                     receivedLength == COMMAND_PAYLOAD_LENGTH &&
                     receivedPayload[0] == COMMAND_FLASH)
            {
                if (commandHandler != nullptr)
                {
                    commandHandler(receivedPayload[1]);
                }
                const uint8_t acknowledgement[2] = {
                    static_cast<uint8_t>(receivedSequence & 0xFFU),
                    static_cast<uint8_t>((receivedSequence >> 8) & 0xFFU)};
                sendPacket(TYPE_ACK, receivedSequence, acknowledgement,
                           sizeof(acknowledgement));
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
        competitionSerial.write(header, sizeof(header));

        competitionSerial.write(VERSION);
        crc = updateCrc(crc, VERSION);
        competitionSerial.write(type);
        crc = updateCrc(crc, type);
        competitionSerial.write(length);
        crc = updateCrc(crc, length);

        const uint8_t sequenceBytes[2] = {
            static_cast<uint8_t>(sequence & 0xFFU),
            static_cast<uint8_t>((sequence >> 8) & 0xFFU)};
        competitionSerial.write(sequenceBytes, sizeof(sequenceBytes));
        crc = updateCrc(crc, sequenceBytes[0]);
        crc = updateCrc(crc, sequenceBytes[1]);

        competitionSerial.write(payload, length);
        for (uint8_t index = 0; index < length; index++)
        {
            crc = updateCrc(crc, payload[index]);
        }

        const uint8_t crcBytes[2] = {
            static_cast<uint8_t>(crc & 0xFFU),
            static_cast<uint8_t>((crc >> 8) & 0xFFU)};

#if WRO_COMPETITION_DEBUG
        logPacket(type, sequence, payload, length, crc);
#endif

        competitionSerial.write(crcBytes, sizeof(crcBytes));
        competitionSerial.flush();
    }

} // namespace robot_protocol
