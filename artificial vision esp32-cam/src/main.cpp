#include <Arduino.h>
#include <esp_heap_caps.h>
#include <algorithm>

#include "camera_board.h"
#include "debug_protocol.h"
#include "robot_protocol.h"
#include "app_config.h"
#include "vision_pipeline.h"

namespace {

constexpr uint32_t DEBUG_BAUD_RATE = 921600;
constexpr uint8_t JPEG_QUALITY = 45;
constexpr int STATUS_LED_GPIO = 4;
constexpr uint8_t TYPE_CORRECTED_JPEG = 2;
constexpr uint8_t TYPE_METADATA = 3;
constexpr size_t METADATA_LENGTH = 6U + vision::CELL_COUNT * 9U;

uint8_t *jpegStorage = nullptr;
uint8_t *previewStorage = nullptr;
uint32_t frameSequence = 0;
uint32_t previousFrameMillis = 0;
vision::Pipeline visionPipeline;
robot_protocol::Link robotLink;

bool allocateDebugStorage()
{
    jpegStorage = static_cast<uint8_t *>(
        heap_caps_malloc(debug_protocol::MAX_JPEG_PAYLOAD, MALLOC_CAP_SPIRAM));
    previewStorage = static_cast<uint8_t *>(heap_caps_malloc(
        static_cast<size_t>(vision::ANALYSIS_WIDTH) * vision::ANALYSIS_HEIGHT * 2U,
        MALLOC_CAP_SPIRAM));
    return jpegStorage != nullptr && previewStorage != nullptr;
}

bool initializeCamera()
{
    if (!psramFound()) {
        return false;
    }

    camera_config_t config = camera_board::makeConfig();
    return esp_camera_init(&config) == ESP_OK;
}

void writeLittleEndian16(uint8_t *buffer, size_t offset, uint16_t value)
{
    buffer[offset] = static_cast<uint8_t>(value & 0xFFU);
    buffer[offset + 1U] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

void encodeMetadata(const vision::Result &result, uint8_t *payload)
{
    payload[0] = result.valid ? 1 : 0;
    payload[1] = result.confidence;
    writeLittleEndian16(payload, 2, result.processingMilliseconds);
    writeLittleEndian16(payload, 4, result.framesPerSecondTenths);

    size_t offset = 6;
    for (uint8_t index = 0; index < vision::CELL_COUNT; index++) {
        const vision::CellResult &cell = result.cells[index];
        payload[offset] = static_cast<uint8_t>(cell.color);
        payload[offset + 1U] = cell.confidence;
        payload[offset + 2U] = cell.red;
        payload[offset + 3U] = cell.green;
        payload[offset + 4U] = cell.blue;
        writeLittleEndian16(payload, offset + 5U, cell.hue);
        payload[offset + 7U] = cell.saturation;
        payload[offset + 8U] = cell.value;
        offset += 9;
    }
}

void indicateError()
{
    pinMode(STATUS_LED_GPIO, OUTPUT);
    while (true) {
        digitalWrite(STATUS_LED_GPIO, LOW);
        delay(150);
        digitalWrite(STATUS_LED_GPIO, HIGH);
        delay(150);
    }
}

}  // namespace

void setup()
{
    pinMode(STATUS_LED_GPIO, OUTPUT);
    digitalWrite(STATUS_LED_GPIO, HIGH);
#if WRO_DEBUG_MODE
    Serial.begin(DEBUG_BAUD_RATE, SERIAL_8N1);

#endif
    if (!visionPipeline.begin() || !initializeCamera()
#if WRO_DEBUG_MODE
        || !allocateDebugStorage()
#endif
    ) {
        indicateError();
    }

    robotLink.begin();
    digitalWrite(STATUS_LED_GPIO, LOW);
}

void loop()
{
    robotLink.service();
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame == nullptr) {
        delay(10);
        return;
    }

    vision::Result result;
    const bool detected = visionPipeline.process(frame, result);
    robotLink.publish(result);
    const uint32_t now = millis();
    if (previousFrameMillis != 0U && now > previousFrameMillis) {
        result.framesPerSecondTenths = static_cast<uint16_t>(std::min<uint32_t>(
            65535U, 10000U / (now - previousFrameMillis)));
    }
    previousFrameMillis = now;

#if WRO_DEBUG_MODE
    if (detected) {
        visionPipeline.renderPreview(
            frame, result, previewStorage,
            static_cast<size_t>(vision::ANALYSIS_WIDTH) * vision::ANALYSIS_HEIGHT * 2U);
        visionPipeline.drawDebugOverlay(frame, result);
    }

    debug_protocol::JpegBuffer jpegBuffer;
    jpegBuffer.data = jpegStorage;
    jpegBuffer.capacity = debug_protocol::MAX_JPEG_PAYLOAD;
    jpegBuffer.length = 0;
    const uint32_t sequence = frameSequence++;
    debug_protocol::sendFrame(Serial, frame, sequence, jpegBuffer, JPEG_QUALITY);

    if (detected) {
        camera_fb_t previewFrame;
        previewFrame.buf = previewStorage;
        previewFrame.len = static_cast<size_t>(vision::ANALYSIS_WIDTH) *
                           vision::ANALYSIS_HEIGHT * 2U;
        previewFrame.width = vision::ANALYSIS_WIDTH;
        previewFrame.height = vision::ANALYSIS_HEIGHT;
        previewFrame.format = PIXFORMAT_RGB565;
        debug_protocol::sendFrameType(Serial, &previewFrame, TYPE_CORRECTED_JPEG,
                                      sequence, jpegBuffer, JPEG_QUALITY);
    }

    uint8_t metadata[METADATA_LENGTH];
    encodeMetadata(result, metadata);
    debug_protocol::sendPacket(Serial, TYPE_METADATA, sequence,
                                metadata, sizeof(metadata));
    Serial.flush();
#else
    (void)detected;
#endif
    esp_camera_fb_return(frame);
}
