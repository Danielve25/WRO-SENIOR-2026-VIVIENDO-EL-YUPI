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
constexpr uint32_t COMPETITION_DEBUG_BAUD_RATE = 115200;
constexpr uint8_t JPEG_QUALITY = 45;
constexpr int FLASH_GPIO = 4;
constexpr uint8_t FLASH_PWM_CHANNEL = 7;
constexpr uint32_t FLASH_PWM_FREQUENCY = 5000;
constexpr uint8_t FLASH_PWM_RESOLUTION = 8;
constexpr uint8_t TYPE_CORRECTED_JPEG = 2;
constexpr uint8_t TYPE_METADATA = 3;
constexpr size_t METADATA_LENGTH = 10U + vision::CELL_COUNT * 9U;

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
    payload[6] = result.borderScore;
    payload[7] = result.gridScore;
    payload[8] = result.validColorCells;
    payload[9] = result.rejectionCode;

    size_t offset = 10;
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

void setFlash(uint8_t brightness);

void indicateError()
{
    while (true) {
        setFlash(0);
        delay(150);
        setFlash(255);
        delay(150);
    }
}

void setFlash(uint8_t brightness)
{
    ledcWrite(FLASH_PWM_CHANNEL, brightness);
}

void initializeFlash()
{
    ledcSetup(FLASH_PWM_CHANNEL, FLASH_PWM_FREQUENCY, FLASH_PWM_RESOLUTION);
    ledcAttachPin(FLASH_GPIO, FLASH_PWM_CHANNEL);
    setFlash(0);
}

}  // namespace

void setup()
{
    initializeFlash();
#if WRO_DEBUG_MODE
    Serial.begin(DEBUG_BAUD_RATE, SERIAL_8N1);
#elif WRO_COMPETITION_DEBUG
    Serial.begin(COMPETITION_DEBUG_BAUD_RATE, SERIAL_8N1);
#endif
    if (!visionPipeline.begin() || !initializeCamera()
#if WRO_DEBUG_MODE
        || !allocateDebugStorage()
#endif
    ) {
        indicateError();
    }

    robotLink.begin(setFlash);
    setFlash(0);
}

void loop()
{
    robotLink.service();
#if WRO_DEBUG_MODE
    debug_protocol::service(Serial, setFlash);
#endif
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
    }
    visionPipeline.drawDebugOverlay(frame, result);

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
