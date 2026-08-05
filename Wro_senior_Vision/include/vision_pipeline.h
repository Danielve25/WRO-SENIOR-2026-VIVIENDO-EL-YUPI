#pragma once

#include <Arduino.h>
#include "esp_camera.h"

namespace vision {

constexpr uint16_t ANALYSIS_WIDTH = 160;
constexpr uint16_t ANALYSIS_HEIGHT = 120;
constexpr uint8_t GRID_COLUMNS = 4;
constexpr uint8_t GRID_ROWS = 3;
constexpr uint8_t CELL_COUNT = GRID_COLUMNS * GRID_ROWS;

struct Point {
    Point() : x(0.0f), y(0.0f) {}
    Point(float xValue, float yValue) : x(xValue), y(yValue) {}

    float x;
    float y;
};

struct Line {
    float theta = 0.0f;
    float rho = 0.0f;
    uint16_t score = 0;
};

enum class Color : uint8_t {
    Unknown = 0,
    Yellow = 1,
    Blue = 2,
    Green = 3,
    White = 4
};

struct CellResult {
    Color color = Color::Unknown;
    uint8_t confidence = 0;
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint16_t hue = 0;
    uint8_t saturation = 0;
    uint8_t value = 0;
    uint16_t sampleX = 0;
    uint16_t sampleY = 0;
};

struct Result {
    bool valid = false;
    bool candidateFound = false;
    uint8_t confidence = 0;
    uint8_t borderScore = 0;
    uint8_t gridScore = 0;
    uint8_t validColorCells = 0;
    uint8_t rejectionCode = 0;
    Point corners[4];
    Line lines[4];
    float homography[9] = {0.0f};
    CellResult cells[CELL_COUNT];
    uint16_t processingMilliseconds = 0;
    uint16_t framesPerSecondTenths = 0;
};

class Pipeline {
public:
    bool begin();
    bool process(const camera_fb_t *frame, Result &result);
    bool renderPreview(const camera_fb_t *frame, const Result &result,
                       uint8_t *output, size_t outputLength) const;
    void drawDebugOverlay(camera_fb_t *frame, const Result &result) const;
};

}  // namespace vision
