#pragma once

#include <stdint.h>

namespace vision_config {

// Change these thresholds and rebuild the firmware.
constexpr uint8_t EDGE_MINIMUM = 35;
constexpr uint8_t EDGE_DIVISOR = 4;

// Black frame test in HSV. Dark colored tiles fail the saturation test.
constexpr uint8_t BLACK_MAX_VALUE = 90;
constexpr uint8_t BLACK_MAX_SATURATION = 140;

// Geometric validation in the 160 x 120 analysis image.
constexpr float MIN_QUADRILATERAL_AREA_RATIO = 0.08f;
constexpr float EXPECTED_ASPECT_RATIO = 1.333f;
constexpr float MAX_ASPECT_RATIO_ERROR = 0.75f;

// WRO frame and internal 4 x 3 structure validation.
constexpr uint8_t BORDER_SAMPLE_COUNT = 12;
constexpr uint8_t MIN_BLACK_BORDER_SCORE = 60;
constexpr uint8_t GRID_LINE_SAMPLE_COUNT = 12;
constexpr uint8_t MIN_GRID_STRUCTURE_SCORE = 35;

// A false rectangle normally has too few valid WRO colors inside.
constexpr uint8_t MIN_VALID_COLOR_CELLS = 8;

}  // namespace vision_config
