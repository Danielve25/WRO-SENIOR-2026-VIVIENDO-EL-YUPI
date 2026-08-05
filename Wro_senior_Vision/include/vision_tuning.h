#pragma once

#include <stdint.h>

// Quick color calibration file. Rebuild and upload after changing a value.
namespace vision_tuning {

// General color acceptance.
constexpr uint8_t COLOR_MIN_VALUE = 35;
constexpr uint8_t COLOR_MIN_SATURATION = 25;
constexpr uint8_t MIN_COLOR_CONFIDENCE = 35;

// White is accepted only when the channels are close together.
constexpr uint8_t WHITE_MIN_VALUE = 80;
constexpr float WHITE_MAX_CHROMA_RATIO = 0.10f;

// HSV centers used by the yellow, green and blue classifiers.
constexpr float YELLOW_HUE = 55.0f;
constexpr float GREEN_HUE = 165.0f;
constexpr float BLUE_HUE = 220.0f;

// Normalized RGB prototypes for the current OV2640 lighting.
constexpr float YELLOW_RED = 0.38f;
constexpr float YELLOW_GREEN = 0.43f;
constexpr float YELLOW_BLUE = 0.13f;
constexpr float GREEN_RED = 0.18f;
constexpr float GREEN_GREEN = 0.40f;
constexpr float GREEN_BLUE = 0.38f;
constexpr float BLUE_RED = 0.16f;
constexpr float BLUE_GREEN = 0.26f;
constexpr float BLUE_BLUE = 0.54f;

// Larger values make RGB matching more permissive.
constexpr float MAX_RGB_DISTANCE = 0.30f;
constexpr float SATURATION_FULL_SCALE = 140.0f;

}  // namespace vision_tuning
