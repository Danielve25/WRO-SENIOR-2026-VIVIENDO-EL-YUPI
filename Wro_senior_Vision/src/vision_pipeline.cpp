#include "vision_pipeline.h"
#include "vision_config.h"
#include "vision_tuning.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <esp_heap_caps.h>

namespace {

constexpr uint16_t THETA_COUNT = 90;
constexpr uint16_t RHO_BINS = 512;
constexpr int RHO_OFFSET = 256;
constexpr uint8_t MAX_LINES = 16;
constexpr float VISION_PI = 3.14159265358979323846f;

struct Rgb {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct Hsv {
    uint16_t hue;
    uint8_t saturation;
    uint8_t value;
};

struct CandidateLine {
    float theta;
    float rho;
    uint16_t score;
};

uint8_t *grayImage = nullptr;
uint8_t *edgeImage = nullptr;
uint8_t *darkImage = nullptr;
uint16_t *hough = nullptr;
uint16_t *floodQueue = nullptr;
CandidateLine lineCandidates[MAX_LINES];
bool buffersReady = false;

uint16_t &houghAt(uint16_t theta, uint16_t rho)
{
    return hough[static_cast<size_t>(theta) * RHO_BINS + rho];
}

float clampFloat(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

uint8_t clampByte(int value)
{
    return static_cast<uint8_t>(std::max(0, std::min(255, value)));
}

Rgb readRgb565(const camera_fb_t *frame, int x, int y)
{
    x = std::max(0, std::min(static_cast<int>(frame->width) - 1, x));
    y = std::max(0, std::min(static_cast<int>(frame->height) - 1, y));
    const size_t offset = (static_cast<size_t>(y) * frame->width + x) * 2U;
    const uint16_t packed = static_cast<uint16_t>(frame->buf[offset]) << 8 |
                            static_cast<uint16_t>(frame->buf[offset + 1]);
    return {
        static_cast<uint8_t>(((packed >> 11) & 0x1FU) * 255U / 31U),
        static_cast<uint8_t>(((packed >> 5) & 0x3FU) * 255U / 63U),
        static_cast<uint8_t>((packed & 0x1FU) * 255U / 31U)
    };
}

uint16_t toPackedRgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint16_t packed = static_cast<uint16_t>(red >> 3) << 11 |
                            static_cast<uint16_t>(green >> 2) << 5 |
                            static_cast<uint16_t>(blue >> 3);
    return packed;
}

void writeRgb565(uint8_t *buffer, size_t pixelIndex, uint16_t packed)
{
    buffer[pixelIndex * 2U] = static_cast<uint8_t>(packed >> 8);
    buffer[pixelIndex * 2U + 1U] = static_cast<uint8_t>(packed & 0xFFU);
}

uint8_t luminance(const Rgb &rgb)
{
    return static_cast<uint8_t>((77U * rgb.red + 150U * rgb.green +
                                 29U * rgb.blue) >> 8);
}

Hsv toHsv(const Rgb &rgb)
{
    const float red = rgb.red / 255.0f;
    const float green = rgb.green / 255.0f;
    const float blue = rgb.blue / 255.0f;
    const float maximum = std::max(red, std::max(green, blue));
    const float minimum = std::min(red, std::min(green, blue));
    const float delta = maximum - minimum;

    float hue = 0.0f;
    if (delta > 0.0001f) {
        if (maximum == red) {
            hue = 60.0f * std::fmod((green - blue) / delta, 6.0f);
        } else if (maximum == green) {
            hue = 60.0f * (((blue - red) / delta) + 2.0f);
        } else {
            hue = 60.0f * (((red - green) / delta) + 4.0f);
        }
        if (hue < 0.0f) {
            hue += 360.0f;
        }
    }

    const float saturation = maximum <= 0.0001f ? 0.0f : delta / maximum;
    return {
        static_cast<uint16_t>(clampFloat(hue, 0.0f, 359.0f)),
        static_cast<uint8_t>(clampFloat(saturation * 255.0f, 0.0f, 255.0f)),
        static_cast<uint8_t>(clampFloat(maximum * 255.0f, 0.0f, 255.0f))
    };
}

float hueDistance(float first, float second)
{
    float distance = std::fabs(first - second);
    if (distance > 180.0f) {
        distance = 360.0f - distance;
    }
    return distance;
}

float normalizedRgbDistance(const Rgb &rgb, float red, float green, float blue)
{
    const float sum = std::max(1.0f,
        static_cast<float>(rgb.red) + rgb.green + rgb.blue);
    const float normalizedRed = rgb.red / sum;
    const float normalizedGreen = rgb.green / sum;
    const float normalizedBlue = rgb.blue / sum;
    const float redDistance = normalizedRed - red;
    const float greenDistance = normalizedGreen - green;
    const float blueDistance = normalizedBlue - blue;
    return std::sqrt(redDistance * redDistance +
                     greenDistance * greenDistance +
                     blueDistance * blueDistance);
}

vision::Color classifyColor(const Rgb &rgb, const Hsv &hsv, uint8_t &confidence)
{
    const uint8_t maximum = std::max(rgb.red, std::max(rgb.green, rgb.blue));
    const uint8_t minimum = std::min(rgb.red, std::min(rgb.green, rgb.blue));
    const uint8_t chroma = maximum - minimum;
    const float chromaRatio = maximum == 0
        ? 1.0f
        : static_cast<float>(chroma) / maximum;

    if (maximum < vision_tuning::COLOR_MIN_VALUE) {
        confidence = 0;
        return vision::Color::Unknown;
    }

    if (maximum >= vision_tuning::WHITE_MIN_VALUE &&
        chromaRatio <= vision_tuning::WHITE_MAX_CHROMA_RATIO) {
        confidence = static_cast<uint8_t>(clampFloat(
            100.0f - chromaRatio * 180.0f -
            std::max(0.0f, 150.0f - maximum) * 0.20f,
            0.0f, 100.0f));
        return vision::Color::White;
    }

    if (hsv.saturation < vision_tuning::COLOR_MIN_SATURATION) {
        confidence = 0;
        return vision::Color::Unknown;
    }

    // The camera shifts the green tiles toward cyan under the current lighting.
    const float hues[3] = {
        vision_tuning::YELLOW_HUE,
        vision_tuning::GREEN_HUE,
        vision_tuning::BLUE_HUE
    };
    const float redPrototypes[3] = {
        vision_tuning::YELLOW_RED,
        vision_tuning::GREEN_RED,
        vision_tuning::BLUE_RED
    };
    const float greenPrototypes[3] = {
        vision_tuning::YELLOW_GREEN,
        vision_tuning::GREEN_GREEN,
        vision_tuning::BLUE_GREEN
    };
    const float bluePrototypes[3] = {
        vision_tuning::YELLOW_BLUE,
        vision_tuning::GREEN_BLUE,
        vision_tuning::BLUE_BLUE
    };
    const vision::Color colors[3] = {
        vision::Color::Yellow,
        vision::Color::Green,
        vision::Color::Blue
    };

    float bestDistance = 1000.0f;
    float bestScore = 0.0f;
    uint8_t bestIndex = 0;
    for (uint8_t index = 0; index < 3; index++) {
        const float hueScore = 1.0f - hueDistance(
            static_cast<float>(hsv.hue), hues[index]) / 180.0f;
        const float rgbDistance = normalizedRgbDistance(
            rgb, redPrototypes[index], greenPrototypes[index],
            bluePrototypes[index]);
        const float rgbScore = clampFloat(
            1.0f - rgbDistance / vision_tuning::MAX_RGB_DISTANCE,
                                          0.0f, 1.0f);
        const float score = (hueScore * 0.55f + rgbScore * 0.45f) *
                            clampFloat(
                                hsv.saturation / vision_tuning::SATURATION_FULL_SCALE,
                                0.45f, 1.0f);
        if (score > bestScore) {
            bestScore = score;
            bestDistance = rgbDistance;
            bestIndex = index;
        }
    }

    (void)bestDistance;
    confidence = static_cast<uint8_t>(clampFloat(bestScore * 100.0f,
                                                 0.0f, 100.0f));
    return colors[bestIndex];
}

void prepareGrayAndEdges(const camera_fb_t *frame)
{
    for (uint16_t y = 0; y < vision::ANALYSIS_HEIGHT; y++) {
        const int sourceY = static_cast<int>(
            static_cast<uint32_t>(y) * frame->height / vision::ANALYSIS_HEIGHT);
        for (uint16_t x = 0; x < vision::ANALYSIS_WIDTH; x++) {
            const int sourceX = static_cast<int>(
                static_cast<uint32_t>(x) * frame->width / vision::ANALYSIS_WIDTH);
            grayImage[y * vision::ANALYSIS_WIDTH + x] =
                luminance(readRgb565(frame, sourceX, sourceY));
        }
    }

    for (uint16_t index = 0;
         index < vision::ANALYSIS_WIDTH * vision::ANALYSIS_HEIGHT; index++) {
        darkImage[index] = grayImage[index] <= vision_config::BLACK_MAX_VALUE ? 1 : 0;
    }

    uint8_t maximumGradient = 0;
    for (uint16_t y = 1; y + 1 < vision::ANALYSIS_HEIGHT; y++) {
        for (uint16_t x = 1; x + 1 < vision::ANALYSIS_WIDTH; x++) {
            const int gx =
                -grayImage[(y - 1) * vision::ANALYSIS_WIDTH + x - 1] +
                grayImage[(y - 1) * vision::ANALYSIS_WIDTH + x + 1] -
                2 * grayImage[y * vision::ANALYSIS_WIDTH + x - 1] +
                2 * grayImage[y * vision::ANALYSIS_WIDTH + x + 1] -
                grayImage[(y + 1) * vision::ANALYSIS_WIDTH + x - 1] +
                grayImage[(y + 1) * vision::ANALYSIS_WIDTH + x + 1];
            const int gy =
                -grayImage[(y - 1) * vision::ANALYSIS_WIDTH + x - 1] -
                2 * grayImage[(y - 1) * vision::ANALYSIS_WIDTH + x] -
                grayImage[(y - 1) * vision::ANALYSIS_WIDTH + x + 1] +
                grayImage[(y + 1) * vision::ANALYSIS_WIDTH + x - 1] +
                2 * grayImage[(y + 1) * vision::ANALYSIS_WIDTH + x] +
                grayImage[(y + 1) * vision::ANALYSIS_WIDTH + x + 1];
            const uint8_t magnitude = static_cast<uint8_t>(
                std::min(255, std::abs(gx) + std::abs(gy)));
            edgeImage[y * vision::ANALYSIS_WIDTH + x] = magnitude;
            maximumGradient = std::max(maximumGradient, magnitude);
        }
    }

    const uint8_t threshold = std::max<uint8_t>(
        vision_config::EDGE_MINIMUM,
        static_cast<uint8_t>(maximumGradient / vision_config::EDGE_DIVISOR));
    for (uint16_t index = 0;
         index < vision::ANALYSIS_WIDTH * vision::ANALYSIS_HEIGHT; index++) {
        edgeImage[index] = edgeImage[index] >= threshold ? 1 : 0;
    }
}

void buildHoughAccumulator()
{
    std::memset(hough, 0, static_cast<size_t>(THETA_COUNT) * RHO_BINS * sizeof(uint16_t));
    static float cosines[THETA_COUNT];
    static float sines[THETA_COUNT];
    static bool initialized = false;
    if (!initialized) {
        for (uint16_t theta = 0; theta < THETA_COUNT; theta++) {
            const float radians = theta * VISION_PI / THETA_COUNT;
            cosines[theta] = std::cos(radians);
            sines[theta] = std::sin(radians);
        }
        initialized = true;
    }

    for (uint16_t y = 1; y + 1 < vision::ANALYSIS_HEIGHT; y++) {
        for (uint16_t x = 1; x + 1 < vision::ANALYSIS_WIDTH; x++) {
            if (edgeImage[y * vision::ANALYSIS_WIDTH + x] == 0) {
                continue;
            }
            for (uint16_t theta = 0; theta < THETA_COUNT; theta++) {
                const float rho = x * cosines[theta] + y * sines[theta];
                const int bin = static_cast<int>(std::lround(rho)) + RHO_OFFSET;
                if (bin >= 0 && bin < RHO_BINS && houghAt(theta, bin) < 0xFFFFU) {
                    houghAt(theta, bin)++;
                }
            }
        }
    }
}

float angleDifferenceDegrees(float first, float second)
{
    float difference = std::fabs(first - second) * 180.0f / VISION_PI;
    if (difference > 90.0f) {
        difference = 180.0f - difference;
    }
    return difference;
}

uint8_t extractLineCandidates()
{
    uint8_t count = 0;
    while (count < MAX_LINES) {
        uint16_t bestScore = 0;
        uint16_t bestTheta = 0;
        uint16_t bestRho = 0;

        for (uint16_t theta = 0; theta < THETA_COUNT; theta++) {
            for (uint16_t rho = 0; rho < RHO_BINS; rho++) {
                if (houghAt(theta, rho) <= bestScore) {
                    continue;
                }

                bool suppressed = false;
                const float angle = theta * VISION_PI / THETA_COUNT;
                const float distance = static_cast<float>(rho - RHO_OFFSET);
                for (uint8_t previous = 0; previous < count; previous++) {
                    if (angleDifferenceDegrees(angle,
                            lineCandidates[previous].theta) < 6.0f &&
                        std::fabs(distance - lineCandidates[previous].rho) < 12.0f) {
                        suppressed = true;
                        break;
                    }
                }
                if (!suppressed) {
                    bestScore = houghAt(theta, rho);
                    bestTheta = theta;
                    bestRho = rho;
                }
            }
        }

        if (bestScore < 18U) {
            break;
        }
        lineCandidates[count++] = {
            bestTheta * VISION_PI / THETA_COUNT,
            static_cast<float>(bestRho - RHO_OFFSET),
            bestScore
        };
    }
    return count;
}

bool intersect(const CandidateLine &first, const CandidateLine &second,
               vision::Point &point)
{
    const float firstA = std::cos(first.theta);
    const float firstB = std::sin(first.theta);
    const float secondA = std::cos(second.theta);
    const float secondB = std::sin(second.theta);
    const float determinant = firstA * secondB - secondA * firstB;
    if (std::fabs(determinant) < 0.08f) {
        return false;
    }
    point.x = (first.rho * secondB - second.rho * firstB) / determinant;
    point.y = (firstA * second.rho - secondA * first.rho) / determinant;
    return true;
}

float distance(const vision::Point &first, const vision::Point &second)
{
    const float dx = first.x - second.x;
    const float dy = first.y - second.y;
    return std::sqrt(dx * dx + dy * dy);
}

float polygonArea(const vision::Point points[4])
{
    float area = 0.0f;
    for (uint8_t index = 0; index < 4; index++) {
        const uint8_t next = (index + 1) % 4;
        area += points[index].x * points[next].y -
                points[next].x * points[index].y;
    }
    return std::fabs(area) * 0.5f;
}

bool isConvex(const vision::Point points[4])
{
    int sign = 0;
    for (uint8_t index = 0; index < 4; index++) {
        const vision::Point &a = points[index];
        const vision::Point &b = points[(index + 1) % 4];
        const vision::Point &c = points[(index + 2) % 4];
        const float cross = (b.x - a.x) * (c.y - b.y) -
                            (b.y - a.y) * (c.x - b.x);
        if (std::fabs(cross) < 0.01f) {
            return false;
        }
        const int currentSign = cross > 0.0f ? 1 : -1;
        if (sign == 0) {
            sign = currentSign;
        } else if (sign != currentSign) {
            return false;
        }
    }
    return true;
}

void orderCorners(vision::Point points[4])
{
    vision::Point centroid{0.0f, 0.0f};
    for (uint8_t index = 0; index < 4; index++) {
        centroid.x += points[index].x;
        centroid.y += points[index].y;
    }
    centroid.x /= 4.0f;
    centroid.y /= 4.0f;

    std::sort(points, points + 4, [centroid](const vision::Point &first,
                                             const vision::Point &second) {
        return std::atan2(first.y - centroid.y, first.x - centroid.x) <
               std::atan2(second.y - centroid.y, second.x - centroid.x);
    });

    uint8_t topLeft = 0;
    float bestSum = points[0].x + points[0].y;
    for (uint8_t index = 1; index < 4; index++) {
        const float sum = points[index].x + points[index].y;
        if (sum < bestSum) {
            bestSum = sum;
            topLeft = index;
        }
    }

    vision::Point ordered[4];
    for (uint8_t index = 0; index < 4; index++) {
        ordered[index] = points[(topLeft + index) % 4];
    }
    std::memcpy(points, ordered, sizeof(ordered));
}

bool solveHomography(const vision::Point source[4], float homography[9]);
bool project(const float homography[9], float x, float y, vision::Point &point);
uint8_t calculateBorderScore(const camera_fb_t *frame,
                             const vision::Point corners[4]);
uint8_t calculateGridScore(const camera_fb_t *frame,
                           const vision::Point corners[4]);
bool detectDarkComponent(const camera_fb_t *frame, vision::Result &result);

bool detectQuadrilateral(const camera_fb_t *frame, vision::Result &result)
{
    const uint8_t candidateCount = extractLineCandidates();
    float bestScore = -1.0f;
    vision::Point bestCorners[4];
    CandidateLine bestLines[4];
    uint8_t bestBorderScore = 0;
    uint8_t bestGridScore = 0;

    for (uint8_t first = 0; first + 3 < candidateCount; first++) {
        for (uint8_t second = first + 1; second + 2 < candidateCount; second++) {
            const float firstPairAngle = angleDifferenceDegrees(
                lineCandidates[first].theta, lineCandidates[second].theta);
            if (firstPairAngle > 15.0f ||
                std::fabs(lineCandidates[first].rho - lineCandidates[second].rho) < 20.0f) {
                continue;
            }

            for (uint8_t third = 0; third + 1 < candidateCount; third++) {
                if (third == first || third == second) {
                    continue;
                }
                for (uint8_t fourth = third + 1; fourth < candidateCount; fourth++) {
                    if (fourth == first || fourth == second) {
                        continue;
                    }

                    const float secondPairAngle = angleDifferenceDegrees(
                        lineCandidates[third].theta, lineCandidates[fourth].theta);
                    if (secondPairAngle > 15.0f ||
                        std::fabs(lineCandidates[third].rho - lineCandidates[fourth].rho) < 20.0f) {
                        continue;
                    }

                    const float pairAngle = angleDifferenceDegrees(
                        (lineCandidates[first].theta + lineCandidates[second].theta) * 0.5f,
                        (lineCandidates[third].theta + lineCandidates[fourth].theta) * 0.5f);
                    if (pairAngle < 55.0f || pairAngle > 125.0f) {
                        continue;
                    }

                    CandidateLine aNear = lineCandidates[first];
                    CandidateLine aFar = lineCandidates[second];
                    CandidateLine bNear = lineCandidates[third];
                    CandidateLine bFar = lineCandidates[fourth];
                    if (aNear.rho > aFar.rho) std::swap(aNear, aFar);
                    if (bNear.rho > bFar.rho) std::swap(bNear, bFar);

                    vision::Point corners[4];
                    if (!intersect(aNear, bNear, corners[0]) ||
                        !intersect(aNear, bFar, corners[1]) ||
                        !intersect(aFar, bFar, corners[2]) ||
                        !intersect(aFar, bNear, corners[3])) {
                        continue;
                    }
                    if (!isConvex(corners)) {
                        continue;
                    }

                    bool inside = true;
                    for (uint8_t index = 0; index < 4; index++) {
                        inside = inside && corners[index].x > -20.0f &&
                                 corners[index].x < vision::ANALYSIS_WIDTH + 20.0f &&
                                 corners[index].y > -20.0f &&
                                 corners[index].y < vision::ANALYSIS_HEIGHT + 20.0f;
                    }
                    if (!inside) {
                        continue;
                    }

                    orderCorners(corners);
                    const float area = polygonArea(corners);
                    if (area < vision::ANALYSIS_WIDTH * vision::ANALYSIS_HEIGHT *
                              vision_config::MIN_QUADRILATERAL_AREA_RATIO) {
                        continue;
                    }

                    const float width = (distance(corners[0], corners[1]) +
                                         distance(corners[2], corners[3])) * 0.5f;
                    const float height = (distance(corners[1], corners[2]) +
                                          distance(corners[3], corners[0])) * 0.5f;
                    if (width < 20.0f || height < 20.0f) {
                        continue;
                    }

                    const float ratio = width > height ? width / height : height / width;
                    const float ratioError = std::fabs(
                        ratio - vision_config::EXPECTED_ASPECT_RATIO) /
                        vision_config::EXPECTED_ASPECT_RATIO;
                    if (ratioError > vision_config::MAX_ASPECT_RATIO_ERROR) {
                        continue;
                    }
                    const float ratioScore = 1.0f - ratioError;
                    const uint8_t borderScore = calculateBorderScore(frame, corners);
                    const uint8_t gridScore = calculateGridScore(frame, corners);
                    const float lineScore = static_cast<float>(
                        aNear.score + aFar.score + bNear.score + bFar.score);
                    const float score = lineScore * 2.0f + area * 0.25f +
                                        ratioScore * 100.0f +
                                        borderScore * 4.0f + gridScore * 3.0f;
                    if (score > bestScore) {
                        bestScore = score;
                        std::memcpy(bestCorners, corners, sizeof(bestCorners));
                        bestLines[0] = aNear;
                        bestLines[1] = aFar;
                        bestLines[2] = bNear;
                        bestLines[3] = bFar;
                        bestBorderScore = borderScore;
                        bestGridScore = gridScore;
                    }
                }
            }
        }
    }

    if (bestScore < 0.0f) {
        if (detectDarkComponent(frame, result)) {
            return true;
        }
        result.candidateFound = false;
        result.rejectionCode = 1;
        return false;
    }

    result.candidateFound = true;
    std::memcpy(result.corners, bestCorners, sizeof(bestCorners));
    for (uint8_t index = 0; index < 4; index++) {
        result.lines[index].theta = bestLines[index].theta;
        result.lines[index].rho = bestLines[index].rho;
        result.lines[index].score = bestLines[index].score;
    }
    result.borderScore = bestBorderScore;
    result.gridScore = bestGridScore;
    result.confidence = static_cast<uint8_t>(clampFloat(
        bestBorderScore * 0.4f + bestGridScore * 0.4f +
        bestScore / 40.0f, 0.0f, 100.0f));
    result.rejectionCode = 0;
    if (bestBorderScore < vision_config::MIN_BLACK_BORDER_SCORE) {
        result.rejectionCode |= 2;
    }
    if (bestGridScore < vision_config::MIN_GRID_STRUCTURE_SCORE) {
        result.rejectionCode |= 4;
    }
    result.valid = result.rejectionCode == 0;
    if (!result.valid) {
        vision::Result fallback;
        if (detectDarkComponent(frame, fallback) && fallback.valid) {
            result = fallback;
        }
    }
    return true;
}

bool solveHomography(const vision::Point source[4], float homography[9])
{
    const float destination[4][2] = {
        {0.0f, 0.0f},
        {vision::ANALYSIS_WIDTH - 1.0f, 0.0f},
        {vision::ANALYSIS_WIDTH - 1.0f, vision::ANALYSIS_HEIGHT - 1.0f},
        {0.0f, vision::ANALYSIS_HEIGHT - 1.0f}
    };
    float matrix[8][9] = {};

    for (uint8_t index = 0; index < 4; index++) {
        const float u = destination[index][0];
        const float v = destination[index][1];
        const float x = source[index].x;
        const float y = source[index].y;
        const uint8_t row = index * 2;
        matrix[row][0] = u;
        matrix[row][1] = v;
        matrix[row][2] = 1.0f;
        matrix[row][6] = -x * u;
        matrix[row][7] = -x * v;
        matrix[row][8] = x;
        matrix[row + 1][3] = u;
        matrix[row + 1][4] = v;
        matrix[row + 1][5] = 1.0f;
        matrix[row + 1][6] = -y * u;
        matrix[row + 1][7] = -y * v;
        matrix[row + 1][8] = y;
    }

    for (uint8_t column = 0; column < 8; column++) {
        uint8_t pivot = column;
        for (uint8_t row = column + 1; row < 8; row++) {
            if (std::fabs(matrix[row][column]) >
                std::fabs(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (std::fabs(matrix[pivot][column]) < 0.00001f) {
            return false;
        }
        if (pivot != column) {
            for (uint8_t item = column; item < 9; item++) {
                std::swap(matrix[pivot][item], matrix[column][item]);
            }
        }
        const float divisor = matrix[column][column];
        for (uint8_t item = column; item < 9; item++) {
            matrix[column][item] /= divisor;
        }
        for (uint8_t row = 0; row < 8; row++) {
            if (row == column) {
                continue;
            }
            const float factor = matrix[row][column];
            for (uint8_t item = column; item < 9; item++) {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }

    for (uint8_t index = 0; index < 8; index++) {
        homography[index] = matrix[index][8];
    }
    homography[8] = 1.0f;
    return true;
}

bool project(const float homography[9], float x, float y, vision::Point &point)
{
    const float denominator = homography[6] * x + homography[7] * y + homography[8];
    if (std::fabs(denominator) < 0.00001f) {
        return false;
    }
    point.x = (homography[0] * x + homography[1] * y + homography[2]) / denominator;
    point.y = (homography[3] * x + homography[4] * y + homography[5]) / denominator;
    return true;
}

bool isBlackPixel(const Rgb &rgb)
{
    const Hsv hsv = toHsv(rgb);
    return hsv.value <= vision_config::BLACK_MAX_VALUE &&
           hsv.saturation <= vision_config::BLACK_MAX_SATURATION;
}

bool isBlackNearAnalysisPoint(const camera_fb_t *frame,
                              const vision::Point &point)
{
    const int centerX = static_cast<int>(std::lround(
        point.x * frame->width / vision::ANALYSIS_WIDTH));
    const int centerY = static_cast<int>(std::lround(
        point.y * frame->height / vision::ANALYSIS_HEIGHT));

    for (int offsetY = -2; offsetY <= 2; offsetY++) {
        for (int offsetX = -2; offsetX <= 2; offsetX++) {
            if (isBlackPixel(readRgb565(frame, centerX + offsetX,
                                        centerY + offsetY))) {
                return true;
            }
        }
    }
    return false;
}

uint8_t calculateBorderScore(const camera_fb_t *frame,
                             const vision::Point corners[4])
{
    uint16_t blackSamples = 0;
    const uint16_t totalSamples = 4U * vision_config::BORDER_SAMPLE_COUNT;
    for (uint8_t side = 0; side < 4; side++) {
        const vision::Point &first = corners[side];
        const vision::Point &second = corners[(side + 1) % 4];
        for (uint8_t sample = 0; sample < vision_config::BORDER_SAMPLE_COUNT; sample++) {
            const float t = (sample + 1.0f) /
                            (vision_config::BORDER_SAMPLE_COUNT + 1.0f);
            const vision::Point point(
                first.x + (second.x - first.x) * t,
                first.y + (second.y - first.y) * t);
            if (isBlackNearAnalysisPoint(frame, point)) {
                blackSamples++;
            }
        }
    }
    return static_cast<uint8_t>(blackSamples * 100U / totalSamples);
}

uint8_t calculateGridScore(const camera_fb_t *frame,
                           const vision::Point corners[4])
{
    float homography[9];
    if (!solveHomography(corners, homography)) {
        return 0;
    }

    uint16_t blackSamples = 0;
    uint16_t totalSamples = 0;
    for (uint8_t column = 1; column < vision::GRID_COLUMNS; column++) {
        const float x = column * vision::ANALYSIS_WIDTH / vision::GRID_COLUMNS;
        for (uint8_t sample = 0; sample < vision_config::GRID_LINE_SAMPLE_COUNT; sample++) {
            const float y = (sample + 1.0f) * vision::ANALYSIS_HEIGHT /
                            (vision_config::GRID_LINE_SAMPLE_COUNT + 1.0f);
            bool black = false;
            for (int offset = -2; offset <= 2; offset++) {
                vision::Point source;
                if (project(homography, x + offset, y, source) &&
                    isBlackNearAnalysisPoint(frame, source)) {
                    black = true;
                    break;
                }
            }
            if (black) blackSamples++;
            totalSamples++;
        }
    }
    for (uint8_t row = 1; row < vision::GRID_ROWS; row++) {
        const float y = row * vision::ANALYSIS_HEIGHT / vision::GRID_ROWS;
        for (uint8_t sample = 0; sample < vision_config::GRID_LINE_SAMPLE_COUNT; sample++) {
            const float x = (sample + 1.0f) * vision::ANALYSIS_WIDTH /
                            (vision_config::GRID_LINE_SAMPLE_COUNT + 1.0f);
            bool black = false;
            for (int offset = -2; offset <= 2; offset++) {
                vision::Point source;
                if (project(homography, x, y + offset, source) &&
                    isBlackNearAnalysisPoint(frame, source)) {
                    black = true;
                    break;
                }
            }
            if (black) blackSamples++;
            totalSamples++;
        }
    }

    return totalSamples == 0 ? 0 :
        static_cast<uint8_t>(blackSamples * 100U / totalSamples);
}

void makeLineFromPoints(const vision::Point &first, const vision::Point &second,
                        vision::Line &line)
{
    float theta = std::atan2(second.y - first.y, second.x - first.x) +
                  VISION_PI * 0.5f;
    while (theta < 0.0f) theta += VISION_PI;
    while (theta >= VISION_PI) theta -= VISION_PI;
    line.theta = theta;
    line.rho = first.x * std::cos(theta) + first.y * std::sin(theta);
    line.score = 0;
}

void setCandidateValidation(const camera_fb_t *frame,
                            const vision::Point corners[4],
                            vision::Result &result, float geometryScore)
{
    std::memcpy(result.corners, corners, sizeof(result.corners));
    for (uint8_t index = 0; index < 4; index++) {
        makeLineFromPoints(corners[index], corners[(index + 1) % 4],
                           result.lines[index]);
    }
    result.candidateFound = true;
    result.borderScore = calculateBorderScore(frame, corners);
    result.gridScore = calculateGridScore(frame, corners);
    result.confidence = static_cast<uint8_t>(clampFloat(
        result.borderScore * 0.4f + result.gridScore * 0.4f +
        geometryScore, 0.0f, 100.0f));
    result.rejectionCode = 0;
    if (result.borderScore < vision_config::MIN_BLACK_BORDER_SCORE) {
        result.rejectionCode |= 2;
    }
    if (result.gridScore < vision_config::MIN_GRID_STRUCTURE_SCORE) {
        result.rejectionCode |= 4;
    }
    result.valid = result.rejectionCode == 0;
}

bool detectDarkComponent(const camera_fb_t *frame, vision::Result &result)
{
    bool found = false;
    float bestScore = -1.0f;
    vision::Point bestCorners[4];
    uint8_t bestBorder = 0;
    uint8_t bestGrid = 0;
    float bestGeometry = 0.0f;

    for (uint16_t start = 0;
         start < vision::ANALYSIS_WIDTH * vision::ANALYSIS_HEIGHT; start++) {
        if (darkImage[start] != 1) continue;

        uint16_t queueLength = 0;
        uint16_t queueRead = 0;
        floodQueue[queueLength++] = start;
        darkImage[start] = 2;
        vision::Point minSum;
        vision::Point maxSum;
        vision::Point minDifference;
        vision::Point maxDifference;
        bool firstPoint = true;

        while (queueRead < queueLength) {
            const uint16_t current = floodQueue[queueRead++];
            const uint16_t x = current % vision::ANALYSIS_WIDTH;
            const uint16_t y = current / vision::ANALYSIS_WIDTH;
            const vision::Point point(static_cast<float>(x), static_cast<float>(y));
            const float sum = point.x + point.y;
            const float difference = point.x - point.y;
            if (firstPoint) {
                minSum = maxSum = minDifference = maxDifference = point;
                firstPoint = false;
            } else {
                if (sum < minSum.x + minSum.y) minSum = point;
                if (sum > maxSum.x + maxSum.y) maxSum = point;
                if (difference < minDifference.x - minDifference.y) minDifference = point;
                if (difference > maxDifference.x - maxDifference.y) maxDifference = point;
            }

            for (int offsetY = -1; offsetY <= 1; offsetY++) {
                for (int offsetX = -1; offsetX <= 1; offsetX++) {
                    if (offsetX == 0 && offsetY == 0) continue;
                    const int neighborX = static_cast<int>(x) + offsetX;
                    const int neighborY = static_cast<int>(y) + offsetY;
                    if (neighborX < 0 || neighborY < 0 ||
                        neighborX >= vision::ANALYSIS_WIDTH ||
                        neighborY >= vision::ANALYSIS_HEIGHT) continue;
                    const uint16_t neighbor = static_cast<uint16_t>(
                        neighborY * vision::ANALYSIS_WIDTH + neighborX);
                    if (darkImage[neighbor] == 1) {
                        darkImage[neighbor] = 2;
                        floodQueue[queueLength++] = neighbor;
                    }
                }
            }
        }

        if (queueLength < 100) continue;

        vision::Point corners[4] = {
            minSum,
            maxDifference,
            maxSum,
            minDifference
        };
        orderCorners(corners);
        if (!isConvex(corners)) continue;

        const float area = polygonArea(corners);
        if (area < vision::ANALYSIS_WIDTH * vision::ANALYSIS_HEIGHT *
                  vision_config::MIN_QUADRILATERAL_AREA_RATIO) continue;

        const float width = (distance(corners[0], corners[1]) +
                             distance(corners[2], corners[3])) * 0.5f;
        const float height = (distance(corners[1], corners[2]) +
                              distance(corners[3], corners[0])) * 0.5f;
        if (width < 20.0f || height < 20.0f) continue;

        const float ratio = width > height ? width / height : height / width;
        const float ratioError = std::fabs(
            ratio - vision_config::EXPECTED_ASPECT_RATIO) /
            vision_config::EXPECTED_ASPECT_RATIO;
        if (ratioError > vision_config::MAX_ASPECT_RATIO_ERROR) continue;

        const uint8_t border = calculateBorderScore(frame, corners);
        const uint8_t grid = calculateGridScore(frame, corners);
        const float score = area * 0.25f + border * 4.0f + grid * 3.0f;
        if (score > bestScore) {
            bestScore = score;
            std::memcpy(bestCorners, corners, sizeof(bestCorners));
            bestBorder = border;
            bestGrid = grid;
            bestGeometry = (1.0f - ratioError) * 20.0f;
            found = true;
        }
    }

    if (!found) return false;

    setCandidateValidation(frame, bestCorners, result, bestGeometry);
    result.borderScore = bestBorder;
    result.gridScore = bestGrid;
    return true;
}

uint16_t colorMarker(vision::Color color)
{
    switch (color) {
        case vision::Color::Yellow: return 0xFFE0;
        case vision::Color::Blue: return 0x001F;
        case vision::Color::Green: return 0x07E0;
        case vision::Color::White: return 0xFFFF;
        default: return 0xF800;
    }
}

void drawPixel(camera_fb_t *frame, int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= static_cast<int>(frame->width) ||
        y >= static_cast<int>(frame->height)) {
        return;
    }
    const size_t index = (static_cast<size_t>(y) * frame->width + x) * 2U;
    frame->buf[index] = static_cast<uint8_t>(color >> 8);
    frame->buf[index + 1U] = static_cast<uint8_t>(color & 0xFFU);
}

void drawPixel(uint8_t *buffer, int width, int height, int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
    }
    const size_t index = static_cast<size_t>(y) * width + x;
    writeRgb565(buffer, index, color);
}

void drawLine(camera_fb_t *frame, int x0, int y0, int x1, int y1, uint16_t color)
{
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
        drawPixel(frame, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        const int twice = 2 * error;
        if (twice >= dy) { error += dy; x0 += sx; }
        if (twice <= dx) { error += dx; y0 += sy; }
    }
}

void drawLine(uint8_t *buffer, int width, int height,
              int x0, int y0, int x1, int y1, uint16_t color)
{
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
        drawPixel(buffer, width, height, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        const int twice = 2 * error;
        if (twice >= dy) { error += dy; x0 += sx; }
        if (twice <= dx) { error += dx; y0 += sy; }
    }
}

void drawCircle(camera_fb_t *frame, int centerX, int centerY, int radius, uint16_t color)
{
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                drawPixel(frame, centerX + x, centerY + y, color);
            }
        }
    }
}

void drawCircle(uint8_t *buffer, int width, int height,
                int centerX, int centerY, int radius, uint16_t color)
{
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                drawPixel(buffer, width, height, centerX + x, centerY + y, color);
            }
        }
    }
}

vision::Point toFramePoint(const vision::Point &point, const camera_fb_t *frame)
{
    return {
        point.x * frame->width / vision::ANALYSIS_WIDTH,
        point.y * frame->height / vision::ANALYSIS_HEIGHT
    };
}

bool lineRectangleIntersections(const CandidateLine &line,
                               vision::Point &first, vision::Point &second)
{
    const float a = std::cos(line.theta);
    const float b = std::sin(line.theta);
    const float width = vision::ANALYSIS_WIDTH - 1.0f;
    const float height = vision::ANALYSIS_HEIGHT - 1.0f;
    vision::Point intersections[4];
    uint8_t count = 0;

    if (std::fabs(b) > 0.0001f) {
        const float yAtZero = line.rho / b;
        const float yAtWidth = (line.rho - a * width) / b;
        if (yAtZero >= 0.0f && yAtZero <= height) intersections[count++] = {0.0f, yAtZero};
        if (yAtWidth >= 0.0f && yAtWidth <= height) intersections[count++] = {width, yAtWidth};
    }
    if (std::fabs(a) > 0.0001f) {
        const float xAtZero = line.rho / a;
        const float xAtHeight = (line.rho - b * height) / a;
        if (xAtZero >= 0.0f && xAtZero <= width) intersections[count++] = {xAtZero, 0.0f};
        if (xAtHeight >= 0.0f && xAtHeight <= width) intersections[count++] = {xAtHeight, height};
    }

    if (count < 2) return false;
    first = intersections[0];
    second = intersections[1];
    return true;
}

void drawProjectedLine(camera_fb_t *frame, const float homography[9],
                       float x0, float y0, float x1, float y1, uint16_t color)
{
    vision::Point first;
    vision::Point second;
    if (!project(homography, x0, y0, first) || !project(homography, x1, y1, second)) {
        return;
    }
    const vision::Point frameFirst = toFramePoint(first, frame);
    const vision::Point frameSecond = toFramePoint(second, frame);
    drawLine(frame, static_cast<int>(frameFirst.x), static_cast<int>(frameFirst.y),
             static_cast<int>(frameSecond.x), static_cast<int>(frameSecond.y), color);
}

}  // namespace

namespace vision {

bool Pipeline::begin()
{
    if (buffersReady) {
        return true;
    }

    const size_t pixelCount = static_cast<size_t>(ANALYSIS_WIDTH) * ANALYSIS_HEIGHT;
    grayImage = static_cast<uint8_t *>(heap_caps_malloc(pixelCount, MALLOC_CAP_SPIRAM));
    edgeImage = static_cast<uint8_t *>(heap_caps_malloc(pixelCount, MALLOC_CAP_SPIRAM));
    darkImage = static_cast<uint8_t *>(heap_caps_malloc(pixelCount, MALLOC_CAP_SPIRAM));
    floodQueue = static_cast<uint16_t *>(heap_caps_malloc(
        pixelCount * sizeof(uint16_t), MALLOC_CAP_SPIRAM));
    hough = static_cast<uint16_t *>(heap_caps_malloc(
        static_cast<size_t>(THETA_COUNT) * RHO_BINS * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM));

    if (grayImage == nullptr || edgeImage == nullptr || darkImage == nullptr ||
        floodQueue == nullptr || hough == nullptr) {
        if (grayImage != nullptr) heap_caps_free(grayImage);
        if (edgeImage != nullptr) heap_caps_free(edgeImage);
        if (darkImage != nullptr) heap_caps_free(darkImage);
        if (floodQueue != nullptr) heap_caps_free(floodQueue);
        if (hough != nullptr) heap_caps_free(hough);
        grayImage = nullptr;
        edgeImage = nullptr;
        darkImage = nullptr;
        floodQueue = nullptr;
        hough = nullptr;
        return false;
    }

    buffersReady = true;
    return true;
}

bool Pipeline::process(const camera_fb_t *frame, Result &result)
{
    std::memset(&result, 0, sizeof(result));
    if (!buffersReady || frame == nullptr || frame->format != PIXFORMAT_RGB565 ||
        frame->width < ANALYSIS_WIDTH || frame->height < ANALYSIS_HEIGHT) {
        return false;
    }

    const uint32_t start = millis();
    prepareGrayAndEdges(frame);
    buildHoughAccumulator();
    const bool candidateFound = detectQuadrilateral(frame, result);
    if (!candidateFound || !solveHomography(result.corners, result.homography)) {
        result.valid = false;
        result.rejectionCode |= 16;
        result.processingMilliseconds = static_cast<uint16_t>(millis() - start);
        return false;
    }

    if (!result.valid) {
        result.processingMilliseconds = static_cast<uint16_t>(millis() - start);
        return false;
    }

    for (uint8_t row = 0; row < GRID_ROWS; row++) {
        for (uint8_t column = 0; column < GRID_COLUMNS; column++) {
            const uint8_t index = row * GRID_COLUMNS + column;
            const float centerX = (column + 0.5f) * ANALYSIS_WIDTH / GRID_COLUMNS;
            const float centerY = (row + 0.5f) * ANALYSIS_HEIGHT / GRID_ROWS;
            result.cells[index].sampleX = static_cast<uint16_t>(centerX);
            result.cells[index].sampleY = static_cast<uint16_t>(centerY);

            uint32_t red = 0;
            uint32_t green = 0;
            uint32_t blue = 0;
            uint16_t samples = 0;
            for (int offsetY = -6; offsetY <= 6; offsetY += 3) {
                for (int offsetX = -8; offsetX <= 8; offsetX += 4) {
                    Point source;
                    if (!project(result.homography, centerX + offsetX,
                                 centerY + offsetY, source)) {
                        continue;
                    }
                    const int frameX = static_cast<int>(std::lround(
                        source.x * frame->width / ANALYSIS_WIDTH));
                    const int frameY = static_cast<int>(std::lround(
                        source.y * frame->height / ANALYSIS_HEIGHT));
                    const Rgb rgb = readRgb565(frame, frameX, frameY);
                    red += rgb.red;
                    green += rgb.green;
                    blue += rgb.blue;
                    samples++;
                }
            }

            if (samples == 0) continue;
            const Rgb average{
                static_cast<uint8_t>(red / samples),
                static_cast<uint8_t>(green / samples),
                static_cast<uint8_t>(blue / samples)
            };
            const Hsv hsv = toHsv(average);
            result.cells[index].red = average.red;
            result.cells[index].green = average.green;
            result.cells[index].blue = average.blue;
            result.cells[index].hue = hsv.hue;
            result.cells[index].saturation = hsv.saturation;
            result.cells[index].value = hsv.value;
            result.cells[index].color = classifyColor(
                average, hsv, result.cells[index].confidence);
        }
    }

    for (uint8_t index = 0; index < CELL_COUNT; index++) {
        if (result.cells[index].color != Color::Unknown &&
             result.cells[index].confidence >= vision_tuning::MIN_COLOR_CONFIDENCE) {
            result.validColorCells++;
        }
    }
    if (result.validColorCells < vision_config::MIN_VALID_COLOR_CELLS) {
        result.valid = false;
        result.rejectionCode |= 8;
    }

    result.processingMilliseconds = static_cast<uint16_t>(millis() - start);
    return result.valid;
}

bool Pipeline::renderPreview(const camera_fb_t *frame, const Result &result,
                             uint8_t *output, size_t outputLength) const
{
    const size_t required = static_cast<size_t>(ANALYSIS_WIDTH) *
                            ANALYSIS_HEIGHT * 2U;
    if (frame == nullptr || output == nullptr || outputLength < required ||
        !result.valid) {
        return false;
    }

    for (uint16_t y = 0; y < ANALYSIS_HEIGHT; y++) {
        for (uint16_t x = 0; x < ANALYSIS_WIDTH; x++) {
            Point source;
            if (!project(result.homography, x, y, source)) continue;
            const int sourceX = static_cast<int>(std::lround(
                source.x * frame->width / ANALYSIS_WIDTH));
            const int sourceY = static_cast<int>(std::lround(
                source.y * frame->height / ANALYSIS_HEIGHT));
            const Rgb rgb = readRgb565(frame, sourceX, sourceY);
            writeRgb565(output, static_cast<size_t>(y) * ANALYSIS_WIDTH + x,
                        toPackedRgb565(rgb.red, rgb.green, rgb.blue));
        }
    }

    for (uint8_t column = 0; column <= GRID_COLUMNS; column++) {
        const int x = column * ANALYSIS_WIDTH / GRID_COLUMNS;
        drawLine(output, ANALYSIS_WIDTH, ANALYSIS_HEIGHT, x, 0, x,
                 ANALYSIS_HEIGHT - 1, 0x07E0);
    }
    for (uint8_t row = 0; row <= GRID_ROWS; row++) {
        const int y = row * ANALYSIS_HEIGHT / GRID_ROWS;
        drawLine(output, ANALYSIS_WIDTH, ANALYSIS_HEIGHT, 0, y,
                 ANALYSIS_WIDTH - 1, y, 0x07E0);
    }
    for (uint8_t index = 0; index < CELL_COUNT; index++) {
        drawCircle(output, ANALYSIS_WIDTH, ANALYSIS_HEIGHT,
                   result.cells[index].sampleX, result.cells[index].sampleY,
                   3, colorMarker(result.cells[index].color));
    }
    return true;
}

void Pipeline::drawDebugOverlay(camera_fb_t *frame, const Result &result) const
{
    if (frame == nullptr || !result.candidateFound) return;

    const uint16_t candidateColor = result.valid ? 0x07E0 : 0xFD20;

    for (uint8_t index = 0; index < 4; index++) {
        const Point first = toFramePoint(result.corners[index], frame);
        const Point second = toFramePoint(result.corners[(index + 1) % 4], frame);
        drawLine(frame, static_cast<int>(first.x), static_cast<int>(first.y),
                 static_cast<int>(second.x), static_cast<int>(second.y), candidateColor);
        drawCircle(frame, static_cast<int>(first.x), static_cast<int>(first.y),
                   5, candidateColor);

        const Point analysisFirst = result.corners[index];
        const Point analysisSecond = result.corners[(index + 1) % 4];
        for (uint8_t sample = 0; sample < vision_config::BORDER_SAMPLE_COUNT; sample++) {
            const float t = (sample + 1.0f) /
                            (vision_config::BORDER_SAMPLE_COUNT + 1.0f);
            const Point analysisPoint(
                analysisFirst.x + (analysisSecond.x - analysisFirst.x) * t,
                analysisFirst.y + (analysisSecond.y - analysisFirst.y) * t);
            const Point framePoint = toFramePoint(analysisPoint, frame);
            drawCircle(frame, static_cast<int>(framePoint.x),
                       static_cast<int>(framePoint.y), 2,
                       isBlackNearAnalysisPoint(frame, analysisPoint)
                           ? 0x07E0 : 0xF800);
        }
    }

    for (uint8_t index = 0; index < 4; index++) {
        CandidateLine line{
            result.lines[index].theta,
            result.lines[index].rho,
            result.lines[index].score
        };
        Point first;
        Point second;
        if (lineRectangleIntersections(line, first, second)) {
            const Point frameFirst = toFramePoint(first, frame);
            const Point frameSecond = toFramePoint(second, frame);
            drawLine(frame, static_cast<int>(frameFirst.x), static_cast<int>(frameFirst.y),
                     static_cast<int>(frameSecond.x), static_cast<int>(frameSecond.y), 0x07FF);
        }
    }

    for (uint8_t column = 0; column <= GRID_COLUMNS; column++) {
        const float x = column * ANALYSIS_WIDTH / GRID_COLUMNS;
        drawProjectedLine(frame, result.homography, x, 0.0f, x,
                          ANALYSIS_HEIGHT - 1.0f, 0x07E0);
    }
    for (uint8_t row = 0; row <= GRID_ROWS; row++) {
        const float y = row * ANALYSIS_HEIGHT / GRID_ROWS;
        drawProjectedLine(frame, result.homography, 0.0f, y,
                          ANALYSIS_WIDTH - 1.0f, y, 0x07E0);
    }

    if (!result.valid) return;

    for (uint8_t index = 0; index < CELL_COUNT; index++) {
        Point sample;
        if (project(result.homography, result.cells[index].sampleX,
                    result.cells[index].sampleY, sample)) {
            const Point frameSample = toFramePoint(sample, frame);
            drawCircle(frame, static_cast<int>(frameSample.x),
                       static_cast<int>(frameSample.y), 4,
                       colorMarker(result.cells[index].color));
        }
    }
}

}  // namespace vision
