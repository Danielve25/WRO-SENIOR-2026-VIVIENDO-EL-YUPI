#include "vision_pipeline.h"

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
uint16_t *hough = nullptr;
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

vision::Color classifyColor(const Rgb &rgb, const Hsv &hsv, uint8_t &confidence)
{
    const uint8_t maximum = std::max(rgb.red, std::max(rgb.green, rgb.blue));
    const uint8_t minimum = std::min(rgb.red, std::min(rgb.green, rgb.blue));
    const uint8_t chroma = maximum - minimum;

    if (maximum < 35U) {
        confidence = 0;
        return vision::Color::Unknown;
    }

    if (chroma < static_cast<uint8_t>(std::max(10, maximum / 8)) &&
        maximum > 100U) {
        confidence = static_cast<uint8_t>(clampFloat(
            100.0f - std::fabs(190.0f - maximum) * 0.35f, 0.0f, 100.0f));
        return vision::Color::White;
    }

    const float hues[3] = {55.0f, 125.0f, 220.0f};
    const vision::Color colors[3] = {
        vision::Color::Yellow,
        vision::Color::Green,
        vision::Color::Blue
    };

    float bestDistance = 1000.0f;
    uint8_t bestIndex = 0;
    for (uint8_t index = 0; index < 3; index++) {
        const float distance = hueDistance(static_cast<float>(hsv.hue), hues[index]);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestIndex = index;
        }
    }

    const float saturationFactor = hsv.saturation / 255.0f;
    const float score = (1.0f - bestDistance / 180.0f) * 100.0f *
                        clampFloat(saturationFactor, 0.35f, 1.0f);
    confidence = static_cast<uint8_t>(clampFloat(score, 0.0f, 100.0f));
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

    const uint8_t threshold = std::max<uint8_t>(35, maximumGradient / 4);
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

bool detectQuadrilateral(vision::Result &result)
{
    const uint8_t candidateCount = extractLineCandidates();
    float bestScore = -1.0f;
    vision::Point bestCorners[4];
    CandidateLine bestLines[4];

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
                    if (area < vision::ANALYSIS_WIDTH * vision::ANALYSIS_HEIGHT * 0.08f) {
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
                    const float ratioScore = 1.0f - clampFloat(
                        std::fabs(ratio - 1.333f) / 1.333f, 0.0f, 1.0f);
                    const float lineScore = static_cast<float>(
                        aNear.score + aFar.score + bNear.score + bFar.score);
                    const float score = lineScore * 2.0f + area * 0.25f + ratioScore * 100.0f;
                    if (score > bestScore) {
                        bestScore = score;
                        std::memcpy(bestCorners, corners, sizeof(bestCorners));
                        bestLines[0] = aNear;
                        bestLines[1] = aFar;
                        bestLines[2] = bNear;
                        bestLines[3] = bFar;
                    }
                }
            }
        }
    }

    if (bestScore < 0.0f) {
        return false;
    }

    std::memcpy(result.corners, bestCorners, sizeof(bestCorners));
    for (uint8_t index = 0; index < 4; index++) {
        result.lines[index].theta = bestLines[index].theta;
        result.lines[index].rho = bestLines[index].rho;
        result.lines[index].score = bestLines[index].score;
    }
    result.confidence = static_cast<uint8_t>(clampFloat(
        bestScore / 12.0f, 0.0f, 100.0f));
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
    hough = static_cast<uint16_t *>(heap_caps_malloc(
        static_cast<size_t>(THETA_COUNT) * RHO_BINS * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM));

    if (grayImage == nullptr || edgeImage == nullptr || hough == nullptr) {
        if (grayImage != nullptr) heap_caps_free(grayImage);
        if (edgeImage != nullptr) heap_caps_free(edgeImage);
        if (hough != nullptr) heap_caps_free(hough);
        grayImage = nullptr;
        edgeImage = nullptr;
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
    result.valid = detectQuadrilateral(result);
    if (!result.valid || !solveHomography(result.corners, result.homography)) {
        result.valid = false;
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

    result.processingMilliseconds = static_cast<uint16_t>(millis() - start);
    return true;
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
    if (frame == nullptr || !result.valid) return;

    for (uint8_t index = 0; index < 4; index++) {
        const Point first = toFramePoint(result.corners[index], frame);
        const Point second = toFramePoint(result.corners[(index + 1) % 4], frame);
        drawLine(frame, static_cast<int>(first.x), static_cast<int>(first.y),
                 static_cast<int>(second.x), static_cast<int>(second.y), 0xF800);
        drawCircle(frame, static_cast<int>(first.x), static_cast<int>(first.y),
                   5, 0xFFE0);
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
