#include "camera_module.h"

#include <cmath>
#include <cstring>

#include <img_converters.h>

namespace {
constexpr framesize_t kFrameSize = FRAMESIZE_QVGA;
constexpr int kFrameWidth = 320;
constexpr int kFrameHeight = 240;

constexpr int kPixelStep = 4;
constexpr int kPixelDiffThreshold = 18;
constexpr int kMotionPixelPercentThreshold = 12;
constexpr int kBlockSize = 8;
constexpr int kBlockPixelThreshold = 10;
constexpr int kMotionBlockPercentThreshold = 20;
constexpr int kMotionConfirmFrames = 2;

constexpr int kMaxShiftPx = 8;
constexpr int kShiftStep = 2;
constexpr float kRotationMinDeg = -8.0f;
constexpr float kRotationMaxDeg = 8.0f;
constexpr float kRotationStepDeg = 2.0f;

constexpr float kShiftMovedThresholdPx = 1.5f;
constexpr float kRotationMovedThresholdDeg = 1.0f;

constexpr int kLaserAbsoluteThreshold = 220;
constexpr int kLaserRelativeThreshold = 45;
constexpr int kLaserMinContrast = 30;
constexpr int kLaserLocalRadiusPx = 3;
constexpr int kLaserPixelSpread = 20;
constexpr int kLaserMinBlobPixels = 1;
constexpr int kLaserMaxBlobPixels = 30;
}  // namespace

CameraModule::CameraModule()
    : initialized_(false),
      cameraMutex_(xSemaphoreCreateMutex()),
      prevFrame_(nullptr),
      prevFrameSize_(0),
      hasPrevFrame_(false),
      motionConfirmCounter_(0),
      lastShotJpeg_(nullptr),
      lastShotJpegLen_(0),
      lastShotTimestampMs_(0),
      hasLastShotFrame_(false) {}

CameraModule::~CameraModule() {
  if (prevFrame_ != nullptr) {
    free(prevFrame_);
    prevFrame_ = nullptr;
  }
  clearLastShotFrame();
  if (cameraMutex_ != nullptr) {
    vSemaphoreDelete(cameraMutex_);
    cameraMutex_ = nullptr;
  }
}

CameraModule::CameraPins CameraModule::pinsForModel(const Model model) {
  switch (model) {
    case Model::AiThinker:
      return {32, -1, 0, 26, 27, 5, 18, 19, 21, 36, 39, 34, 35, 25, 23, 22};
    case Model::WroverKit:
      return {-1, -1, 21, 26, 27, 4, 5, 18, 19, 36, 39, 34, 35, 25, 23, 22};
    case Model::EspEye:
      return {-1, -1, 4, 18, 23, 34, 13, 14, 35, 39, 38, 37, 36, 5, 27, 25};
    case Model::M5StackPsram:
      return {-1, 15, 27, 25, 23, 32, 35, 34, 5, 39, 18, 36, 19, 22, 26, 21};
    case Model::M5StackUnitCam:
      return {-1, 15, 27, 25, 23, 32, 35, 34, 5, 39, 18, 36, 19, 22, 26, 21};
  }
  return {32, -1, 0, 26, 27, 5, 18, 19, 21, 36, 39, 34, 35, 25, 23, 22};
}

bool CameraModule::begin(const Model model) {
  camera_config_t config = {};
  const CameraPins pins = pinsForModel(model);

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = pins.y2;
  config.pin_d1 = pins.y3;
  config.pin_d2 = pins.y4;
  config.pin_d3 = pins.y5;
  config.pin_d4 = pins.y6;
  config.pin_d5 = pins.y7;
  config.pin_d6 = pins.y8;
  config.pin_d7 = pins.y9;
  config.pin_xclk = pins.xclk;
  config.pin_pclk = pins.pclk;
  config.pin_vsync = pins.vsync;
  config.pin_href = pins.href;
  config.pin_sccb_sda = pins.siod;
  config.pin_sccb_scl = pins.sioc;
  config.pin_pwdn = pins.pwdn;
  config.pin_reset = pins.reset;
  config.xclk_freq_hz = 20000000;
  config.frame_size = kFrameSize;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  initialized_ = configureSensor();
  if (!initialized_) {
    Serial.println("Camera sensor setup failed");
    return false;
  }

  Serial.println("Camera initialized");
  return true;
}

bool CameraModule::configureSensor() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return false;
  }

  // Lock auto exposure/white balance to keep frame differencing stable.
  sensor->set_whitebal(sensor, 0);
  sensor->set_awb_gain(sensor, 0);
  sensor->set_exposure_ctrl(sensor, 0);
  sensor->set_gain_ctrl(sensor, 0);
  sensor->set_ae_level(sensor, -1);
  sensor->set_aec2(sensor, 0);
  sensor->set_special_effect(sensor, 2);
  sensor->set_saturation(sensor, -2);

  return true;
}

bool CameraModule::capturePreviewJpeg(uint8_t *&jpegData, size_t &jpegLen) {
  jpegData = nullptr;
  jpegLen = 0;

  if (!initialized_) {
    return false;
  }
  if (cameraMutex_ == nullptr) {
    return false;
  }
  if (xSemaphoreTake(cameraMutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  bool ok = false;
  if (fb->format == PIXFORMAT_JPEG) {
    jpegData = static_cast<uint8_t *>(malloc(fb->len));
    if (jpegData != nullptr) {
      memcpy(jpegData, fb->buf, fb->len);
      jpegLen = fb->len;
      ok = true;
    }
  } else {
    uint8_t *jpgBuf = nullptr;
    size_t jpgLen = 0;
    if (frame2jpg(fb, 80, &jpgBuf, &jpgLen)) {
      jpegData = jpgBuf;
      jpegLen = jpgLen;
      ok = true;
    }
  }

  esp_camera_fb_return(fb);
  xSemaphoreGive(cameraMutex_);
  return ok;
}

bool CameraModule::ensurePrevFrame(const size_t frameSize) {
  if (prevFrame_ != nullptr && prevFrameSize_ == frameSize) {
    return true;
  }

  if (prevFrame_ != nullptr) {
    free(prevFrame_);
    prevFrame_ = nullptr;
    prevFrameSize_ = 0;
  }

  prevFrame_ = static_cast<uint8_t *>(malloc(frameSize));
  if (prevFrame_ == nullptr) {
    return false;
  }

  prevFrameSize_ = frameSize;
  hasPrevFrame_ = false;
  return true;
}

bool CameraModule::computeMotion(const uint8_t *current, CameraTrackingResult &result) {
  int changedPixels = 0;
  int totalSamples = 0;

  int changedBlocks = 0;
  int totalBlocks = 0;

  for (int y = 0; y < kFrameHeight; y += kBlockSize) {
    for (int x = 0; x < kFrameWidth; x += kBlockSize) {
      int blockChanged = 0;

      for (int by = 0; by < kBlockSize && (y + by) < kFrameHeight; ++by) {
        for (int bx = 0; bx < kBlockSize && (x + bx) < kFrameWidth; ++bx) {
          const int idx = (y + by) * kFrameWidth + (x + bx);
          const int diff = static_cast<int>(current[idx]) - static_cast<int>(prevFrame_[idx]);
          if (abs(diff) > kPixelDiffThreshold) {
            ++blockChanged;
          }
        }
      }

      if (blockChanged > kBlockPixelThreshold) {
        ++changedBlocks;
      }
      ++totalBlocks;
    }
  }

  for (int i = 0; i < kFrameWidth * kFrameHeight; i += kPixelStep) {
    const int diff = static_cast<int>(current[i]) - static_cast<int>(prevFrame_[i]);
    if (abs(diff) > kPixelDiffThreshold) {
      ++changedPixels;
    }
    ++totalSamples;
  }

  result.changedPixelPercent = (changedPixels * 100) / max(1, totalSamples);
  result.changedBlockPercent = (changedBlocks * 100) / max(1, totalBlocks);

  const bool rawMotion = (result.changedPixelPercent >= kMotionPixelPercentThreshold) ||
                         (result.changedBlockPercent >= kMotionBlockPercentThreshold);
  if (rawMotion) {
    ++motionConfirmCounter_;
  } else {
    motionConfirmCounter_ = 0;
  }

  result.motionDetected = motionConfirmCounter_ >= kMotionConfirmFrames;
  if (result.motionDetected) {
    motionConfirmCounter_ = 0;
  }

  return true;
}

bool CameraModule::detectLaserPoint(const uint8_t *current, LaserShotResult &result) const {
  int maxValue = 0;
  int maxIdx = 0;
  uint32_t sum = 0;
  const int totalPixels = kFrameWidth * kFrameHeight;
  for (int i = 0; i < totalPixels; ++i) {
    const int value = current[i];
    sum += static_cast<uint32_t>(value);
    if (value > maxValue) {
      maxValue = value;
      maxIdx = i;
    }
  }

  const int meanValue = static_cast<int>(sum / max(1, totalPixels));
  const int threshold = max(kLaserAbsoluteThreshold, meanValue + kLaserRelativeThreshold);
  if (maxValue < threshold) {
    return true;
  }

  const int centerX = maxIdx % kFrameWidth;
  const int centerY = maxIdx / kFrameWidth;

  int blobCount = 0;
  int blobSumX = 0;
  int blobSumY = 0;
  int neighborhoodSum = 0;
  int neighborhoodCount = 0;
  const int blobThreshold = max(threshold, maxValue - kLaserPixelSpread);

  for (int y = max(0, centerY - kLaserLocalRadiusPx); y <= min(kFrameHeight - 1, centerY + kLaserLocalRadiusPx); ++y) {
    for (int x = max(0, centerX - kLaserLocalRadiusPx); x <= min(kFrameWidth - 1, centerX + kLaserLocalRadiusPx); ++x) {
      const int idx = y * kFrameWidth + x;
      const int value = current[idx];
      neighborhoodSum += value;
      ++neighborhoodCount;
      if (value >= blobThreshold) {
        ++blobCount;
        blobSumX += x;
        blobSumY += y;
      }
    }
  }

  if (blobCount < kLaserMinBlobPixels || blobCount > kLaserMaxBlobPixels) {
    return true;
  }

  const int neighborhoodMean = neighborhoodSum / max(1, neighborhoodCount);
  if ((maxValue - neighborhoodMean) < kLaserMinContrast) {
    return true;
  }

  result.detected = true;
  result.x = blobSumX / blobCount;
  result.y = blobSumY / blobCount;
  result.intensity = maxValue;
  return true;
}

void CameraModule::clearLastShotFrame() {
  if (lastShotJpeg_ != nullptr) {
    free(lastShotJpeg_);
    lastShotJpeg_ = nullptr;
  }
  lastShotJpegLen_ = 0;
  lastShotTimestampMs_ = 0;
  hasLastShotFrame_ = false;
}

bool CameraModule::storeShotFrameFromFb(camera_fb_t *fb) {
  if (fb == nullptr) {
    return false;
  }

  uint8_t *newJpeg = nullptr;
  size_t newJpegLen = 0;
  if (fb->format == PIXFORMAT_JPEG) {
    newJpeg = static_cast<uint8_t *>(malloc(fb->len));
    if (newJpeg == nullptr) {
      return false;
    }
    memcpy(newJpeg, fb->buf, fb->len);
    newJpegLen = fb->len;
  } else {
    if (!frame2jpg(fb, 80, &newJpeg, &newJpegLen) || newJpeg == nullptr || newJpegLen == 0) {
      return false;
    }
  }

  clearLastShotFrame();
  lastShotJpeg_ = newJpeg;
  lastShotJpegLen_ = newJpegLen;
  lastShotTimestampMs_ = millis();
  hasLastShotFrame_ = true;
  return true;
}

float CameraModule::alignmentError(const uint8_t *current,
                                   const int dx,
                                   const int dy,
                                   const float angleDeg) const {
  const float rad = angleDeg * 0.0174532925f;
  const float c = cosf(rad);
  const float s = sinf(rad);
  const float cx = static_cast<float>(kFrameWidth - 1) * 0.5f;
  const float cy = static_cast<float>(kFrameHeight - 1) * 0.5f;

  float errorSum = 0.0f;
  int samples = 0;

  for (int y = 0; y < kFrameHeight; y += kPixelStep) {
    for (int x = 0; x < kFrameWidth; x += kPixelStep) {
      const float xr = static_cast<float>(x) - cx;
      const float yr = static_cast<float>(y) - cy;
      const float xRot = xr * c - yr * s + cx + static_cast<float>(dx);
      const float yRot = xr * s + yr * c + cy + static_cast<float>(dy);

      const int sx = static_cast<int>(xRot + 0.5f);
      const int sy = static_cast<int>(yRot + 0.5f);

      if (sx < 0 || sy < 0 || sx >= kFrameWidth || sy >= kFrameHeight) {
        continue;
      }

      const int srcIdx = sy * kFrameWidth + sx;
      const int dstIdx = y * kFrameWidth + x;
      errorSum += fabsf(static_cast<float>(current[srcIdx]) - static_cast<float>(prevFrame_[dstIdx]));
      ++samples;
    }
  }

  if (samples == 0) {
    return 1e9f;
  }
  return errorSum / static_cast<float>(samples);
}

void CameraModule::estimateShiftAndRotation(const uint8_t *current, CameraTrackingResult &result) const {
  float bestError = 1e9f;
  int bestDx = 0;
  int bestDy = 0;

  for (int dy = -kMaxShiftPx; dy <= kMaxShiftPx; dy += kShiftStep) {
    for (int dx = -kMaxShiftPx; dx <= kMaxShiftPx; dx += kShiftStep) {
      const float err = alignmentError(current, dx, dy, 0.0f);
      if (err < bestError) {
        bestError = err;
        bestDx = dx;
        bestDy = dy;
      }
    }
  }

  float bestAngle = 0.0f;
  bestError = 1e9f;
  for (float angle = kRotationMinDeg; angle <= kRotationMaxDeg; angle += kRotationStepDeg) {
    const float err = alignmentError(current, bestDx, bestDy, angle);
    if (err < bestError) {
      bestError = err;
      bestAngle = angle;
    }
  }

  result.shiftXPx = static_cast<float>(bestDx);
  result.shiftYPx = static_cast<float>(bestDy);
  result.rotationDeg = bestAngle;
}

bool CameraModule::updateTracking(CameraTrackingResult &result) {
  result = CameraTrackingResult{};

  if (!initialized_) {
    return false;
  }
  if (cameraMutex_ == nullptr) {
    return false;
  }
  if (xSemaphoreTake(cameraMutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  const size_t required = static_cast<size_t>(kFrameWidth) * static_cast<size_t>(kFrameHeight);
  if (fb->len < required) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  if (!ensurePrevFrame(required)) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  const uint8_t *current = fb->buf;
  result.hasFrame = true;

  if (!hasPrevFrame_) {
    memcpy(prevFrame_, current, required);
    hasPrevFrame_ = true;
    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraMutex_);
    return true;
  }

  computeMotion(current, result);
  estimateShiftAndRotation(current, result);
  result.cameraMoved = result.motionDetected ||
                       fabsf(result.shiftXPx) >= kShiftMovedThresholdPx ||
                       fabsf(result.shiftYPx) >= kShiftMovedThresholdPx ||
                       fabsf(result.rotationDeg) >= kRotationMovedThresholdDeg;

  memcpy(prevFrame_, current, required);
  esp_camera_fb_return(fb);
  xSemaphoreGive(cameraMutex_);
  return true;
}

bool CameraModule::detectLaserShot(LaserShotResult &result) {
  result = LaserShotResult{};

  if (!initialized_) {
    return false;
  }
  if (cameraMutex_ == nullptr) {
    return false;
  }
  if (xSemaphoreTake(cameraMutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  const size_t required = static_cast<size_t>(kFrameWidth) * static_cast<size_t>(kFrameHeight);
  if (fb->len < required) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  result.hasFrame = true;
  const bool ok = detectLaserPoint(fb->buf, result);
  if (ok && result.detected) {
    storeShotFrameFromFb(fb);
  }
  esp_camera_fb_return(fb);
  xSemaphoreGive(cameraMutex_);
  return ok;
}

bool CameraModule::getLastShotJpegCopy(uint8_t *&jpegData, size_t &jpegLen, uint32_t &timestampMs) {
  jpegData = nullptr;
  jpegLen = 0;
  timestampMs = 0;

  if (!initialized_) {
    return false;
  }
  if (cameraMutex_ == nullptr) {
    return false;
  }
  if (xSemaphoreTake(cameraMutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }
  if (!hasLastShotFrame_ || lastShotJpeg_ == nullptr || lastShotJpegLen_ == 0) {
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  jpegData = static_cast<uint8_t *>(malloc(lastShotJpegLen_));
  if (jpegData == nullptr) {
    xSemaphoreGive(cameraMutex_);
    return false;
  }

  memcpy(jpegData, lastShotJpeg_, lastShotJpegLen_);
  jpegLen = lastShotJpegLen_;
  timestampMs = lastShotTimestampMs_;
  xSemaphoreGive(cameraMutex_);
  return true;
}
