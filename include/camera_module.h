#pragma once

#include <Arduino.h>
#include <esp_camera.h>
#include <freertos/semphr.h>

struct CameraTrackingResult {
  bool hasFrame = false;
  bool motionDetected = false;
  bool cameraMoved = false;
  int changedPixelPercent = 0;
  int changedBlockPercent = 0;
  float shiftXPx = 0.0f;
  float shiftYPx = 0.0f;
  float rotationDeg = 0.0f;
};

struct LaserShotResult {
  bool hasFrame = false;
  bool detected = false;
  int x = 0;
  int y = 0;
  int intensity = 0;
};

class CameraModule {
 public:
  enum class Model : uint8_t {
    AiThinker = 0,
    WroverKit = 1,
    EspEye = 2,
    M5StackPsram = 3,
    M5StackUnitCam = 4,
  };

  CameraModule();
  ~CameraModule();

  bool begin(Model model = Model::AiThinker);
  bool capturePreviewJpeg(uint8_t *&jpegData, size_t &jpegLen);
  bool updateTracking(CameraTrackingResult &result);
  bool detectLaserShot(LaserShotResult &result);
  bool getLastShotJpegCopy(uint8_t *&jpegData, size_t &jpegLen, uint32_t &timestampMs);
  const String &getCalibrationTableJson() const;

 private:
  struct CameraPins {
    int pwdn;
    int reset;
    int xclk;
    int siod;
    int sioc;
    int y2;
    int y3;
    int y4;
    int y5;
    int y6;
    int y7;
    int y8;
    int y9;
    int vsync;
    int href;
    int pclk;
  };

  static CameraPins pinsForModel(Model model);
  bool configureSensor();
  bool ensurePrevFrame(size_t frameSize);
  bool computeMotion(const uint8_t *current, CameraTrackingResult &result);
  bool detectLaserPoint(const uint8_t *current, LaserShotResult &result) const;
  bool storeShotFrameFromFb(camera_fb_t *fb);
  void clearLastShotFrame();
  void estimateShiftAndRotation(const uint8_t *current, CameraTrackingResult &result) const;
  float alignmentError(const uint8_t *current, int dx, int dy, float angleDeg) const;

  bool initialized_;
  SemaphoreHandle_t cameraMutex_;
  uint8_t *prevFrame_;
  size_t prevFrameSize_;
  bool hasPrevFrame_;
  int motionConfirmCounter_;
  uint8_t *lastShotJpeg_;
  size_t lastShotJpegLen_;
  uint32_t lastShotTimestampMs_;
  bool hasLastShotFrame_;
  String calibrationTableJson_;
};
