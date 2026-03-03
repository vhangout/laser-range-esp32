#include <Arduino.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "camera_module.h"

namespace {
#include "secrets.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

enum class WorkMode : uint8_t {
  Idle = 0,
  Calibration = 1,
  Working = 2,
};

volatile WorkMode g_mode = WorkMode::Idle;
portMUX_TYPE g_modeMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t g_workerTaskHandle = nullptr;
uint32_t g_lastCalibrationRefreshMs = 0;
uint32_t g_lastTrackingBroadcastMs = 0;
uint32_t g_lastShotBroadcastMs = 0;
bool g_shotDetectedLatch = false;
CameraModule g_camera;

constexpr BaseType_t kWorkerCore = (CONFIG_ARDUINO_RUNNING_CORE == 0) ? 1 : 0;

const char *modeToString(const WorkMode mode) {
  switch (mode) {
    case WorkMode::Idle:
      return "idle";
    case WorkMode::Calibration:
      return "calibration";
    case WorkMode::Working:
      return "working";
  }
  return "idle";
}

bool parseMode(const String &value, WorkMode &outMode) {
  if (value == "idle") {
    outMode = WorkMode::Idle;
    return true;
  }
  if (value == "calibration") {
    outMode = WorkMode::Calibration;
    return true;
  }
  if (value == "working") {
    outMode = WorkMode::Working;
    return true;
  }
  return false;
}

void setMode(const WorkMode mode) {
  portENTER_CRITICAL(&g_modeMux);
  g_mode = mode;
  portEXIT_CRITICAL(&g_modeMux);
}

WorkMode getMode() {
  portENTER_CRITICAL(&g_modeMux);
  const WorkMode currentMode = g_mode;
  portEXIT_CRITICAL(&g_modeMux);
  return currentMode;
}

void broadcastMode() {
  const String payload = String("{\"mode\":\"") + modeToString(getMode()) + "\"}";
  ws.textAll(payload);
}

void resetCalibrationTracking() {
  g_lastCalibrationRefreshMs = millis();
  g_lastTrackingBroadcastMs = millis();
}

void resetWorkingTracking() {
  g_lastShotBroadcastMs = 0;
  g_shotDetectedLatch = false;
}

void processCalibrationTracking() {
  CameraTrackingResult tracking;
  if (!g_camera.updateTracking(tracking) || !tracking.hasFrame) {
    return;
  }

  const uint32_t nowMs = millis();
  if (tracking.cameraMoved && (nowMs - g_lastCalibrationRefreshMs >= 300U)) {
    g_lastCalibrationRefreshMs = nowMs;
    ws.textAll("refresh");
    Serial.printf("Calibration refresh: dx=%.1f dy=%.1f rot=%.1f motion=%d%%/%d%%\n",
                  tracking.shiftXPx,
                  tracking.shiftYPx,
                  tracking.rotationDeg,
                  tracking.changedPixelPercent,
                  tracking.changedBlockPercent);
  }

  if (nowMs - g_lastTrackingBroadcastMs >= 500U) {
    g_lastTrackingBroadcastMs = nowMs;
    const String payload = String("{\"tracking\":{") +
                           "\"dx\":" + String(tracking.shiftXPx, 2) + "," +
                           "\"dy\":" + String(tracking.shiftYPx, 2) + "," +
                           "\"rot\":" + String(tracking.rotationDeg, 2) + "," +
                           "\"motion\":" + String(tracking.motionDetected ? 1 : 0) + "," +
                           "\"changed\":" + String(tracking.changedPixelPercent) +
                           "}}";
    ws.textAll(payload);
  }
}

void processWorkingTracking() {
  LaserShotResult shot;
  if (!g_camera.detectLaserShot(shot) || !shot.hasFrame) {
    return;
  }

  if (!shot.detected) {
    g_shotDetectedLatch = false;
    return;
  }

  const uint32_t nowMs = millis();
  const bool cooldownPassed = (g_lastShotBroadcastMs == 0) || (nowMs - g_lastShotBroadcastMs >= 200U);
  if (g_shotDetectedLatch || !cooldownPassed) {
    return;
  }

  g_shotDetectedLatch = true;
  g_lastShotBroadcastMs = nowMs;

  const String payload = String("{\"shot\":{") +
                         "\"x\":" + String(shot.x) + "," +
                         "\"y\":" + String(shot.y) + "," +
                         "\"intensity\":" + String(shot.intensity) +
                         "}}";
  ws.textAll(payload);
  Serial.printf("Shot detected: x=%d y=%d intensity=%d\n", shot.x, shot.y, shot.intensity);
}

void workerTask(void * /*parameter*/) {
  WorkMode previousMode = getMode();
  if (previousMode == WorkMode::Calibration) {
    resetCalibrationTracking();
  } else if (previousMode == WorkMode::Working) {
    resetWorkingTracking();
  }

  for (;;) {
    const WorkMode mode = getMode();
    if (mode != previousMode) {
      if (mode == WorkMode::Calibration) {
        resetCalibrationTracking();
      } else if (mode == WorkMode::Working) {
        resetWorkingTracking();
      }
    }

    switch (mode) {
      case WorkMode::Idle:
        // TODO: Add idle mode processing here.
        break;
      case WorkMode::Calibration:
        processCalibrationTracking();
        break;
      case WorkMode::Working:
        processWorkingTracking();
        break;
    }

    previousMode = mode;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void onWebSocketEvent(AsyncWebSocket *serverRef,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len) {
  (void)serverRef;

  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected\n", client->id());
    client->text(String("{\"mode\":\"") + modeToString(getMode()) + "\"}");
    return;
  }

  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
    return;
  }

  if (type != WS_EVT_DATA) {
    return;
  }

  AwsFrameInfo *info = reinterpret_cast<AwsFrameInfo *>(arg);
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
    client->text("{\"error\":\"Only single text frames are supported\"}");
    return;
  }

  String message;
  message.reserve(len);
  for (size_t i = 0; i < len; ++i) {
    message += static_cast<char>(data[i]);
  }
  message.trim();
  message.toLowerCase();

  WorkMode nextMode = WorkMode::Idle;
  if (!parseMode(message, nextMode)) {
    client->text("{\"error\":\"Unknown mode. Use idle/calibration/working\"}");
    return;
  }

  setMode(nextMode);
  Serial.printf("Mode changed via WS to: %s\n", modeToString(nextMode));
  broadcastMode();
}

void addCacheHeaders(AsyncWebServerResponse *response, const String &path) {
  if (path == "/index.html.gz" || path == "/index.html") {
    response->addHeader("Cache-Control", "no-cache");
    return;
  }
  response->addHeader("Cache-Control", "public, max-age=31536000, immutable");
}

bool tryServeFile(AsyncWebServerRequest *request, const String &path) {
  const String actualPath = path + ".gz";
  if (!LittleFS.exists(actualPath)) {
    return false;
  }

  AsyncWebServerResponse *response = request->beginResponse(
      LittleFS, actualPath, String(), false);
  response->addHeader("Content-Encoding", "gzip");
  addCacheHeaders(response, actualPath);
  request->send(response);
  return true;
}

void handleRoot(AsyncWebServerRequest *request) {
  if (!tryServeFile(request, "/index.html")) {
    request->send(404, "text/plain", "index.html.gz not found");
  }
}

void handleNotFound(AsyncWebServerRequest *request) {
  const String path = request->url();
  if (tryServeFile(request, path)) {
    return;
  }
  if (tryServeFile(request, "/index.html")) {
    return;
  }
  request->send(404, "text/plain", "Not Found");
}

void handleRawCam(AsyncWebServerRequest *request) {
  uint8_t *jpegData = nullptr;
  size_t jpegLen = 0;
  if (!g_camera.capturePreviewJpeg(jpegData, jpegLen) || jpegData == nullptr || jpegLen == 0) {
    request->send(500, "text/plain", "Camera preview capture failed");
    return;
  }

  AsyncWebServerResponse *response = request->beginResponse(
      200, "image/jpeg", jpegData, jpegLen);
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  response->addHeader("Pragma", "no-cache");
  response->addHeader("Expires", "0");
  request->onDisconnect([jpegData]() {
    free(jpegData);
  });
  request->send(response);
}

void handleRawCamShot(AsyncWebServerRequest *request) {
  uint8_t *jpegData = nullptr;
  size_t jpegLen = 0;
  uint32_t timestampMs = 0;
  if (!g_camera.getLastShotJpegCopy(jpegData, jpegLen, timestampMs) ||
      jpegData == nullptr ||
      jpegLen == 0) {
    request->send(404, "text/plain", "Shot frame buffer is empty");
    return;
  }

  AsyncWebServerResponse *response = request->beginResponse(
      200, "image/jpeg", jpegData, jpegLen);
  response->addHeader("Cache-Control", "no-cache");
  response->addHeader("X-Shot-Timestamp-Ms", String(timestampMs));
  request->onDisconnect([jpegData]() {
    free(jpegData);
  });
  request->send(response);
}

void handlePrintTarget(AsyncWebServerRequest *request) {
  const String filePath = "/print_target.pdf";
  if (!LittleFS.exists(filePath)) {
    request->send(404, "text/plain", "print_target.pdf not found");
    return;
  }

  AsyncWebServerResponse *response = request->beginResponse(
      LittleFS, filePath, "application/pdf", false);
  response->addHeader("Content-Disposition",
                      "attachment; filename=\"print_target.pdf\"");
  response->addHeader("Cache-Control", "public, max-age=31536000, immutable");
  request->send(response);
}

void handleCameraDistorsion(AsyncWebServerRequest *request) {
  const String filePath = "/camera-distorsion.json";
  if (!LittleFS.exists(filePath)) {
    request->send(404, "text/plain", "camera-distorsion.json not found");
    return;
  }

  AsyncWebServerResponse *response = request->beginResponse(
      LittleFS, filePath, "application/json", false);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
}

void handleSetShotAreaBody(AsyncWebServerRequest *request,
                           uint8_t *data,
                           size_t len,
                           size_t index,
                           size_t total) {
  if (index == 0) {
    String *body = new String();
    if (body == nullptr || !body->reserve(total)) {
      delete body;
      request->send(500, "text/plain", "Body allocation failed");
      return;
    }
    request->_tempObject = body;
  }

  String *body = reinterpret_cast<String *>(request->_tempObject);
  if (body == nullptr) {
    request->send(500, "text/plain", "Body state is invalid");
    return;
  }

  if (!body->concat(reinterpret_cast<const char *>(data), len)) {
    delete body;
    request->_tempObject = nullptr;
    request->send(500, "text/plain", "Body append failed");
    return;
  }

  if (index + len != total) {
    return;
  }

  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  sscanf(body->c_str(), "%d;%d;%d;%d", &x1, &y1, &x2, &y2);
  delete body;
  request->_tempObject = nullptr;

  g_camera.setShotDetectionArea(x1, y1, x2, y2);
  request->send(200, "text/plain", "OK");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else {
    Serial.println("LittleFS mounted");
  }

  if (!g_camera.begin(CameraModule::Model::AiThinker)) {
    Serial.println("Camera init failed, reboot required");
  }

  // WiFi.mode(WIFI_AP);
  // WiFi.softAP(kApSsid, kApPass);
  WiFi.mode(WIFI_STA);
  WiFi.begin(kApSsid, kApPass);
  Serial.print("AP IP: ");
  // Serial.println(WiFi.softAPIP());
  Serial.println(WiFi.localIP());


  server.on("/", HTTP_GET, handleRoot);
  server.on("/rawcam", HTTP_GET, handleRawCam);
  server.on("/rawcamshot", HTTP_GET, handleRawCamShot);
  server.on("/print_target", HTTP_GET, handlePrintTarget);
  server.on("/camera-distorsion", HTTP_GET, handleCameraDistorsion);
  server.on("/shot-area",
            HTTP_POST,
            [](AsyncWebServerRequest *request) {
              (void)request;
            },
            nullptr,
            handleSetShotAreaBody);
  server.onNotFound(handleNotFound);
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP server started");

  xTaskCreatePinnedToCore(workerTask,
                          "modeWorkerTask",
                          4096,
                          nullptr,
                          1,
                          &g_workerTaskHandle,
                          kWorkerCore);
  Serial.printf("Worker task started on core %d\n", kWorkerCore);
}

void loop() {
  ws.cleanupClients();
  delay(10);
}
