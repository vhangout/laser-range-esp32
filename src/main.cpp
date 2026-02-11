#include <Arduino.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

void workerTask(void * /*parameter*/) {
  for (;;) {
    const WorkMode mode = getMode();

    switch (mode) {
      case WorkMode::Idle:
        // TODO: Add idle mode processing here.
        break;
      case WorkMode::Calibration:
        // TODO: Add calibration mode processing here.
        break;
      case WorkMode::Working:
        // TODO: Add working mode processing here.
        break;
    }

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

const char *getContentType(const String &path) {
  if (path.endsWith(".txt") || path.endsWith(".txt.gz")) return "text/plain";
  if (path.endsWith(".html") || path.endsWith(".html.gz")) return "text/html";
  if (path.endsWith(".css") || path.endsWith(".css.gz")) return "text/css";
  if (path.endsWith(".js") || path.endsWith(".js.gz")) return "application/javascript";
  if (path.endsWith(".map") || path.endsWith(".map.gz")) return "application/json";
  if (path.endsWith(".json") || path.endsWith(".json.gz")) return "application/json";
  if (path.endsWith(".xml") || path.endsWith(".xml.gz")) return "application/xml";
  if (path.endsWith(".wasm") || path.endsWith(".wasm.gz")) return "application/wasm";
  if (path.endsWith(".png") || path.endsWith(".png.gz")) return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg") || path.endsWith(".jpg.gz") ||
      path.endsWith(".jpeg.gz")) {
    return "image/jpeg";
  }
  if (path.endsWith(".svg") || path.endsWith(".svg.gz")) return "image/svg+xml";
  if (path.endsWith(".ico") || path.endsWith(".ico.gz")) return "image/x-icon";
  if (path.endsWith(".webp") || path.endsWith(".webp.gz")) return "image/webp";
  if (path.endsWith(".gif") || path.endsWith(".gif.gz")) return "image/gif";
  if (path.endsWith(".mp3") || path.endsWith(".mp3.gz")) return "audio/mpeg";
  if (path.endsWith(".mp4") || path.endsWith(".mp4.gz")) return "video/mp4";
  if (path.endsWith(".woff") || path.endsWith(".woff.gz")) return "font/woff";
  if (path.endsWith(".woff2") || path.endsWith(".woff2.gz")) return "font/woff2";
  return "application/octet-stream";
}

void addCacheHeaders(AsyncWebServerResponse *response, const String &path) {
  if (path == "/index.html.gz" || path == "/index.html") {
    response->addHeader("Cache-Control", "no-cache");
    return;
  }
  response->addHeader("Cache-Control", "public, max-age=31536000, immutable");
}

bool tryServeFile(AsyncWebServerRequest *request, const String &path) {
  String actualPath = path;
  bool isGzip = false;

  if (LittleFS.exists(path + ".gz")) {
    actualPath = path + ".gz";
    isGzip = true;
  } else if (!LittleFS.exists(path)) {
    return false;
  }

  const char *contentType = getContentType(actualPath);
  AsyncWebServerResponse *response = request->beginResponse(
      LittleFS, actualPath, contentType, false);
  if (isGzip) {
    response->addHeader("Content-Encoding", "gzip");
  }
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
  String actualPath = "/calibrate.jpg";
  const char *contentType = getContentType(actualPath);
  AsyncWebServerResponse *response = request->beginResponse(
      LittleFS, actualPath, contentType, false);
  response->addHeader("Cache-Control", "no-cache");
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

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else {
    Serial.println("LittleFS mounted");
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
  server.on("/print_target", HTTP_GET, handlePrintTarget);
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
