#include <Arduino.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

namespace {
#include "secrets.h"

AsyncWebServer server(80);

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
      LittleFS, actualPath, contentType, true);
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
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  delay(10);
}
