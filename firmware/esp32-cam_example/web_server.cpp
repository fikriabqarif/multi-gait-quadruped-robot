#include "web_server.h"
#include "web_interface.h"
#include "video_stream.h"

WebServerManager::WebServerManager(UARTComm* uart) : uart(uart), videoStream(nullptr) {
    server = new WebServer(80);
}

void WebServerManager::init(VideoStream* video) {
    videoStream = video;
    
    // Set up route handlers
    server->on("/", HTTP_GET, [this]() { this->handleRoot(); });
    server->on("/control", HTTP_POST, [this]() { this->handleControl(); });
    server->on("/status", HTTP_GET, [this]() { this->handleStatus(); });
    server->on("/stream", HTTP_GET, [this]() { this->handleStream(); });
    server->on("/video", HTTP_GET, [this]() { this->handleVideo(); });
    server->onNotFound([this]() { this->handleNotFound(); });
    
    server->begin();
}

void WebServerManager::begin() {
    server->begin();
}

void WebServerManager::handleClient() {
    server->handleClient();
}

void WebServerManager::setupAP(const char* ssid, const char* password) {
    WiFi.mode(WIFI_AP);
    if (password && strlen(password) >= 8) {
        WiFi.softAP(ssid, password);
    } else {
        WiFi.softAP(ssid);
    }
}

void WebServerManager::setupSTA(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        attempts++;
    }
}

bool WebServerManager::isConnected() {
    return WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP;
}

String WebServerManager::getIPAddress() {
    if (WiFi.getMode() == WIFI_AP) {
        return WiFi.softAPIP().toString();
    } else {
        return WiFi.localIP().toString();
    }
}

void WebServerManager::handleRoot() {
    server->send(200, "text/html", WEB_INTERFACE_HTML);
}

void WebServerManager::handleControl() {
    if (server->hasArg("plain")) {
        String body = server->arg("plain");
        
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            sendErrorResponse(400, "Invalid JSON");
            return;
        }
        
        const char* action = doc["action"];
        if (!action) {
            sendErrorResponse(400, "Missing 'action' field");
            return;
        }
        
        // Handle different actions
        if (strcmp(action, "gait") == 0) {
            const char* gait = doc["gait"];
            float speed = doc["speed"] | 0.5;
            float direction = doc["direction"] | 0.0;
            
            GaitType gaitType = GAIT_IDLE;
            if (strcmp(gait, "walk") == 0) gaitType = GAIT_WALK;
            else if (strcmp(gait, "roll") == 0) gaitType = GAIT_ROLL;
            else if (strcmp(gait, "climb") == 0) gaitType = GAIT_CLIMB;
            else if (strcmp(gait, "stop") == 0) gaitType = GAIT_STOP;
            else if (strcmp(gait, "idle") == 0) gaitType = GAIT_IDLE;
            
            if (uart->sendGaitCommand(gaitType, speed, direction)) {
                StaticJsonDocument<128> response;
                response["status"] = "ok";
                response["message"] = "Command sent";
                sendJSONResponse(200, response.as<JsonObject>());
            } else {
                sendErrorResponse(500, "Failed to send command");
            }
        } else if (strcmp(action, "servo") == 0) {
            uint8_t leg = doc["leg"];
            uint8_t joint = doc["joint"];
            float angle = doc["angle"];
            
            if (uart->sendServoControl(leg, joint, angle)) {
                StaticJsonDocument<128> response;
                response["status"] = "ok";
                response["message"] = "Servo command sent";
                sendJSONResponse(200, response.as<JsonObject>());
            } else {
                sendErrorResponse(500, "Failed to send servo command");
            }
        } else if (strcmp(action, "emergency_stop") == 0) {
            if (uart->sendEmergencyStop()) {
                StaticJsonDocument<128> response;
                response["status"] = "ok";
                response["message"] = "Emergency stop activated";
                sendJSONResponse(200, response.as<JsonObject>());
            } else {
                sendErrorResponse(500, "Failed to send emergency stop");
            }
        } else {
            sendErrorResponse(400, "Unknown action");
        }
    } else {
        sendErrorResponse(400, "Missing request body");
    }
}

void WebServerManager::handleStatus() {
    // Request status from Arduino
    if (uart->sendStatusRequest()) {
        delay(50);  // Wait for response
        
        StatusResponse status;
        if (uart->receiveStatusResponse(status)) {
            StaticJsonDocument<256> response;
            response["state"] = status.state;
            response["batteryVoltage"] = status.batteryVoltage;
            response["roll"] = status.roll;
            response["pitch"] = status.pitch;
            response["emergencyStop"] = status.emergencyStop;
            response["uptime"] = status.uptime;
            
            sendJSONResponse(200, response.as<JsonObject>());
        } else {
            sendErrorResponse(500, "Failed to receive status");
        }
    } else {
        sendErrorResponse(500, "Failed to request status");
    }
}

void WebServerManager::handleVideo() {
    // Redirect to stream endpoint
    server->sendHeader("Location", "/stream", true);
    server->send(302, "text/plain", "");
}

void WebServerManager::handleStream() {
    if (videoStream && videoStream->isInitialized()) {
        videoStream->handleStream(server);
    } else {
        server->send(503, "text/plain", "Camera not available");
    }
}

void WebServerManager::handleNotFound() {
    server->send(404, "text/plain", "Not Found");
}

void WebServerManager::sendJSONResponse(int code, const JsonObject& data) {
    String json;
    serializeJson(data, json);
    server->send(code, "application/json", json);
}

void WebServerManager::sendErrorResponse(int code, const char* message) {
    StaticJsonDocument<128> doc;
    doc["status"] = "error";
    doc["message"] = message;
    sendJSONResponse(code, doc.as<JsonObject>());
}

