#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "uart_comm.h"

// Forward declaration
class VideoStream;

class WebServerManager {
public:
    WebServerManager(UARTComm* uart);
    
    // Initialization
    void init(VideoStream* video = nullptr);
    void begin();
    void handleClient();
    
    // WiFi configuration
    void setupAP(const char* ssid, const char* password = nullptr);
    void setupSTA(const char* ssid, const char* password);
    bool isConnected();
    String getIPAddress();
    
private:
    WebServer* server;
    UARTComm* uart;
    VideoStream* videoStream;
    
    // Request handlers
    void handleRoot();
    void handleControl();
    void handleStatus();
    void handleVideo();
    void handleStream();
    void handleNotFound();
    
    // Helper functions
    String getHTML();
    void sendJSONResponse(int code, const JsonObject& data);
    void sendErrorResponse(int code, const char* message);
};

#endif // WEB_SERVER_H

