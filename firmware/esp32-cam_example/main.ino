#include "uart_comm.h"
#include "web_server.h"
#include "video_stream.h"
#include <WiFi.h>

// WiFi Configuration
// Uncomment and configure one of the following:

// Option 1: Access Point Mode (robot creates its own WiFi network)
#define WIFI_AP_MODE
#define AP_SSID "QuadrupedRobot"
#define AP_PASSWORD "robot1234"  // Minimum 8 characters

// Option 2: Station Mode (connect to existing WiFi)
// #define WIFI_STA_MODE
// #define STA_SSID "YourWiFiNetwork"
// #define STA_PASSWORD "YourWiFiPassword"

// System objects
UARTComm uart;
WebServerManager webServer(&uart);
VideoStream videoStream;

// Status update timing
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 1000;  // ms

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== ESP32-CAM Quadruped Robot Controller ===");
    
    // Initialize UART communication with Arduino
    Serial.println("Initializing UART...");
    uart.init();
    delay(500);
    
    // Initialize camera and video streaming
    Serial.println("Initializing camera...");
    if (!videoStream.init()) {
        Serial.println("ERROR: Camera initialization failed!");
        // Continue without camera - robot can still be controlled
    } else {
        Serial.println("Camera initialized successfully");
        videoStream.setFrameSize(FRAMESIZE_VGA);
        videoStream.setQuality(12);
    }
    
    // Setup WiFi
    Serial.println("Setting up WiFi...");
    #ifdef WIFI_AP_MODE
        webServer.setupAP(AP_SSID, AP_PASSWORD);
        Serial.print("Access Point started. SSID: ");
        Serial.println(AP_SSID);
        Serial.print("Password: ");
        Serial.println(AP_PASSWORD);
        Serial.print("IP Address: ");
        Serial.println(webServer.getIPAddress());
    #endif
    
    #ifdef WIFI_STA_MODE
        webServer.setupSTA(STA_SSID, STA_PASSWORD);
        if (webServer.isConnected()) {
            Serial.print("Connected to WiFi. IP Address: ");
            Serial.println(webServer.getIPAddress());
        } else {
            Serial.println("Failed to connect to WiFi!");
            // Continue anyway - can retry later
        }
    #endif
    
    // Initialize web server with video stream
    Serial.println("Initializing web server...");
    webServer.init(&videoStream);
    
    Serial.println("=== Initialization Complete ===");
    Serial.println("Open a web browser and navigate to:");
    Serial.print("http://");
    Serial.println(webServer.getIPAddress());
    Serial.println();
}

void loop() {
    // Handle web server requests
    webServer.handleClient();
    
    // Handle video streaming (if camera is initialized)
    // Note: Video streaming is typically handled by the web server
    // This is a simplified approach - in production, integrate with WebServer
    
    // Periodically request status from Arduino
    if (millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
        // Status is requested by the web interface via AJAX
        // We can also request it here for logging/debugging
        lastStatusUpdate = millis();
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}

