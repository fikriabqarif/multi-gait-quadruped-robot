#include "video_stream.h"

VideoStream::VideoStream() : initialized(false), frameSize(FRAMESIZE_VGA), jpegQuality(12) {
}

bool VideoStream::init() {
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    // Frame size
    config.frame_size = frameSize;
    config.jpeg_quality = jpegQuality;
    config.fb_count = 2;
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        return false;
    }
    
    // Get camera sensor
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_framesize(s, frameSize);
        s->set_quality(s, jpegQuality);
    }
    
    initialized = true;
    return true;
}

void VideoStream::handleStream(WebServer* server) {
    if (!initialized) {
        server->send(500, "text/plain", "Camera not initialized");
        return;
    }
    
    WiFiClient client = server->client();
    
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server->client().print(response);
    
    while (client.connected()) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            break;
        }
        
        client.print("--frame\r\n");
        client.print("Content-Type: image/jpeg\r\n");
        client.print("Content-Length: " + String(fb->len) + "\r\n\r\n");
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        
        esp_camera_fb_return(fb);
        
        delay(33);  // ~30 FPS
    }
}

void VideoStream::setFrameSize(framesize_t size) {
    frameSize = size;
    if (initialized) {
        sensor_t * s = esp_camera_sensor_get();
        if (s != NULL) {
            s->set_framesize(s, size);
        }
    }
}

void VideoStream::setQuality(uint8_t quality) {
    if (quality > 63) quality = 63;
    jpegQuality = quality;
    if (initialized) {
        sensor_t * s = esp_camera_sensor_get();
        if (s != NULL) {
            s->set_quality(s, quality);
        }
    }
}

bool VideoStream::isInitialized() {
    return initialized;
}

