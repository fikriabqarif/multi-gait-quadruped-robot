#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

// Default thresholds for 2S Li-ion battery (7.4V nominal)
#define DEFAULT_LOW_BATTERY_VOLTAGE 6.0   // 3.0V per cell
#define DEFAULT_CRITICAL_BATTERY_VOLTAGE 5.5  // 2.75V per cell
#define DEFAULT_FULL_BATTERY_VOLTAGE 8.4  // 4.2V per cell

// Voltage divider configuration
// If using a voltage divider, set these values
// Example: R1 = 10k, R2 = 10k -> divider ratio = 2.0
#define VOLTAGE_DIVIDER_RATIO 2.0
#define ADC_REFERENCE_VOLTAGE 5.0  // Arduino Nano uses 5V reference
#define ADC_RESOLUTION 1024  // 10-bit ADC

class BatteryMonitor {
public:
    BatteryMonitor(uint8_t pin, float dividerRatio = VOLTAGE_DIVIDER_RATIO);
    
    // Initialization
    void init();
    
    // Reading
    void update();  // Call regularly to update voltage reading
    float getVoltage();  // Get battery voltage in volts
    float getPercentage();  // Get battery percentage (0-100%)
    
    // Safety monitoring
    void setLowBatteryThreshold(float voltage);
    void setCriticalBatteryThreshold(float voltage);
    void setFullBatteryVoltage(float voltage);
    
    bool isLowBattery();
    bool isCriticalBattery();
    bool isBatteryOK();
    
    // Utility
    uint8_t getPin();
    
private:
    uint8_t adcPin;
    float dividerRatio;
    float currentVoltage;
    float lowBatteryThreshold;
    float criticalBatteryThreshold;
    float fullBatteryVoltage;
    
    // Filtering
    static const int FILTER_SIZE = 10;
    float voltageHistory[FILTER_SIZE];
    int historyIndex;
    bool historyFilled;
    
    float filterVoltage(float rawVoltage);
};

#endif // BATTERY_MONITOR_H

