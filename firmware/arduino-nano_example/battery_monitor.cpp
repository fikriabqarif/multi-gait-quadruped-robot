#include "battery_monitor.h"

BatteryMonitor::BatteryMonitor(uint8_t pin, float dividerRatio)
    : adcPin(pin), dividerRatio(dividerRatio),
      currentVoltage(0.0),
      lowBatteryThreshold(DEFAULT_LOW_BATTERY_VOLTAGE),
      criticalBatteryThreshold(DEFAULT_CRITICAL_BATTERY_VOLTAGE),
      fullBatteryVoltage(DEFAULT_FULL_BATTERY_VOLTAGE),
      historyIndex(0), historyFilled(false) {
    for (int i = 0; i < FILTER_SIZE; i++) {
        voltageHistory[i] = 0.0;
    }
}

void BatteryMonitor::init() {
    pinMode(adcPin, INPUT);
    // Take initial reading
    update();
}

void BatteryMonitor::update() {
    // Read ADC value
    int adcValue = analogRead(adcPin);
    
    // Convert ADC value to voltage at ADC pin
    float adcVoltage = (adcValue / (float)ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
    
    // Apply voltage divider ratio to get actual battery voltage
    float rawVoltage = adcVoltage * dividerRatio;
    
    // Apply filtering
    currentVoltage = filterVoltage(rawVoltage);
}

float BatteryMonitor::filterVoltage(float rawVoltage) {
    // Simple moving average filter
    voltageHistory[historyIndex] = rawVoltage;
    historyIndex = (historyIndex + 1) % FILTER_SIZE;
    
    if (!historyFilled && historyIndex == 0) {
        historyFilled = true;
    }
    
    float sum = 0.0;
    int count = historyFilled ? FILTER_SIZE : historyIndex;
    
    for (int i = 0; i < count; i++) {
        sum += voltageHistory[i];
    }
    
    return sum / count;
}

float BatteryMonitor::getVoltage() {
    return currentVoltage;
}

float BatteryMonitor::getPercentage() {
    // Linear interpolation between critical and full voltage
    if (currentVoltage <= criticalBatteryThreshold) {
        return 0.0;
    }
    if (currentVoltage >= fullBatteryVoltage) {
        return 100.0;
    }
    
    float range = fullBatteryVoltage - criticalBatteryThreshold;
    float voltageAboveCritical = currentVoltage - criticalBatteryThreshold;
    return (voltageAboveCritical / range) * 100.0;
}

void BatteryMonitor::setLowBatteryThreshold(float voltage) {
    lowBatteryThreshold = voltage;
}

void BatteryMonitor::setCriticalBatteryThreshold(float voltage) {
    criticalBatteryThreshold = voltage;
}

void BatteryMonitor::setFullBatteryVoltage(float voltage) {
    fullBatteryVoltage = voltage;
}

bool BatteryMonitor::isLowBattery() {
    return currentVoltage < lowBatteryThreshold;
}

bool BatteryMonitor::isCriticalBattery() {
    return currentVoltage < criticalBatteryThreshold;
}

bool BatteryMonitor::isBatteryOK() {
    return !isCriticalBattery();
}

uint8_t BatteryMonitor::getPin() {
    return adcPin;
}

