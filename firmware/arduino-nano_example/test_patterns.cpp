#include "test_patterns.h"

TestPatterns::TestPatterns(ServoController* servos) : servos(servos) {
}

void TestPatterns::allServoSweep() {
    Serial.println("Starting all servo sweep test...");
    
    for (float angle = 0; angle <= 180; angle += 1) {
        servos->setAllServos(angle);
        delay(20);
    }
    
    for (float angle = 180; angle >= 0; angle -= 1) {
        servos->setAllServos(angle);
        delay(20);
    }
    
    centerAllServos();
    Serial.println("Sweep test complete");
}

void TestPatterns::legLiftTest(uint8_t leg) {
    if (leg >= 4) return;
    
    Serial.print("Testing leg ");
    Serial.println(leg);
    
    // Center all servos first
    centerAllServos();
    delay(500);
    
    // Lift leg by adjusting upper and lower leg servos
    servos->setLegJointAngle(leg, JOINT_UPPER_LEG, 45.0);  // Lift up
    servos->setLegJointAngle(leg, JOINT_LOWER_LEG, 90.0);  // Keep straight
    delay(1000);
    
    // Return to center
    centerAllServos();
    delay(500);
}

void TestPatterns::waveTest() {
    Serial.println("Starting wave test...");
    
    centerAllServos();
    delay(500);
    
    // Wave front legs
    for (int i = 0; i < 3; i++) {
        // Lift front-left leg
        servos->setLegJointAngle(LEG_FRONT_LEFT, JOINT_HIP, 45.0);
        servos->setLegJointAngle(LEG_FRONT_LEFT, JOINT_UPPER_LEG, 30.0);
        delay(300);
        
        servos->setLegJointAngle(LEG_FRONT_LEFT, JOINT_HIP, 135.0);
        delay(300);
        
        servos->setLegJointAngle(LEG_FRONT_LEFT, JOINT_HIP, 45.0);
        delay(300);
        
        centerAllServos();
        delay(500);
        
        // Lift front-right leg
        servos->setLegJointAngle(LEG_FRONT_RIGHT, JOINT_HIP, 135.0);
        servos->setLegJointAngle(LEG_FRONT_RIGHT, JOINT_UPPER_LEG, 30.0);
        delay(300);
        
        servos->setLegJointAngle(LEG_FRONT_RIGHT, JOINT_HIP, 45.0);
        delay(300);
        
        servos->setLegJointAngle(LEG_FRONT_RIGHT, JOINT_HIP, 135.0);
        delay(300);
        
        centerAllServos();
        delay(500);
    }
    
    Serial.println("Wave test complete");
}

void TestPatterns::standTest() {
    Serial.println("Starting stand test...");
    
    // Start in crouched position
    servos->setAllServos(135.0);  // Servos at 135 degrees (crouched)
    delay(1000);
    
    // Stand up gradually
    for (float angle = 135; angle >= 90; angle -= 1) {
        servos->setAllServos(angle);
        delay(30);
    }
    
    delay(1000);
    
    // Crouch down
    for (float angle = 90; angle <= 135; angle += 1) {
        servos->setAllServos(angle);
        delay(30);
    }
    
    centerAllServos();
    Serial.println("Stand test complete");
}

void TestPatterns::individualServoTest(uint8_t channel) {
    if (channel >= 12) return;
    
    Serial.print("Testing servo channel ");
    Serial.println(channel);
    
    // Sweep individual servo
    for (float angle = 0; angle <= 180; angle += 1) {
        servos->setAngle(channel, angle);
        delay(20);
    }
    
    for (float angle = 180; angle >= 0; angle -= 1) {
        servos->setAngle(channel, angle);
        delay(20);
    }
    
    // Return to center
    servos->setAngle(channel, 90.0);
    Serial.println("Individual servo test complete");
}

void TestPatterns::setAllServos(float angle) {
    servos->setAllServos(angle);
}

void TestPatterns::centerAllServos() {
    servos->setAllServos(90.0);
}

