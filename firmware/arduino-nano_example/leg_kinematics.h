#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <Arduino.h>
#include <math.h>

// Leg dimensions (in mm) - adjust these based on your robot design
#define UPPER_LEG_LENGTH 80.0   // Length from hip to knee
#define LOWER_LEG_LENGTH 80.0   // Length from knee to foot

// Workspace limits
#define MAX_REACH 140.0  // Maximum reach distance (approximate)
#define MIN_REACH 40.0   // Minimum reach distance

struct Point3D {
    float x, y, z;
};

struct JointAngles {
    float hip;        // Hip rotation angle (degrees)
    float upperLeg;   // Upper leg angle (degrees)
    float lowerLeg;   // Lower leg angle (degrees)
};

class LegKinematics {
public:
    LegKinematics();
    
    // Set leg dimensions
    void setDimensions(float upperLength, float lowerLength);
    
    // Forward kinematics: joint angles -> foot position
    // Input: joint angles in degrees, leg frame coordinates
    // Output: foot position in leg frame (x, y, z)
    Point3D forwardKinematics(const JointAngles& angles);
    
    // Inverse kinematics: foot position -> joint angles
    // Input: desired foot position in leg frame (x, y, z)
    // Output: joint angles in degrees
    // Returns: true if position is reachable, false otherwise
    bool inverseKinematics(const Point3D& target, JointAngles& angles);
    
    // Coordinate transformations
    // Convert from body frame to leg frame
    Point3D bodyToLegFrame(const Point3D& bodyPoint, uint8_t leg);
    
    // Convert from leg frame to body frame
    Point3D legToBodyFrame(const Point3D& legPoint, uint8_t leg);
    
    // Workspace checking
    bool isReachable(const Point3D& target);
    
    // Utility functions
    float distance(const Point3D& p1, const Point3D& p2);
    
private:
    float upperLegLength;
    float lowerLegLength;
    
    // Helper functions
    float normalizeAngle(float angle);  // Normalize to -180 to 180
    float radiansToDegrees(float rad);
    float degreesToRadians(float deg);
    
    // Leg frame offsets (position of leg origin relative to body center)
    // These depend on robot geometry - adjust as needed
    Point3D getLegOffset(uint8_t leg);
};

#endif // LEG_KINEMATICS_H

