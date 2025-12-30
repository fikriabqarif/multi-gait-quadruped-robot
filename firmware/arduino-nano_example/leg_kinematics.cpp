#include "leg_kinematics.h"

LegKinematics::LegKinematics() {
    upperLegLength = UPPER_LEG_LENGTH;
    lowerLegLength = LOWER_LEG_LENGTH;
}

void LegKinematics::setDimensions(float upperLength, float lowerLength) {
    upperLegLength = upperLength;
    lowerLegLength = lowerLength;
}

Point3D LegKinematics::forwardKinematics(const JointAngles& angles) {
    Point3D result;
    
    // Convert angles to radians
    float hipRad = degreesToRadians(angles.hip);
    float upperRad = degreesToRadians(angles.upperLeg);
    float lowerRad = degreesToRadians(angles.lowerLeg);
    
    // Leg frame: x = forward, y = lateral (left positive), z = vertical (down positive)
    // Hip rotates around z-axis (yaw)
    // Upper and lower leg rotate around y-axis (pitch)
    
    // Calculate position in leg's local x-z plane (after hip rotation)
    float legX = upperLegLength * cos(upperRad) + lowerLegLength * cos(upperRad + lowerRad);
    float legZ = upperLegLength * sin(upperRad) + lowerLegLength * sin(upperRad + lowerRad);
    
    // Apply hip rotation
    result.x = legX * cos(hipRad);
    result.y = legX * sin(hipRad);
    result.z = legZ;
    
    return result;
}

bool LegKinematics::inverseKinematics(const Point3D& target, JointAngles& angles) {
    // Check if target is reachable
    if (!isReachable(target)) {
        return false;
    }
    
    // Calculate distance from hip to target in x-y plane
    float r = sqrt(target.x * target.x + target.y * target.y);
    float d = sqrt(r * r + target.z * target.z);  // 3D distance
    
    // Check if target is within reach
    float maxReach = upperLegLength + lowerLegLength;
    if (d > maxReach) {
        return false;
    }
    
    // Calculate hip angle (rotation in x-y plane)
    angles.hip = radiansToDegrees(atan2(target.y, target.x));
    
    // Calculate angles in leg plane (x-z plane after hip rotation)
    // Using law of cosines
    float cosLower = (upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - d * d) /
                     (2.0 * upperLegLength * lowerLegLength);
    
    // Clamp to valid range for acos
    if (cosLower > 1.0) cosLower = 1.0;
    if (cosLower < -1.0) cosLower = -1.0;
    
    angles.lowerLeg = radiansToDegrees(acos(cosLower)) - 180.0;  // Negative for typical servo mounting
    
    // Calculate upper leg angle
    float alpha = atan2(target.z, r);
    float beta = acos((upperLegLength * upperLegLength + d * d - lowerLegLength * lowerLegLength) /
                      (2.0 * upperLegLength * d));
    
    if (beta != beta) {  // Check for NaN
        return false;
    }
    
    angles.upperLeg = radiansToDegrees(alpha - beta);
    
    // Normalize angles
    angles.hip = normalizeAngle(angles.hip);
    angles.upperLeg = normalizeAngle(angles.upperLeg);
    angles.lowerLeg = normalizeAngle(angles.lowerLeg);
    
    return true;
}

Point3D LegKinematics::bodyToLegFrame(const Point3D& bodyPoint, uint8_t leg) {
    Point3D legOffset = getLegOffset(leg);
    Point3D result;
    
    // Translate to leg origin
    result.x = bodyPoint.x - legOffset.x;
    result.y = bodyPoint.y - legOffset.y;
    result.z = bodyPoint.z - legOffset.z;
    
    // Rotate based on leg position (front/back, left/right)
    // This depends on your robot's coordinate system
    // Assuming: front legs face forward, back legs face backward
    if (leg == LEG_BACK_LEFT || leg == LEG_BACK_RIGHT) {
        // Back legs are rotated 180 degrees
        result.x = -result.x;
        result.y = -result.y;
    }
    
    return result;
}

Point3D LegKinematics::legToBodyFrame(const Point3D& legPoint, uint8_t leg) {
    Point3D legOffset = getLegOffset(leg);
    Point3D result;
    
    // Rotate back (inverse of bodyToLegFrame rotation)
    if (leg == LEG_BACK_LEFT || leg == LEG_BACK_RIGHT) {
        result.x = -legPoint.x;
        result.y = -legPoint.y;
        result.z = legPoint.z;
    } else {
        result = legPoint;
    }
    
    // Translate back to body origin
    result.x += legOffset.x;
    result.y += legOffset.y;
    result.z += legOffset.z;
    
    return result;
}

bool LegKinematics::isReachable(const Point3D& target) {
    float d = distance({0, 0, 0}, target);
    float maxReach = upperLegLength + lowerLegLength;
    float minReach = abs(upperLegLength - lowerLegLength);
    
    return (d >= minReach && d <= maxReach);
}

float LegKinematics::distance(const Point3D& p1, const Point3D& p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

float LegKinematics::normalizeAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

float LegKinematics::radiansToDegrees(float rad) {
    return rad * 180.0 / PI;
}

float LegKinematics::degreesToRadians(float deg) {
    return deg * PI / 180.0;
}

Point3D LegKinematics::getLegOffset(uint8_t leg) {
    // Leg positions relative to body center (in mm)
    // Adjust these based on your robot's dimensions
    Point3D offset = {0, 0, 0};
    
    // Example layout (adjust to your robot):
    // Front-left:  x=+50, y=+50
    // Front-right: x=+50, y=-50
    // Back-left:   x=-50, y=+50
    // Back-right:  x=-50, y=-50
    
    switch (leg) {
        case LEG_FRONT_LEFT:
            offset.x = 50.0;
            offset.y = 50.0;
            break;
        case LEG_FRONT_RIGHT:
            offset.x = 50.0;
            offset.y = -50.0;
            break;
        case LEG_BACK_LEFT:
            offset.x = -50.0;
            offset.y = 50.0;
            break;
        case LEG_BACK_RIGHT:
            offset.x = -50.0;
            offset.y = -50.0;
            break;
    }
    
    return offset;
}

