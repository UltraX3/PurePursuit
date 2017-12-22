package com.github.ultrax3.purepursuit;

/**
 * @deprecated
 */
public interface TankRobotState {
    double getLeftTreadVelocity();
    double getRightTreadVelocity();
    double getLeftMaxVelocity();
    double getRightMaxVelocity();
    float getYaw();
}
