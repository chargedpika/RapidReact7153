package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class maxSpeed {
    public double maxSpeedLow;
    public double maxSpeedHigh;

    public double currentMax;
    private Joystick joy;

    public maxSpeed(int joyId, double min, double max) {
        joy = new Joystick(joyId);
        maxSpeedLow = min;
        maxSpeedHigh = max;
        currentMax = min;
    }

    public double applyMaxSpeed(double value) {
        return Math.min(Math.max(value, -currentMax), currentMax);
    }

    public void refresh() {
        if (joy.getThrottle() > 0) {
            currentMax = maxSpeedLow;
        } else {
            currentMax = maxSpeedHigh;
        }
        SmartDashboard.putNumber("Max Speed", currentMax);
        SmartDashboard.putNumber("Joystick 2 Throttle", joy.getThrottle());
    }
}
