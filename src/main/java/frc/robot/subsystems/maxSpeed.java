package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class maxSpeed {
    public double maxSpeedLow;
    public double maxSpeedHigh;

    private double currentMax;
    private Joystick joy = new Joystick(1);

    public maxSpeed(double min, double max) {
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
