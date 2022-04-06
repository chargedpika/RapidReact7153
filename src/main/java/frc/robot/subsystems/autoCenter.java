package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;

public class autoCenter {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-toast");
    private NetworkTableEntry ty = table.getEntry("ty"); 
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry tv = table.getEntry("tv");
    private PIDController pid = new PIDController(0.015, 0.005, 0.0);

    private Double xCache;
    private Double yCache;
    private Double xCacheTime = -1.0;
    private Double yCacheTime = -1.0;

    private Double maxSpeed = 0.4;
    private Double err = 0.5;

    public autoCenter() {
        pid.setSetpoint(0.0);
    }

    private double getX() {
        if (tv.getDouble(0.0) == 1.0) {
            xCache = tx.getDouble(15.0);
            xCacheTime = Timer.getFPGATimestamp();
            return xCache;
        } else if (Timer.getFPGATimestamp() - xCacheTime <= 1.5 && xCacheTime != -1) {
            return xCache;
        } else {
            return 15.0;
        }
    }

    private double getY() {
        if (tv.getDouble(0.0) == 1.0) {
            yCache = ty.getDouble(0.0);
            yCacheTime = Timer.getFPGATimestamp();
            return yCache;
        } else if (Timer.getFPGATimestamp() - yCacheTime <= 1.5 && yCacheTime != -1) {
            return yCache;
        } else {
            return 0.0;
        }
    }

    private double clampSpeed(double speed) {
        return Math.min(Math.max(speed, -maxSpeed), maxSpeed);
    }

    public double getTurn() {
        double x = getX();
        double pidOut = pid.calculate(x);
        return clampSpeed(pidOut);
    }

    public double getSuggestedSpeed() {
        double s = (-37.661*getY()) + 4916;
        return s - 25;
    }

    public boolean isInTarget() {
        if (tv.getDouble(0.0) == 0.0) { return false; }
        if (getX() < err && -getX() > -err) { return true; }
        return false;
    }

    public double distanceGauge() {
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        Double yVal = ty.getDouble(0.0);
        if (yVal != 0.0) {
            SmartDashboard.putNumber("Limelight Y", yVal);
        }
        //SmartDashboard.putNumber("Limelight Y", ty.getDouble(0.0));

        // how high is your limelight off the ground?
        double limelightHeightInches = 27.5;

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 29.0;

        // distance from the target to the floor
        double goalHeightInches = 104.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance

        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
        distanceFromLimelightToGoalInches += 21;

        SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
        SmartDashboard.putNumber("Limelight Angle", yVal);

        // Calculate shoot speed
        if (distanceFromLimelightToGoalInches < 85) {
            return 4150;
        } else {
            return 3550 + (7.5 * distanceFromLimelightToGoalInches);
        }

        //System.out.println(distanceFromLimelightToGoalInches);
    }

}
