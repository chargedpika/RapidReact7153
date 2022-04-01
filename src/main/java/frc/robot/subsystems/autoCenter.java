package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
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
    private Double cacheTime = -1.0;

    private Double maxSpeed = 0.4;
    private Double err = 0.5;

    public autoCenter() {
        pid.setSetpoint(0.0);
    }

    private double getX() {
        if (tv.getDouble(0.0) == 1.0) {
            xCache = tx.getDouble(15.0);
            cacheTime = Timer.getFPGATimestamp();
            return xCache;
        } else if (Timer.getFPGATimestamp() - cacheTime <= 1.5 && cacheTime != -1) {
            return xCache;
        } else {
            return 15.0;
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
    public void distanceGauge() {



    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-toast");
NetworkTableEntry ty = table.getEntry("ty");
double targetOffsetAngle_Vertical = ty.getDouble(0.0);

// how high is your limelight off the ground?
double limelightHeightInches = 27.5;

// how many degrees back is your limelight rotated from perfectly vertical?
double limelightMountAngleDegrees = 30.0;

// distance from the center of the Limelight lens to the floor
double limelightLensHeightInches = 20.0;

// distance from the target to the floor
double goalHeightInches = 103.0;

double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance
double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
    
    
        //System.out.println(distanceFromLimelightToGoalInches);
        SmartDashboard.putNumber("inches from goal", distanceFromLimelightToGoalInches);
    }

    }
