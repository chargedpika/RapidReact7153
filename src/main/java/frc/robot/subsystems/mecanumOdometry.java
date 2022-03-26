package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

public class mecanumOdometry {
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;
    private MecanumDrive drive;
    public AHRS gyro;
    private RelativeEncoder[] encoders;
    private PIDController[] pids = {
        new PIDController(0.35, 0.2, 0.1), // Forward/Backward
        new PIDController(0.35, 0.2, 0.1), // Left/Right
        new PIDController(0.35, 0.2, 0.1) // Rotation
    };
    
    public boolean setpointsDone = true;
    public double maxSpeed = 0.3;
    public boolean robotDisabled = true;

    public double rotationToMeterRate = 1.0287/20.0;

    public mecanumOdometry(
        CANSparkMax frontLeft, 
        CANSparkMax frontRight, 
        CANSparkMax rearLeft, 
        CANSparkMax rearRight,
        AHRS g,
        double distanceBetweenFrontAndBackWheels, 
        double distanceBetweenLeftAndRightWheels
    ) {
        // Assumes robot wheels are equally spaced and centered
        // Units are in meters and should be measured from the center of the wheels
        // Uses NavX gyroscope
        gyro = g;

        double halfWidth = Math.abs(distanceBetweenLeftAndRightWheels/2.0);
        double halfHeight = Math.abs(distanceBetweenFrontAndBackWheels/2.0);

        kinematics = new MecanumDriveKinematics(
            new Translation2d(-halfWidth, halfHeight), 
            new Translation2d(halfWidth, halfHeight), 
            new Translation2d(-halfWidth, -halfHeight), 
            new Translation2d(halfWidth, -halfHeight)
        );

        odometry = new MecanumDriveOdometry(
            kinematics, 
            Rotation2d.fromDegrees(gyro.getYaw()),
            new Pose2d(0.0, 0.0, new Rotation2d())
        );

        encoders = new RelativeEncoder[] {
            frontLeft.getEncoder(),
            frontRight.getEncoder(),
            rearLeft.getEncoder(),
            rearRight.getEncoder()
        };

        encoders[0].setVelocityConversionFactor(rotationToMeterRate);
        encoders[1].setVelocityConversionFactor(rotationToMeterRate);
        encoders[2].setVelocityConversionFactor(rotationToMeterRate);
        encoders[3].setVelocityConversionFactor(rotationToMeterRate);

        drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
        
        updateSetpoints(0.0, 0.0, 0.0);
    }

    private double clampSpeed(double input) {
        return Math.max(Math.min(input, maxSpeed), -maxSpeed);
    }

    public Pose2d update() {
        MecanumDriveWheelSpeeds wheelSpeed = new MecanumDriveWheelSpeeds(
            encoders[0].getVelocity()/60.0,
            encoders[1].getVelocity()/60.0,
            encoders[2].getVelocity()/60.0,
            encoders[3].getVelocity()/60.0
        );
        
        Pose2d pose = odometry.update(
            Rotation2d.fromDegrees(-gyro.getYaw()), // Gyro must be inverted
            wheelSpeed
        );
        
        if (!robotDisabled) {
            drive.driveCartesian(
                clampSpeed(pids[0].calculate(pose.getY())), 
                clampSpeed(pids[1].calculate(pose.getX())), 
                clampSpeed(pids[2].calculate(pose.getRotation().getDegrees()))
            );
        }

        setpointsDone = (pids[0].atSetpoint() && pids[1].atSetpoint() && pids[2].atSetpoint());
        
        return pose;
    }

    public Pose2d updateWithSmartDashboard() {
        Pose2d pose = update();

        SmartDashboard.putNumber("Pose X", pose.getX());
        SmartDashboard.putNumber("Pose Y", pose.getY());
        SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());

        return pose;
    }

    public void updateSetpoints(double newY, double newX, double newZ) {
        pids[0].setSetpoint(-newY);
        pids[1].setSetpoint(newX);
        pids[2].setSetpoint(newZ);
        reset();
        setpointsDone = false;
    }

    public void reset() {
        if (!setpointsDone) {
            System.out.println("!! Robot position as been reset during PID loop, may make robot move in unexpected directions!");
        }
        odometry.resetPosition(
            new Pose2d(0.0, 0.0, new Rotation2d()), 
            Rotation2d.fromDegrees(-gyro.getYaw())
        );
    }
}