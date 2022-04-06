// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/*
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
*/
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.talonSRXwheel;
//import frc.robot.subsystems.falcon500; //UNUSED UNTIL FURTHER NOTICE
import frc.robot.subsystems.solenoidCode;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.maxSpeed;
import com.revrobotics.RelativeEncoder;
//import frc.robot.subsystems.THEGYRO;
import edu.wpi.first.cscore.UsbCamera;
//import frc.robot.subsystems.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.falcon500;
import frc.robot.subsystems.auto;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.telemetry;
import frc.robot.subsystems.mecanumOdometry;
import frc.robot.subsystems.autoCenter;
import frc.robot.subsystems.shooterPID;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  /*private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 0;
*/


  
  private static final int kJoystickChannel = 0;

  private DifferentialDrive m_shooterControl;
  private MecanumDrive m_robotDrive;
  private auto autoControl;
  private static Joystick DriveJoy = new Joystick(0), spinJoy = new Joystick(1), FXNJoy = new Joystick(2);
  private talonSRXwheel falconCode = new talonSRXwheel();
  private solenoidCode Solonoids = new solenoidCode();
  private falcon500 FALCONCODE = new falcon500();
  private maxSpeed speedAdjust = new maxSpeed(1, 0.4, 0.8);
  private maxSpeed shooterSpeed = new maxSpeed(2, 0.89, 0.2);
  
  //private THEGYRO gyro = new THEGYRO();

  CANSparkMax frontLeftSpark = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax frontRightSpark = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rearLeftSpark = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rearRightSpark = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax m_leftMotor = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax m_rightMotor = new CANSparkMax(8, MotorType.kBrushless);

  public UsbCamera frontCamera;

  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  public telemetry telemetrySteam = new telemetry(
    frontLeftSpark,
    frontRightSpark,
    rearLeftSpark,
    rearRightSpark,
    gyro
  );
  public mecanumOdometry odometry;
  public JoystickButton autoCenterBttn = new JoystickButton(FXNJoy, 2);
  public autoCenter center = new autoCenter();
  public shooterPID shootPID;
  //RelativeEncoder frontLeftEncoder = frontLeftSpark.getEncoder();
  /*
  encoder frontLeftEncoder = new encoder(frontLeftSpark);
  encoder frontRightEncoder = new encoder(frontRightSpark);
  encoder rearLeftEncoder = new encoder(rearLeftSpark);
  encoder rearRightEncoder = new encoder(rearRightSpark);
*/
    //intakeWheel intakeWheel1 = new intakeWheel(); BRO WHAT IS THIS

  private double startTime;
  private NetworkTableEntry shooterSpeedSlider;

  @Override
  public void robotInit() {
    
    frontCamera = CameraServer.startAutomaticCapture(0);
  

    // You may need to change or remove this to match your robot.
    rearRightSpark.setInverted(true);
    frontRightSpark.setInverted(true);

    //m_rightMotor.setInverted(true); //THIS IS FOR THE SHOOTER

    m_robotDrive = new MecanumDrive(frontLeftSpark, rearLeftSpark, frontRightSpark, rearRightSpark);
    //m_shooterControl = new DifferentialDrive(m_leftMotor, m_rightMotor);
    autoControl = new auto(
      m_shooterControl,
      odometry,
      falcon500.motor,
      m_robotDrive,
      frontCamera,
      "blue",
      Solonoids,
      falconCode,
      center
    );

    odometry = new mecanumOdometry(
    frontLeftSpark,
    frontRightSpark,
    rearLeftSpark,
    rearRightSpark,
    gyro,
    0.5207,
    0.508,
    m_robotDrive
  );

    shootPID = new shooterPID(m_leftMotor, m_rightMotor);
    DriveJoy = new Joystick(kJoystickChannel);
    m_robotDrive.setDeadband(.2);

    shooterSpeedSlider = Shuffleboard.getTab("Shoot Test")
    .add("Shooter Speed", 3000.0)
    .withWidget(BuiltInWidgets.kTextView)
    .withProperties(Map.of("min", 3000.0, "max", 6000.0))
    .getEntry();
  }

  @Override
  public void robotPeriodic() {
    telemetrySteam.refresh();
    Solonoids.refreshValues();
    odometry.updateWithSmartDashboard();
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    autoControl.autoStart();
  }

  @Override
  public void autonomousPeriodic() {
    //autoControl.autoPeriodic();
    autoControl.autoV2();
   /*double time = Timer.getFPGATimestamp(); //AUTONOMOUS CODE
    //System.out.println(time - startTime);
    SmartDashboard.putNumber("Auto Timer", time-startTime);

    if (time - startTime < 3) {
      m_robotDrive.driveCartesian(-.3, 0, 0);
    } else {
      m_robotDrive.driveCartesian(0, 0, 0);
    }*/
    /*frontLeftEncoder.go(4);
    frontRightEncoder.go(4);
    rearLeftEncoder.go(4);
    rearRightEncoder.go(4);*/
  }

  @Override 
  public void teleopInit() {
    //Solonoids.teleopStarted();
  }

  @Override
  public void teleopPeriodic() {
    speedAdjust.refresh();
    shooterSpeed.refresh();
     
    //m_robotDrive.driveCartesian(-DriveJoy.getY(), DriveJoy.getX(), DriveJoy.getZ(), 0.0); UNUSUED UNTIL FURTHER NOTICE
    if (autoCenterBttn.get()) {
      m_robotDrive.driveCartesian(0.0, 0.0, center.getTurn());
    } else {
      m_robotDrive.driveCartesian(
        speedAdjust.applyMaxSpeed(DriveJoy.getY()),
        speedAdjust.applyMaxSpeed(-DriveJoy.getX()), 
        speedAdjust.applyMaxSpeed(-spinJoy.getZ())
      );
    }


    //double speed = shooterSpeedSlider.getDouble(0.0);

    //SmartDashboard.putNumber("Shooter Speed !", speed);
    if (FXNJoy.getTrigger()) {
      //m_shooterControl.arcadeDrive(-shooterSpeed.currentMax, 0);
      // :)
      //m_shooterControl.arcadeDrive(speed, 0.0);
      //shootPID.setSpeed(speed);
      if (FXNJoy.getThrottle() > 0) {
        // Low goal
        SmartDashboard.putString("Goal", "low");
        shootPID.setSpeed(1800.0);
      } else {
        // High goal
        SmartDashboard.putString("Goal", "high");
        double speed = center.getSuggestedSpeed();
        SmartDashboard.putNumber("Shooter Speed", speed);
        shootPID.setSpeed(speed);
      }
    } else {

      //m_shooterControl.arcadeDrive(0, 0);
      shootPID.setSpeed(0.0);
    }
    //falconCode.ballLift();
    FALCONCODE.move(); //NOW BEING USED 
    FALCONCODE.winchMove();
    Solonoids.pistonMovement(); 
    falconCode.intakeWheel();
    //center.distanceGauge();
    shootPID.refresh();
    }

    @Override
    public void disabledInit() {
      odometry.robotDisabled = true;
    }

    // Measuring Rotation to Meter ratio
    RelativeEncoder[] test_encoders;
    Double[] test_startRotations;

    @Override
    public void testInit() {
      test_encoders = new RelativeEncoder[] {
        frontLeftSpark.getEncoder(),
        frontRightSpark.getEncoder(),
        rearLeftSpark.getEncoder(),
        rearRightSpark.getEncoder()
      };

      test_startRotations = new Double[] {
        test_encoders[0].getPosition(),
        test_encoders[1].getPosition(),
        test_encoders[2].getPosition(),
        test_encoders[3].getPosition()
      };

      m_robotDrive.driveCartesian(-0.4, 0.0, 0.0);
    }

    @Override
    public void testPeriodic() {
      double dist = Math.abs(
        (test_encoders[0].getPosition()-test_startRotations[0]) +
        (test_encoders[1].getPosition()-test_startRotations[1]) +
        (test_encoders[2].getPosition()-test_startRotations[2]) +
        (test_encoders[3].getPosition()-test_startRotations[3])
      )/4;
      System.out.println(dist);
      if (dist >= 20.0) {
        m_robotDrive.driveCartesian(0.0, 0.0, 0.0);
      } else {
        m_robotDrive.driveCartesian(-0.4, 0.0, 0.0);
      }
    }
}
//           :) (: 