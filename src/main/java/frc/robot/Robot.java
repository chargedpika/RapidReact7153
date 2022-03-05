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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.subsystems.THEGYRO;


import edu.wpi.first.cameraserver.CameraServer;


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
  private static Joystick DriveJoy = new Joystick(0), spinJoy = new Joystick(1), FXNJoy = new Joystick(2);
  private talonSRXwheel falconCode = new talonSRXwheel();
  //private falcon500 falcon = new falcon500(); //UNUSED UNTIL FURTHER NOTICE
  private solenoidCode Solonoids = new solenoidCode();
  private maxSpeed speedAdjust = new maxSpeed(0.3, 0.5);
  
  private THEGYRO gyro = new THEGYRO();

  CANSparkMax frontLeftSpark = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax frontRightSpark = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rearLeftSpark = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rearRightSpark = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax m_leftMotor = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax m_rightMotor = new CANSparkMax(8, MotorType.kBrushless); 

    //intakeWheel intakeWheel1 = new intakeWheel(); BRO WHAT IS THIS

    
  private double startTime;

  @Override
  public void robotInit() {
  
    CameraServer.startAutomaticCapture();
  

    // You may need to change or remove this to match your robot.
    rearRightSpark.setInverted(true);
    frontRightSpark.setInverted(true);

    m_rightMotor.setInverted(true); //THIS IS FOR THE SHOOTER

    m_robotDrive = new MecanumDrive(frontLeftSpark, rearLeftSpark, frontRightSpark, rearRightSpark);
    m_shooterControl = new DifferentialDrive(m_leftMotor, m_rightMotor);


    DriveJoy = new Joystick(kJoystickChannel);
    m_robotDrive.setDeadband(.1);
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
   double time = Timer.getFPGATimestamp(); //AUTONOMOUS CODE
    //System.out.println(time - startTime);
    SmartDashboard.putNumber("Auto Timer", time-startTime);

    if (time - startTime < 3) {
      m_robotDrive.driveCartesian(-.3, 0, 0);
    } else {
      m_robotDrive.driveCartesian(0, 0, 0);;
    }
  }


  @Override 
  public void teleopInit() {
    Solonoids.teleopStarted();
  }

  //public double getJoystickValue(Joystick joystick) {  //DEADBAND FOR USE LATER!!!!!!!!!!!
    //  if(Math.abs(joystick.getValue() < 0.1)) return 0;
    //  else return joystick.getValue();
 // }

  @Override
  public void teleopPeriodic() {
    speedAdjust.refresh();
     
    //m_robotDrive.driveCartesian(-DriveJoy.getY(), DriveJoy.getX(), DriveJoy.getZ(), 0.0); UNUSUED UNTIL FURTHER NOTICE
    m_robotDrive.driveCartesian(
      speedAdjust.applyMaxSpeed(DriveJoy.getY()),
      speedAdjust.applyMaxSpeed(-DriveJoy.getX()), 
      speedAdjust.applyMaxSpeed(-spinJoy.getZ()),
      gyro.getGyro()
    );
    



    if (FXNJoy.getTrigger()) {
      m_shooterControl.arcadeDrive(-0.7, 0);
    } else {
      m_shooterControl.arcadeDrive(0, 0);
    }
    falconCode.ballLift();
    //falcon.move(); //UNUSED UNTIL FURTHER NOTICE
    Solonoids.pistonMovement(); 
    falconCode.intakeWheel();
    }
}
//           :)