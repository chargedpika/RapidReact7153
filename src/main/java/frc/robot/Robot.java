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
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.talonSRXwheel;
import frc.robot.subsystems.falcon500;
import frc.robot.subsystems.solenoidCode;
//import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.XboxController; //harmless potential import
//import edu.wpi.first.wpilibj.Timer;


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
  private falcon500 falcon = new falcon500();
  private solenoidCode Solonoids = new solenoidCode();

  //private final XboxController m_driverController = new XboxController(0); //THIS IS FOR IF WE USE AN XBOX CONTROLER. 
  //BUTTONS ARENT MAPPED YET
  
  
  
  //private double startTime;

  @Override
  public void robotInit() {
    CANSparkMax frontLeftSpark = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax frontRightSpark = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rearLeftSpark = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rearRightSpark = new CANSparkMax(6, MotorType.kBrushless);
 
  CANSparkMax m_leftMotor = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax m_rightMotor = new CANSparkMax(8, MotorType.kBrushless); 
    //intakeWheel intakeWheel1 = new intakeWheel();
    
  

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
  //  startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
   //double time = Timer.getFPGATimestamp();
    //System.out.println(time - startTime);

    //if (time - startTime < 3) {
      //frontLeftSpark.set(.6);
      //frontRightSpark.set(.6);
      //rearLeftSpark.set(.6);
      //rearRightSpark.set(.6);
    //} else {
      //frontLeftSpark.set(0);
      //frontRightSpark.set(0);
      //rearLeftSpark.set(0);
      //rearRightSpark.set(0);
    //}
  }


  @Override 
  public void teleopInit() {
    Solonoids.teleopStarted();
  }

  //public double getJoystickValue(Joystick joystick) {
    //  if(Math.abs(joystick.getValue() < 0.1)) return 0;
    //  else return joystick.getValue();
 // }

  @Override
  public void teleopPeriodic() {
     
    //m_robotDrive.driveCartesian(-DriveJoy.getY(), DriveJoy.getX(), DriveJoy.getZ(), 0.0);
   m_robotDrive.driveCartesian(DriveJoy.getY(), -DriveJoy.getX(), -spinJoy.getZ());
    //m_robotDrive.driveCartesian(-m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX());
    //XBOX CONTROLER CODE ABOVE



    if (FXNJoy.getTrigger()) {
      m_shooterControl.arcadeDrive(-0.6, 0);
    } else {
      m_shooterControl.arcadeDrive(0, 0);
    }
    falconCode.ballLift();
    //falcon.move(); unused until further notice.
    Solonoids.pistonMovement(); 
    falconCode.intakeWheel();
    }
}
