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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController; //harmless potential import

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
  private static Joystick DriveJoy = new Joystick(0), FXNJoy = new Joystick(1);
  private talonSRXwheel falconCode = new talonSRXwheel();
  private falcon500 falcon = new falcon500();
  private solenoidCode moverThingy = new solenoidCode();

  //private final XboxController m_driverController = new XboxController(0); //THIS IS FOR IF WE USE AN XBOX CONTROLER. 
  //BUTTONS ARENT MAPPED YET


  @Override
  public void robotInit() {
    //intakeWheel intakeWheel1 = new intakeWheel();
    
    CANSparkMax frontLeftSpark = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax frontRightSpark = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax rearLeftSpark = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax rearRightSpark = new CANSparkMax(6, MotorType.kBrushless);
   
    CANSparkMax m_leftMotor = new CANSparkMax(7, MotorType.kBrushless);
    CANSparkMax m_rightMotor = new CANSparkMax(8, MotorType.kBrushless);
    m_shooterControl = new DifferentialDrive(m_leftMotor, m_rightMotor);

    CameraServer.startAutomaticCapture();
  

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRightSpark.setInverted(false);
    rearRightSpark.setInverted(false); 
    m_rightMotor.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeftSpark, rearLeftSpark, frontRightSpark, rearRightSpark);

    DriveJoy = new Joystick(kJoystickChannel);
    m_robotDrive.setDeadband(.1);
  }

  @Override 
  public void teleopInit() {
    moverThingy.teleopStarted();
  }

  @Override
  public void teleopPeriodic() {
     
    m_robotDrive.driveCartesian(-DriveJoy.getY(), DriveJoy.getX(), DriveJoy.getZ(), 0.0);
    //m_robotDrive.driveCartesian(-m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX());
    //XBOX CONTROLER CODE ABOVE



    if (FXNJoy.getTrigger()) {
      m_shooterControl.arcadeDrive(-0.6, 0);
    } else {
      m_shooterControl.arcadeDrive(0, 0);
    }
    falconCode.ballLift(); // ignore this pls
    //falcon.move(); unused until further notice.
    moverThingy.amogus(); 
    falconCode.intakeWheel();
    }
}
