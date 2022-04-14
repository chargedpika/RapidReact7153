// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class solenoidCode extends SubsystemBase {
  private final PneumaticsModuleType PHType = PneumaticsModuleType.REVPH; //we're using both now 
  private Compressor comp = new Compressor(2, PHType);
  private final PneumaticsModuleType PCMType = PneumaticsModuleType.CTREPCM;
  
  private final Joystick spinJoy = new Joystick(1);
  private final Joystick fxnJoy = new Joystick(2);
  //private final JoystickButton TURNON = new JoystickButton(spinJoy, 3);
  //private final JoystickButton TURNOFF = new JoystickButton(spinJoy, 5);
  private final JoystickButton goUp = new JoystickButton(fxnJoy, 4);
  private final JoystickButton goDown = new JoystickButton(fxnJoy, 6);
  private final JoystickButton witchLock = new JoystickButton(fxnJoy, 12);
  //private final JoystickButton winchUp = new JoystickButton(m_stick, )
  


  
  // DoubleSolenoid corresponds to a double solenoid.
  //private final DoubleSolenoid l_doubleSolenoid = new DoubleSolenoid(2, type, 8, 9);
  //private final DoubleSolenoid r_doubleSolenoid = new DoubleSolenoid(2, type, 6, 7);

  private final DoubleSolenoid frontRight_doubleSolenoid = new DoubleSolenoid(12, PCMType, 7, 6);
  private final DoubleSolenoid frontLeft_doubleSolenoid = new DoubleSolenoid(12, PCMType, 5, 4);
  private final DoubleSolenoid barGrabberLeft_doubleSolenoid = new DoubleSolenoid(2, PHType, 3, 2);
  private final DoubleSolenoid barGrabberRight_DoubleSolenoid = new DoubleSolenoid(2, PHType, 1, 0);
  //private final Solenoid whinchSolenoid = new (, 4);
  //private final DoubleSolenoid whinchSolenoid = new DoubleSolenoid(2, type, 1, 0)
  ;

  //private final Compressor comp = new Compressor(2, type);
  //Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  public solenoidCode(){
    comp.enableDigital();
  }


  public void teleopStarted() {
    //comp.disable();
  }


  //private static final int kSolenoidButton = 2;
  //private static final int kDoubleSolenoidForward = 5;
  //private static final int kDoubleSolenoidReverse = 3;

  public void pistonMovement() {
    if (!spinJoy.getTrigger()) {
      frontLeft_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      frontRight_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      frontLeft_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      frontRight_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    if (goUp.get()) {
      barGrabberLeft_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      barGrabberRight_DoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (goDown.get()) {
      barGrabberRight_DoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      barGrabberLeft_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    //if (witchLock.get()) {
//whinchSolenoid.set(true);
    //}
  }

  public void goToState(boolean state) {
    DoubleSolenoid.Value val = (state) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    frontLeft_doubleSolenoid.set(val);
    frontRight_doubleSolenoid.set(val);
  }

  public void refreshValues() {
    SmartDashboard.putBoolean("Full Pressure?", !comp.getPressureSwitchValue());
    SmartDashboard.putNumber("Pressure (PSI)", comp.getPressure());
  }
}
