// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class solenoidCode extends SubsystemBase {
  private final PneumaticsModuleType type = PneumaticsModuleType.REVPH; // Change this if needed :)
  
  private final Joystick spinJoy = new Joystick(1);
  private final Joystick fxnJoy = new Joystick(2);
  //private final JoystickButton TURNON = new JoystickButton(spinJoy, 3);
  //private final JoystickButton TURNOFF = new JoystickButton(spinJoy, 5);
  private final JoystickButton goUp = new JoystickButton(fxnJoy, 4);
  private final JoystickButton goDown = new JoystickButton(fxnJoy, 6);
  //private final JoystickButton winchUp = new JoystickButton(m_stick, )


  // DoubleSolenoid corresponds to a double solenoid.
  //private final DoubleSolenoid l_doubleSolenoid = new DoubleSolenoid(2, type, 8, 9);
  //private final DoubleSolenoid r_doubleSolenoid = new DoubleSolenoid(2, type, 6, 7);

  private final DoubleSolenoid frontRight_doubleSolenoid = new DoubleSolenoid(2, type, 9, 8);
  private final DoubleSolenoid frontLeft_doubleSolenoid = new DoubleSolenoid(2, type, 7, 6);
  private final DoubleSolenoid barGrabberLeft_doubleSolenoid = new DoubleSolenoid(2, type, 3, 2);
  private final DoubleSolenoid barGrabberRight_DoubleSolenoid = new DoubleSolenoid(2, type, 1, 0);
  //private final DoubleSolenoid whinchSolenoid = new DoubleSolenoid(2, type, 1, 0);

  //private final Compressor comp = new Compressor(2, type);
  //Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);


  public void teleopStarted() {
    //comp.disable();
  }


  //private static final int kSolenoidButton = 2;
  //private static final int kDoubleSolenoidForward = 5;
  //private static final int kDoubleSolenoidReverse = 3;

  public void pistonMovement() {
    /*
     * The output of GetRawButton is true/false depending on whether
     * the button is pressed; Set takes a boolean for whether
     * to use the default (false) channel or the other (true).
     */
    //m_solenoid.set(m_stick.getRawButton(kSolenoidButton));

    /*
     * In order to set the double solenoid, if just one button
     * is pressed, set the solenoid to correspond to that button.
     * If both are pressed, set the solenoid will be set to Forwards.
     */
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

    /*if (fxnJoy.getTrigger()) {
      System.out.println(comp.getPressureSwitchValue());
      System.out.println(comp.enabled());
      comp.enableDigital();
    }*/
    
  }
}
