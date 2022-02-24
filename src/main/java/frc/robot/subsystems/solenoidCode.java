// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class solenoidCode extends SubsystemBase {
  private final PneumaticsModuleType type = PneumaticsModuleType.REVPH; // Change this if needed :)
  
  private final Joystick m_stick = new Joystick(2);
  private final JoystickButton TURNON = new JoystickButton(m_stick, 3);
  private final JoystickButton TURNOFF = new JoystickButton(m_stick, 5);

  // Solenoid corresponds to a single solenoid.
  //private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 2); 

  // DoubleSolenoid corresponds to a double solenoid.
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(2, type, 1, 2);
  private final Compressor comp = new Compressor(2, type);
  //Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);


  public void teleopStarted() {
    //comp.disable();
  }


  //private static final int kSolenoidButton = 2;
  //private static final int kDoubleSolenoidForward = 5;
  //private static final int kDoubleSolenoidReverse = 3;

  public void amogus() {
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
    if (TURNON.get()) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (TURNOFF.get()) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    if (m_stick.getTrigger()) {
      System.out.println(comp.getPressureSwitchValue());
      System.out.println(comp.enabled());
      comp.enableDigital();
    }
    
  }
}
