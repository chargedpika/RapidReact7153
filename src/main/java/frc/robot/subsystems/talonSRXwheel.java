
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 */
public class talonSRXwheel extends SubsystemBase {
  private TalonSRX motor9;
  private TalonSRX motor10;

  private Joystick spinJoy;
  private Joystick fxnJoy;
  private JoystickButton button2;
  private JoystickButton button3;
  private JoystickButton button5;
  //private JoystickButton button11;
  private JoystickButton button12;
  private double motor9speed = .5;
  //private double motor10speed = -0.8; // We might need to change this later :)

  public talonSRXwheel() {
    motor9 = new TalonSRX(9);
    //motor10 = new TalonSRX(10);

    spinJoy = new Joystick(1);
    fxnJoy = new Joystick(2);

    //button5 = new JoystickButton(spinJoy, 3);
    //button3 = new JoystickButton(spinJoy, 5);

    button2 = new JoystickButton(fxnJoy, 2);
    //button11 = new JoystickButton(fxnJoy, 11);
    //button12 = new JoystickButton(fxnJoy, 12);
  }

  //public void ballLift() {
    //motor9.set(ControlMode.PercentOutput, joystick.getY());

    
    //if (button2.get()) {
     // motor10.set(ControlMode.PercentOutput, motor10speed);
   // } else {
     // motor10.set(ControlMode.PercentOutput, 0.0);
    //}
  

 public void intakeWheel() {
   // motor9.set(ControlMode.PercentOutput, joystick.getY());

   //INTAKE!!! IF YOU REMAP THIS, REMAP THE PISTON TOO
    if (spinJoy.getTrigger()) {
      motor9.set(ControlMode.PercentOutput, motor9speed);
    } else {
      motor9.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void setIntakeState(boolean state) {
    double speed = (state) ? 0.5 : 0.0;
    motor9.set(ControlMode.PercentOutput, speed);
  }
}
