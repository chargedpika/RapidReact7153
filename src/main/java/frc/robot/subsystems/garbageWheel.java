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
public class garbageWheel extends SubsystemBase {
  private TalonSRX motor9;
  private TalonSRX motor10;

  private Joystick joystick;
  private JoystickButton button7;
  private JoystickButton button8;

  private double motor10speed = 1; // We might need to change this later :)

  public garbageWheel() {
    motor9 = new TalonSRX(9);
    motor10 = new TalonSRX(10);

    joystick = new Joystick(2);
    button7 = new JoystickButton(joystick, 7);
    button8 = new JoystickButton(joystick, 8);
  }

  public void minerMyBeloved() {
    motor9.set(ControlMode.PercentOutput, joystick.getY());

    if (button7.get()) {
      motor10.set(ControlMode.PercentOutput, motor10speed);
    } else if (button8.get()) {
      motor10.set(ControlMode.PercentOutput, 0-motor10speed);
    } else {
      motor10.set(ControlMode.PercentOutput, 0.0);
    }
  }
}
