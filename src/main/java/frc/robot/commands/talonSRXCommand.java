/*
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class talonSRXCommand extends CommandBase {
  TalonSRX wheel = new TalonSRX(9);
  Double percent;

  public talonSRXCommand(int id, Double p) {
    System.out.println("#2 ran :)");
   TalonSRX wheel = new TalonSRX(id);             rest in peace talonSRX command
    percent = p;                                  alas, we hardly used ye
  }

  @Override
  public void execute() {
    System.out.println("Execute ran :)");
    wheel.set(ControlMode.PercentOutput, percent);
  }
}
*/