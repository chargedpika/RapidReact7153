package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class falcon500 extends SubsystemBase {
    private TalonFX motor;
    private Joystick joystick;

    public falcon500() {
        motor = new TalonFX(11);
        joystick = new Joystick(2);
    }

    public void move() {
        motor.set(ControlMode.PercentOutput, joystick.getThrottle());
    }
}
