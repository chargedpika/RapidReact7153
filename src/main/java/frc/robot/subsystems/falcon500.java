package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Date;
import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.TimeUnit;
public class falcon500 extends SubsystemBase {
    public TalonFX motor;
    public TalonFX daveWheel;
    private Joystick joystick;
   // private JoystickButton button2; // NOT USED, INDEXER IS ON SHOOTER NOW
    private JoystickButton button11;
    private JoystickButton button5;
    private JoystickButton button3;
    private double motorSpeed = 0.85; // .85

    private double daveSpeed = .5; 

    private double shooterBttnDown = -1;

    public falcon500() {
        motor = new TalonFX(10);
        //daveWheel = new TalonFX(11);
        joystick = new Joystick(2);
       // button2 = new JoystickButton(joystick, 2);// NOT USED, INDEXER IS ON SHOOTER NOW
        button11 = new JoystickButton(joystick, 11);
        button5 = new JoystickButton(joystick, 5);
        button3 = new JoystickButton(joystick, 3);
    }

    // NEW FALCON FX MOVEFORWARD
    public void move() {
                if (joystick.getTrigger()) {
                  if (shooterBttnDown == -1) {
                    shooterBttnDown = Timer.getFPGATimestamp();
                    motor.set(ControlMode.PercentOutput, 0.0);
                  } else if (Timer.getFPGATimestamp() - shooterBttnDown >= 0.9) {//was 2
                    motor.set(ControlMode.PercentOutput, motorSpeed);
                  } else {
                    motor.set(ControlMode.PercentOutput, 0.0);
                  }
                  //try {Thread.sleep(3000);
                      
                  //} catch (InterruptedException ex) {
                  //}  
                    //motor.set(ControlMode.PercentOutput, motorSpeed);
          } else {
            motor.set(ControlMode.PercentOutput, 0.0);
            shooterBttnDown = -1.0;
            //NEW REVERSE
            if (button11.get()) {
                motor.set(ControlMode.PercentOutput, -motorSpeed);
          }

    }
}
  /*public void winchMove() {
    if (button5.get()) {
      daveWheel.set(ControlMode.PercentOutput, daveSpeed);
    } else if (button3.get()) {
      daveWheel.set(ControlMode.PercentOutput, -daveSpeed);
    } else {
      daveWheel.set(ControlMode.PercentOutput, 0.0);
    }
  }*/
}
