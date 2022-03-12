package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Date;
import java.util.concurrent.TimeUnit;
public class falcon500 extends SubsystemBase {
    private TalonFX motor;
    private Joystick joystick;
   // private JoystickButton button2; // NOT USED, INDEXER IS ON SHOOTER NOW
    private JoystickButton button11;
    private double motorSpeed = .5;
    
    


    public falcon500() {
        motor = new TalonFX(10);
        joystick = new Joystick(2);
       // button2 = new JoystickButton(joystick, 2);// NOT USED, INDEXER IS ON SHOOTER NOW
        button11 = new JoystickButton(joystick, 11);
    }

    // NEW FALCON FX MOVEFORWARD
    public void move() {
                if (joystick.getTrigger()) {
                  //try {Thread.sleep(3000);
                      
                  //} catch (InterruptedException ex) {
                  //}  
                    motor.set(ControlMode.PercentOutput, motorSpeed);
          } else {
            motor.set(ControlMode.PercentOutput, 0.0);
            //NEW REVERSE
            if (button11.get()) {
                motor.set(ControlMode.PercentOutput, -motorSpeed);
          }

    }
}}
