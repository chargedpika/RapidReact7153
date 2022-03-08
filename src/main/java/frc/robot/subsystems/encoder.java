package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

public class encoder {
    private CANSparkMax motor;
    private RelativeEncoder relative;
    private SparkMaxPIDController pidController;

    public encoder(int canId) {
        motor = new CANSparkMax(canId, MotorType.kBrushless);
        relative = motor.getEncoder();
        pidController = motor.getPIDController();

        // PID Coefficents (sus)
        pidController.setP(0.1);
        pidController.setI(1e-4);
        pidController.setD(1);
        pidController.setIZone(0);
        pidController.setFF(0);

        pidController.setOutputRange(-1, 1); // Motor range
    }

    public double go(double rotations) {
        pidController.setReference(rotations, ControlType.kPosition);
        return relative.getPosition();
    }
}