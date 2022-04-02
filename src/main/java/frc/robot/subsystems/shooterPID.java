package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class shooterPID {
    private CANSparkMax shooter1;
    private CANSparkMax shooter2;

    private SparkMaxPIDController shooter1PID;
    private SparkMaxPIDController shooter2PID;

    private RelativeEncoder shooter1Encoder;
    private RelativeEncoder shooter2Encoder;

    private Double currentSpeed;
    
    public shooterPID(CANSparkMax s1, CANSparkMax s2) {
        shooter1 = s1;
        shooter2 = s2;

        shooter1PID = shooter1.getPIDController();
        shooter2PID = shooter2.getPIDController();

        shooter1Encoder = shooter1.getEncoder();
        shooter2Encoder = shooter2.getEncoder();

        shooter1PID.setP(1e-4);
        shooter1PID.setI(1e-6);
        shooter1PID.setD(0);
        shooter1PID.setIZone(0);
        shooter1PID.setFF(0.000015);
        shooter1PID.setOutputRange(-1.0, 1.0);

        shooter2PID.setP(1e-4);
        shooter2PID.setI(1e-6);
        shooter2PID.setD(0);
        shooter2PID.setIZone(0);
        shooter2PID.setFF(0.000015);
        shooter2PID.setOutputRange(-1.0, 1.0);

        currentSpeed = 0.0;

        setSpeed(0.0);
    }

    public void setSpeed(Double speed) {
        currentSpeed = speed;
        shooter1PID.setReference(-currentSpeed, CANSparkMax.ControlType.kVelocity);
        shooter2PID.setReference(-currentSpeed, CANSparkMax.ControlType.kVelocity); //disconnected
    }

    public void refresh() {
        SmartDashboard.putNumber("Shooter 1 Encoder", shooter1Encoder.getVelocity());
        SmartDashboard.putNumber("Shooter 2 Encoder", shooter2Encoder.getVelocity());
    }
}
