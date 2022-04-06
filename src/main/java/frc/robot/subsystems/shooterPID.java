package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class shooterPID {
    private CANSparkMax shooter1;
    private CANSparkMax shooter2;

    private SparkMaxPIDController shooter1PID;

    private RelativeEncoder shooter1Encoder;
    private RelativeEncoder shooter2Encoder;

    private Double currentSpeed;
    
    public shooterPID(CANSparkMax s1, CANSparkMax s2) {
        shooter1 = s1;
        shooter2 = s2;

        shooter2.follow(shooter1, true);

        shooter1PID = shooter1.getPIDController();

        shooter1Encoder = shooter1.getEncoder();
        shooter2Encoder = shooter2.getEncoder();

        shooter1PID.setP(0.00008);
        shooter1PID.setI(6e-7);
        shooter1PID.setD(0.00002);
        shooter1PID.setIZone(0);
        shooter1PID.setFF(0.0);
        shooter1PID.setOutputRange(-1.0, 1.0);

        setSpeed(0.0);
    }

    public void setSpeed(Double speed) {
        currentSpeed = speed;
        shooter1PID.setReference(-currentSpeed, CANSparkMax.ControlType.kVelocity);

        if (speed <= 0.1 && speed >= -0.1) {
            shooter1.stopMotor();
            shooter2.stopMotor();
        }
    }

    public void refresh() {
        SmartDashboard.putNumber("Shooter 1 Encoder", shooter1Encoder.getVelocity());
        SmartDashboard.putNumber("Shooter 2 Encoder", shooter2Encoder.getVelocity());
    }
}
