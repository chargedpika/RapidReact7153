package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Timer;

public class auto {
    // Drive elements
    private DifferentialDrive shooter;
    private TalonFX greenWheel;
    private MecanumDrive mecDrive;

    double start;
    int step;

    public auto(DifferentialDrive shooterDrive, TalonFX greenIntakeWheel, MecanumDrive mecanumDrive) {
        shooter = shooterDrive;
        greenWheel = greenIntakeWheel;
        mecDrive = mecanumDrive;
    }

    public void autoStart() {
        step = 0;
        start = Timer.getFPGATimestamp();
    }

    private void nextStep() {
        step++;
        start = Timer.getFPGATimestamp();
        greenWheel.set(ControlMode.PercentOutput, 0.0);
        shooter.arcadeDrive(0.0, 0.0);
        mecDrive.stopMotor();
    }

    public void autoPeriodic() {
        if (step == 0) {
            if (Timer.getFPGATimestamp() - start >= 1) { nextStep(); }
            shooter.arcadeDrive(-0.6, 0.0);
            greenWheel.set(ControlMode.PercentOutput, 0.5);
        } else if (step == 1) {
            if (Timer.getFPGATimestamp() - start >= 1.5) { nextStep(); }
            mecDrive.driveCartesian(0.35, 0.0, 0.0);
        }
    }
}
