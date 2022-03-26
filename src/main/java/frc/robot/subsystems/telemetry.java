package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// PDH
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// Encoders
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

// Gyro
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.SPI;

public class telemetry {
    // PDH
    private PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);

    // Encoders
    private RelativeEncoder frontLeft;
    private RelativeEncoder frontRight;
    private RelativeEncoder rearLeft;
    private RelativeEncoder rearRight;

    // Gyro
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public telemetry(CANSparkMax fl, CANSparkMax fr, CANSparkMax rl, CANSparkMax rr) {
        frontLeft = fl.getEncoder();
        frontRight = fr.getEncoder();
        rearLeft = rl.getEncoder();
        rearRight = rr.getEncoder();

        Shuffleboard.getTab("Gyro").add(gyro);
    }

    public void refresh() {
        // PDH
        SmartDashboard.putNumber("PDH Voltage", pd.getVoltage());
        SmartDashboard.putNumber("PDH Current", pd.getTotalCurrent());
        SmartDashboard.putNumber("PDH Temperature", pd.getTemperature());

        PowerDistributionFaults faults = pd.getFaults();
        SmartDashboard.putBoolean("PDH Brownout", !faults.Brownout);

        // Encoders
        SmartDashboard.putNumber("Front Left", -frontLeft.getPosition());
        SmartDashboard.putNumber("Front Right", -frontRight.getPosition());
        SmartDashboard.putNumber("Rear Left", -rearLeft.getPosition());
        SmartDashboard.putNumber("Rear Right", -rearRight.getPosition());

        // Gyro
        //SmartDashboard.putNumber("Gyro X", gyro.getRoll());
        //SmartDashboard.putNumber("Gyro Y", gyro.getPitch());
        //SmartDashboard.putNumber("Gyro Z", gyro.getYaw());

        //SmartDashboard.putNumber("Gyro X Speed", gyro.getRawGyroX());
        //SmartDashboard.putNumber("Gyro Y Speed", gyro.getRawGyroY());
        //SmartDashboard.putNumber("Gyro Z Speed", gyro.getRawGyroZ());
    }
}
