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

public class telemetry {
    // PDH
    private PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);

    // Encoders
    private RelativeEncoder frontLeft;
    private RelativeEncoder frontRight;
    private RelativeEncoder rearLeft;
    private RelativeEncoder rearRight;

    // Gyro
    private AHRS gyro;

    public telemetry(CANSparkMax fl, CANSparkMax fr, CANSparkMax rl, CANSparkMax rr, AHRS g) {
        frontLeft = fl.getEncoder();
        frontRight = fr.getEncoder();
        rearLeft = rl.getEncoder();
        rearRight = rr.getEncoder();

        gyro = g;
        //gyro.calibrate();
    }

    public void refresh() {
        // PDH
        SmartDashboard.putNumber("PDH Voltage", pd.getVoltage());
        SmartDashboard.putNumber("PDH Current", pd.getTotalCurrent());
        SmartDashboard.putNumber("PDH Temperature", pd.getTemperature());

        PowerDistributionFaults faults = pd.getFaults();
        SmartDashboard.putBoolean("PDH Brownout", !faults.Brownout);
        
        Boolean[] channelFaults = {
            faults.Channel0BreakerFault,
            faults.Channel1BreakerFault,
            faults.Channel2BreakerFault,
            faults.Channel3BreakerFault,
            faults.Channel4BreakerFault,
            faults.Channel5BreakerFault,
            faults.Channel6BreakerFault,
            faults.Channel7BreakerFault,
            faults.Channel8BreakerFault,
            faults.Channel9BreakerFault,
            faults.Channel10BreakerFault,
            faults.Channel11BreakerFault,
            faults.Channel12BreakerFault,
            faults.Channel13BreakerFault,
            faults.Channel14BreakerFault,
            faults.Channel15BreakerFault,
            faults.Channel16BreakerFault,
            faults.Channel17BreakerFault,
            faults.Channel18BreakerFault,
            faults.Channel19BreakerFault,
            faults.Channel20BreakerFault,
            faults.Channel21BreakerFault,
            faults.Channel22BreakerFault,
            faults.Channel23BreakerFault
        };
        String faultMessage = "";
        for (int x = 0; x < channelFaults.length; x++) {
            if (channelFaults[x]) {
                faultMessage += (x + ", ");
            }
        }
        SmartDashboard.putString("Channel Faults", faultMessage);

        // Encoders
        SmartDashboard.putNumber("Front Left", -frontLeft.getPosition());
        SmartDashboard.putNumber("Front Right", -frontRight.getPosition());
        SmartDashboard.putNumber("Rear Left", -rearLeft.getPosition());
        SmartDashboard.putNumber("Rear Right", -rearRight.getPosition());
        
        // Gyro
        /*if (!gyro.isCalibrating()) {
            SmartDashboard.putNumber("Gyro X", gyro.getRoll());
            SmartDashboard.putNumber("Gyro Y", gyro.getPitch());
            SmartDashboard.putNumber("Gyro Z", gyro.getYaw());

            SmartDashboard.putNumber("Gyro X Speed", gyro.getRawGyroX());
            SmartDashboard.putNumber("Gyro Y Speed", gyro.getRawGyroY());
            SmartDashboard.putNumber("Gyro Z Speed", gyro.getRawGyroZ());
        }*/
        SmartDashboard.putBoolean("NavX Enabled", gyro.isConnected());
    }
}