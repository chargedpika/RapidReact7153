package frc.robot.subsystems;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class pdh {
    private PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);

    public void refresh() {
        SmartDashboard.putNumber("PDH Voltage", pd.getVoltage());
        SmartDashboard.putNumber("PDH Current", pd.getTotalCurrent());
        //SmartDashboard.putNumber("PDH Power", pd.getTotalPower());
        //SmartDashboard.putNumber("PDF Energy", pd.getTotalEnergy());
        SmartDashboard.putNumber("PDH Temperature", pd.getTemperature());

        PowerDistributionFaults faults = pd.getFaults();
        SmartDashboard.putBoolean("PDH Brownout", !faults.Brownout);
    }
}
