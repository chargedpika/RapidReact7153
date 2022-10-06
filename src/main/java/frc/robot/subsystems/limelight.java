package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
//import frc.robot.subsystems.PID;
//import frc.robot.commands.UpdateCameraValues;
public class limelight extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
  NetworkTableEntry tx= table.getEntry("tx"); 
  NetworkTableEntry ty = table.getEntry("ty"); 
  NetworkTableEntry ta = table.getEntry("ta");
  public Boolean aligned = false;
  private PID pid = new PID(0.35,
                              0.2, 
                              0.1,
                               0.25, 
                               -0.2); 

	  public limelight() {

	}

public double alignX(double des){
	this.aligned = (Math.abs(getX()) > 0.2) ? false : true;
	return this.pid.calc(des, getX());
}

	public double getX(){
		return tx.getDouble(0.0000);
	}
  
  
	public double gety(){
		return ty.getDouble(0.0000); 
	}

public Boolean getAligned() {
	return aligned;
}



  @Override
  public void periodic() {
	SmartDashboard.putNumber("Limelight X", this.getX());

}
}

