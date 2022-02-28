/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// why do we have this?????????
// - jonah
// It's for controlling the flow, pressure, speed, and temp (more stuff too) 
// Using differential and partial differentials (the math of change)
// You wouldn't want to manually do the math, trust me
// - Matt


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class PID {
    Timer time = new Timer();
    Timer check  =new Timer();
    

    private double kP, kI, kD;
    private double pastError = 0;
    private double integAcum = 0;
    private double Hthreshold = 0;
    private double Lthreshold = 0;

    private double pastVal ;
    private double curVal  ;
    private double actualRun ;


    public PID(double kP, double kI, double kD, double Hthreshold, double Lthreshold){

        this.kP =kP;
        this.kI = kI;
        this.kD = kD;
        this.Hthreshold = Hthreshold;
        this.Lthreshold = Lthreshold;
        pastError =0 ;
        curVal = 0;
        actualRun=0;
    }

    public double getError(double des, double real){
        return des - real;
    }
    
 
    private double P(double error){return this.kP * error;}

    private double I(double error, double dt){
        double midpoint = ((error + pastError)/2)*kI;
        double integral = (midpoint * dt) + integAcum; //something to fix
        this.integAcum = integral;
        return this.kI * integral;
    }
    private double D(double error, double dt){
        double dy = (error - pastError)/dt;
        this.pastError = error;
        return this.kD * (dy);
    }


    public double calc(double des, double current){
        time.stop();
        double dt = time.get(); 
        time.reset();
        time.start();

        double error = getError(des, current);
        double pidVal = P(error) + I(error, dt)  + D(error,dt) ;
        
//        System.out.println(error);
        //double pidVal = P(error) + I(error, dt)  + D(error,dt) ; 
        double minMaxNorm = Math.min(Math.max(pidVal, Lthreshold), Hthreshold);  
        pastVal = pastError;
        curVal = pidVal; 
        actualRun = minMaxNorm;
        selfCheck(pastVal, curVal, actualRun);
       return minMaxNorm; 
    }

   public void resetPID(){
       time.reset();
   }
   
   public void selfCheck(double Past, double Current,double Actual){
       check.start();
    if (curVal>0 && (Current-Past)>0){
        SmartDashboard.delete("statue");
        SmartDashboard.putString("statue","the error is increasing");
       }
    else if(curVal<0){
        SmartDashboard.delete("statue");
        SmartDashboard.putString("statue","overshotting, input value is negative, error should go back");
      }
   else{
       SmartDashboard.delete("statue");
       SmartDashboard.putString("statue", "everything right");
   } 
} 
}
