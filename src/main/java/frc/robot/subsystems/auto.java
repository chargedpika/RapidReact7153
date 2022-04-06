package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.util.Collection;
import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.mecanumOdometry;
import frc.robot.subsystems.solenoidCode;
import frc.robot.subsystems.talonSRXwheel;
import frc.robot.subsystems.autoCenter;

// Vision Imports
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.*;
import frc.robot.visiontargets.BlueBallPipeline;

public class auto {
    // Drive elements
    private DifferentialDrive shooter;
    private TalonFX greenWheel;
    private MecanumDrive mecDrive;
    private mecanumOdometry mecOdometry;
    private solenoidCode solenoid;
    private talonSRXwheel intake;
    private autoCenter limelight;

    // Vision Pipelines
    private final Object imgLock = new Object();
    private VisionThread visionThread;
    private double currentXTarget = 0;

    private int videoWidth = 320;
    private int videoHeight = 240;

    private UsbCamera camera1;

    // Auto
    double start;
    int step;

    public auto(
        DifferentialDrive shooterDrive, 
        mecanumOdometry odo, 
        TalonFX greenIntakeWheel, 
        MecanumDrive mecanumDrive, 
        UsbCamera cam1, 
        String robotColor, 
        solenoidCode solonoid,
        talonSRXwheel intakeWheel,
        autoCenter ll
    ) {
        shooter = shooterDrive;
        greenWheel = greenIntakeWheel;
        mecDrive = mecanumDrive;
        mecOdometry = odo;
        solenoid = solonoid;
        intake = intakeWheel;
        limelight = ll;

        camera1 = cam1;
        camera1.setResolution(videoWidth, videoHeight);
        
        visionThread = new VisionThread(
            camera1,
            (robotColor == "blue") ? new BlueBallPipeline() : new BlueBallPipeline(), // change
            pipeline -> {
                if (!pipeline.findBlobsOutput().empty()) {
                    KeyPoint point = (pipeline.findBlobsOutput().toList().get(0));
                    Point pt = point.pt;

                    synchronized (imgLock) {
                        currentXTarget = pt.x - (videoWidth/2);
                    }
                }
            }
        );
        //visionThread.start();
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
        mecDrive.driveCartesian(0.0, 0.0, 0.0);
    }

    public void autoPeriodic() {
        if (step == 0) {
            shooter.arcadeDrive(-0.6, 0.0);
            greenWheel.set(ControlMode.PercentOutput, 0.5);
            if (Timer.getFPGATimestamp() - start >= 1) { nextStep(); }
        } else if (step == 1) {
            mecDrive.driveCartesian(0.35, 0.0, 0.0);
            if (Timer.getFPGATimestamp() - start >= 3.0) { nextStep(); }
        } else if (step == 2) {
            mecDrive.driveCartesian(0.0, 0.0, 0.35);
            if (Timer.getFPGATimestamp() - start >= 1.4) { nextStep(); }
        } else if (step == 3) {
            //mecDrive.driveCartesian(0.0, 0.0, (currentXTarget > 0) ? -0.05 : 0.05);
        }
    }

    public void autoV2() {
        if (step == 0) {
            solenoid.goToState(true);
            intake.setIntakeState(true);
            nextStep();
        } else if (step == 1) {
            mecDrive.driveCartesian(0.35, 0.0, 0.0);
            if (Timer.getFPGATimestamp() - start >= 2.0) { nextStep(); }
        } else if (step == 2) {
            solenoid.goToState(false);
            intake.setIntakeState(false);
            nextStep();
        } else if (step == 3) {
            mecDrive.driveCartesian(0.0, 0.0, limelight.getTurn());
            if (limelight.isInTarget()) { nextStep(); }
        } else if (step == 4) {
            mecDrive.driveCartesian(0.35, 0.0, 0.0);
            if (Timer.getFPGATimestamp() - start > 0.5) { nextStep(); }
        } else if (step == 5) {
            shooter.arcadeDrive(limelight.getSuggestedSpeed(), 0.0);
            if (Timer.getFPGATimestamp() - start >= 1.0) { nextStep(); }
        } else if (step == 6) {
            greenWheel.set(ControlMode.PercentOutput, 0.5);
            if (Timer.getFPGATimestamp() - start >= 3.0) { nextStep(); }
        }
    }

    public void autoOdometry() {
        if (step == 0) {
            mecOdometry.updateSetpoints(3.0, 0.0, 0.0);
            mecOdometry.robotDisabled = false;
            step++;
        } else if (step == 1) {
            if (mecOdometry.setpointsDone) {
                mecOdometry.robotDisabled = true;
                nextStep();
            }
        }
    }
}
