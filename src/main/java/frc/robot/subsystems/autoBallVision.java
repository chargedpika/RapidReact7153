package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.mecanumOdometry;
import frc.robot.subsystems.solenoidCode;
import frc.robot.subsystems.talonSRXwheel;
import frc.robot.subsystems.autoCenter;
import frc.robot.subsystems.shooterPID;

// Vision Imports
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.*;
import frc.robot.visiontargets.BlueBallPipeline;

public class autoBallVision {
    // Drive elements
    private DifferentialDrive shooter;
    private TalonFX greenWheel;
    private MecanumDrive mecDrive;
    private mecanumOdometry mecOdometry;
    private solenoidCode solenoid;
    private talonSRXwheel intake;
    private autoCenter limelight;
    private shooterPID shootPID;

    // Vision Pipelines
    private final Object imgLock = new Object();
    private VisionThread visionThread;
    private double currentXTarget = 0;

    private int videoWidth = 320;
    private int videoHeight = 240;

    private UsbCamera camera1;
    boolean isBalling = true;
    boolean seesBalls = false;
    double camError = 1.0;

    // Auto
    double shootStart = -1.0;

    public autoBallVision(
        DifferentialDrive shooterDrive, 
        mecanumOdometry odo, 
        TalonFX greenIntakeWheel, 
        MecanumDrive mecanumDrive, 
        UsbCamera cam1, 
        String robotColor, 
        solenoidCode solonoid,
        talonSRXwheel intakeWheel,
        autoCenter ll,
        shooterPID PIDshoot
    ) {
        shooter = shooterDrive;
        greenWheel = greenIntakeWheel;
        mecDrive = mecanumDrive;
        mecOdometry = odo;
        solenoid = solonoid;
        intake = intakeWheel;
        limelight = ll;
        shootPID = PIDshoot;

        camera1 = cam1;
        camera1.setResolution(videoWidth, videoHeight);

        
        visionThread = new VisionThread(
            camera1,
            (robotColor == "blue") ? new BlueBallPipeline() : new BlueBallPipeline(), // change
            pipeline -> {
                if (!isBalling) { return; }
                boolean noBalls = pipeline.findBlobsOutput().empty();

                if (noBalls && seesBalls) {
                    // Picked up a ball
                    isBalling = false;
                    solenoid.goToState(false);
                    intake.setIntakeState(false);
                } else if (!noBalls) {
                    // Sees a ball
                    seesBalls = true;
                    List<KeyPoint> blobs = pipeline.findBlobsOutput().toList();
                    Collections.sort(blobs, new Comparator<KeyPoint>() {
                        public int compare(KeyPoint a, KeyPoint b) {
                            return Math.round(b.size - a.size);
                        }
                    });
                    Point pt = blobs.get(0).pt;
                    if (pt.x < -camError) {
                        // Need to turn left
                        mecDrive.driveCartesian(0.0, 0.0, -0.4);
                    } else if (pt.x > camError) {
                        // Needs to turn right
                        mecDrive.driveCartesian(0.0, 0.0, 0.4);
                    } else {
                        // Needs to go forward
                        mecDrive.driveCartesian(0.4, 0.0, 0.0);
                        solenoid.goToState(true);
                        intake.setIntakeState(true);
                    }
                } else {
                    // No balls
                    mecDrive.driveCartesian(0.0, 0.0, 0.4);
                }
            }
        );
        visionThread.start();
    }

    public void refresh() {
        if (isBalling) { return; }
        if (shootStart == -1.0 && !limelight.isInTarget()) {
            // Align limelight
            mecDrive.driveCartesian(0.0, 0.0, limelight.getTurn());
        } else if (shootStart == -1.0) {
            // Start shooter
            shootStart = Timer.getFPGATimestamp();
            shootPID.setSpeed(limelight.getSuggestedSpeed());
            greenWheel.set(ControlMode.PercentOutput, 0.85);
        } else if (Timer.getFPGATimestamp() - shootStart >= 2.0) {
            // Stop shooter (has shot)
            shootPID.setSpeed(0.0);
            greenWheel.set(ControlMode.PercentOutput, 0.0);
            isBalling = true;
            seesBalls = false;
        }
    }
}