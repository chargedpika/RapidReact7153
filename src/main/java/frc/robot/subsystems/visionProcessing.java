package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import frc.robot.vision.RedBallVision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class visionProcessing {
    private final Object imgLock = new Object();
    private VisionThread visionThread;
    private UsbCamera camera ;
    private double currentX;
    
    private int width;
    private int height;

    private double maxSpeed = 0.05;
    private double acceptableError = 0.1;

    public visionProcessing(int w, int h) {
        camera = CameraServer.startAutomaticCapture(1);
        camera.setResolution(width, height);

        width = w;
        height = h;

        visionThread = new VisionThread(
        camera, 
        new RedBallVision(), 
        pipeline -> {
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    currentX = ((r.x + (r.width / 2)) - (width/2))-10;
                    //currentX = r.x + (r.width / 2);
                    System.out.println("X position: " + currentX);
                }
            }
        });
    }

    public void start() {
        visionThread.start();
    }

    public void end() {
        visionThread.interrupt();
    }

    public double getCurrentPosition() {
        return currentX;
    }

    public double getRobotTarget() {
        if (currentX > acceptableError) {
            return -maxSpeed;
        } else if (currentX < -acceptableError) {
            return maxSpeed;
        } else {
            return 0.2;
        }
    }
}
