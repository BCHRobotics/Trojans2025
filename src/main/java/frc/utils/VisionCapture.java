package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

/*
 * storing an estimated pose, the odometry reading at the time, and the capture time
 */
public class VisionCapture {
    public Pose2d visionMeasurement;
    public Pose2d odometryMeasurement;

    public double timeTaken;

    public VisionCapture(Pose2d vision, Pose2d odometry) {
        visionMeasurement = vision;
        odometryMeasurement = odometry;

        timeTaken = Timer.getFPGATimestamp();
    }
}
