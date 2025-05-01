package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * PhotonVisionPoseV2 subsystem for updating robot odometry using PhotonPoseEstimator.
 * 
 * This subsystem uses the PhotonVision library's PhotonPoseEstimator to detect AprilTags and estimate
 * the robot's position on the field. It integrates with the Drivetrain subsystem
 * to update the robot's odometry.
 */
public class Vision extends SubsystemBase {
    private final Drivetrain m_drivetrain; // Reference to the drivetrain subsystem

    public PhotonCamera m_camera; // PhotonVision camera for detecting AprilTags
    
    private double oldDist;
    private Pose2d oldPose;

    private double lastEstimate;

    private double lastDistEstimate;
    private double distEstimateInterval;

    public Vision(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        
        // Initialize the cameras using the camera names from constants
        m_camera = new PhotonCamera("Center");

        distEstimateInterval = 0.5;
    }

    /**
     * Periodic method that runs every scheduler cycle.
     * This method updates the robot's pose using PhotonPoseEstimator.
     */
    @Override
    public void periodic() {
        printToDashboard();
    }

    public double getYaw() {
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
        
        if (oldPose == null) {
            oldPose = m_drivetrain.getPose();
        }

        for (int i = 0; i < results.size(); i++) {
            if (results.get(i).getBestTarget() == null || results.get(i).getBestTarget().getDetectedObjectClassID() != 0) {continue;}

            List<TargetCorner> corners = results.get(i).getBestTarget().minAreaRectCorners;

            TargetCorner bottomLeft = corners.get(0);
            TargetCorner bottomRight = corners.get(1);
            // TargetCorner topLeft = corners.get(2);
            // TargetCorner topRight = corners.get(3);

            System.out.println(bottomRight.x - bottomLeft.x);

            // if (Timer.getFPGATimestamp() > lastDistEstimate + distEstimateInterval) {

            //     lastDistEstimate = Timer.getFPGATimestamp();

            //     List<TargetCorner> corners = results.get(i).getBestTarget().minAreaRectCorners;

            //     TargetCorner bottomLeft = corners.get(0);
            //     TargetCorner bottomRight = corners.get(1);
            //     // TargetCorner topLeft = corners.get(2);
            //     // TargetCorner topRight = corners.get(3);
                
            //     double estimatedDist = estimateDistance(oldDist, bottomRight.x - bottomLeft.x, getDistanceBetweenPoses(oldPose, m_drivetrain.getPose()));

            //     if (estimatedDist > 0.1 && estimatedDist < 10) {
            //         System.out.println(estimatedDist);
            //         lastEstimate = estimatedDist;
            //     } else {System.out.println(lastEstimate);}

            //     // update the old variables
            //     oldDist = bottomRight.x - bottomLeft.x;
            //     oldPose = m_drivetrain.getPose();
            // }

            return results.get(i).getBestTarget().yaw;
        }

        return 0;
    }

    double getDistanceBetweenPoses(Pose2d a, Pose2d b) {
        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
    }

    // estimate the distance to a vision target
    double estimateDistance(double oldScale, double newScale, double travelDist) {
        return (travelDist * (newScale / oldScale) / (1 - (newScale / oldScale))) - travelDist;
    }

    String toCorner(TargetCorner corner) {
        return "(" + corner.x + "," + corner.y + ")";
    }

    void printToDashboard() {
    }
} 