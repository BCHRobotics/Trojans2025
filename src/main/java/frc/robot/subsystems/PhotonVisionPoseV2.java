package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * PhotonVisionPoseV2 subsystem for updating robot odometry using PhotonPoseEstimator.
 * 
 * This subsystem uses the PhotonVision library's PhotonPoseEstimator to detect AprilTags and estimate
 * the robot's position on the field. It integrates with the Drivetrain subsystem
 * to update the robot's odometry.
 */
public class PhotonVisionPoseV2 extends SubsystemBase {
    private final Drivetrain m_drivetrain; // Reference to the drivetrain subsystem

    public PhotonCamera m_camera; // PhotonVision camera for detecting AprilTags
    private Transform3d m_cameraToRobot; // Transform from the robot center to the camera

    private AprilTagFieldLayout m_fieldLayout; // Layout of AprilTags on the field
    private PhotonPoseEstimator m_poseEstimator; // Estimator for calculating robot pose

    private boolean useVisionPose = true; // Flag to enable/disable vision-based updates
    
    private PhotonPipelineResult latestResult;

    private boolean isTargetingLeft;

    private double lastConnectionLog;
    private double connectionLogInterval;

    /**
     * Creates a new PhotonVisionPoseV2 subsystem.
     * 
     * @param drivetrain The drivetrain subsystem to update with vision measurements
     */
    public PhotonVisionPoseV2(Drivetrain drivetrain) {
        connectionLogInterval = 2;

        m_drivetrain = drivetrain;
        try {
            // Initialize the cameras using the camera names from constants
            m_camera = new PhotonCamera("Center");
            // Create the camera to robot transform using offsets from constants
            m_cameraToRobot = VisionConstants.cameraOffsets[0];

            m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

            // Initialize the pose estimator with the field layout, strategy, and camera transform
            m_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_cameraToRobot);

        } catch (Exception e) {
            // Report initialization error to the DriverStation
            DriverStation.reportError("Error initializing PhotonVisionV2: " + e.getMessage(), e.getStackTrace());
            m_fieldLayout = null;
        }
    }

    /**
     * Enable or disable vision-based pose updates
     * @param enabled Whether vision-based pose updates should be enabled
     */
    public void setVisionPoseEnabled(boolean enabled) {
        useVisionPose = enabled;
    }

    /**
     * Returns whether vision-based pose updates are enabled
     * @return True if vision pose updates are enabled, false otherwise
     */
    public boolean isVisionPoseEnabled() {
        return useVisionPose;
    }

    public Pose2d getTagPoseOfId(int tagId) {
        return m_fieldLayout.getTagPose(tagId).get().toPose2d();
    }

    public Pose2d getClosestTagPose() {
        if (getClosestTagID() == -1) {return null;}
        if (m_fieldLayout.getTagPose(getClosestTagID()).isEmpty()) {return null;}

        return m_fieldLayout.getTagPose(getClosestTagID()).get().toPose2d();
    }
    
    // -1 means no tag seen
    public int getClosestTagID() {
        if (latestResult == null) {return -1;}
        if (!latestResult.hasTargets()) { return -1;}

        double lowestDistance = latestResult.targets.get(0).getBestCameraToTarget().getX();
        int lowestId = latestResult.targets.get(0).fiducialId;

        for (int i = 1; i < latestResult.targets.size(); i++) {
            Transform3d offset = latestResult.targets.get(i).getBestCameraToTarget();

            if (offset.getX() < lowestDistance) {
                lowestDistance = offset.getX();
                lowestId = latestResult.targets.get(i).fiducialId;
            }
        }

        return lowestId;
    }

    /**
     * Periodic method that runs every scheduler cycle.
     * This method updates the robot's pose using PhotonPoseEstimator.
     */
    @Override
    public void periodic() {
        updatePose();

        printToDashboard();
    }

    void tryGetCamera() {
        m_camera = new PhotonCamera(NetworkTableInstance.getDefault(), "Cam");
    }

    public double getXOffset() {
        return 0.53;
    }

    public double getYOffset() {
        return isTargetingLeft ? -0.16 : 0.16;
    }

    public void targetSide(boolean isLeft) {
        isTargetingLeft = isLeft;
    }

    void printToDashboard() {
        Pose2d closestTagPose = getClosestTagPose();

        if (closestTagPose != null) {
            SmartDashboard.putNumber("tagX", closestTagPose.getX());
            SmartDashboard.putNumber("tagY", closestTagPose.getY());
            SmartDashboard.putNumber("tagRot", closestTagPose.getRotation().getDegrees());
        }

        if (Timer.getFPGATimestamp() > lastConnectionLog + connectionLogInterval) {
            lastConnectionLog = Timer.getFPGATimestamp();
            System.out.println("NetworkTables connections: " + NetworkTableInstance.getDefault().getConnections().length);
            System.out.println("Is camera connected? " + m_camera.isConnected());
        }
    }

    private void updatePose() {
        latestResult = null;

        if (!useVisionPose || m_camera == null || m_fieldLayout == null) {
            return; // Exit if any condition is not met
        }

        SmartDashboard.putNumber("cam", m_camera.isConnected() ? 1 : 0);

        // Retrieve all unread pipeline results from the camera
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

        if (results.size() > 0) {
            // save the latest result
            latestResult = results.get(results.size() - 1);
        }

        double ambiguity = Double.MAX_VALUE;
        Pose3d robotPose = null;

        for (PhotonPipelineResult result : results) {
            if (result.getBestTarget() == null) {continue;}

            ambiguity = result.getBestTarget().getPoseAmbiguity();

            if (ambiguity < 0.2) {
                
                robotPose = m_poseEstimator.update(result).get().estimatedPose;
            }
        }
        
        if (robotPose != null) {
            m_drivetrain.resetOdometry(robotPose.toPose2d());
        }
    }
} 