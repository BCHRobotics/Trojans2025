package frc.utils;

import java.util.Optional;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * PhotonVisionPoseV2 utility class for updating robot odometry using PhotonPoseEstimator.
 * 
 * This class uses the PhotonVision library's PhotonPoseEstimator to detect AprilTags and estimate
 * the robot's position on the field. It integrates with the Drivetrain subsystem
 * to update the robot's odometry.
 */
public class PhotonVisionPoseV2 {
    private final Drivetrain m_drivetrain;
    private PhotonCamera m_camera;
    private Transform3d m_cameraToRobot;
    private AprilTagFieldLayout m_fieldLayout;
    private PhotonPoseEstimator m_poseEstimator;
    private boolean m_visionPoseEnabled = true;

    /**
     * Creates a new PhotonVisionPoseV2 utility.
     * 
     * @param drivetrain The drivetrain subsystem to update with vision measurements
     */
    public PhotonVisionPoseV2(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        try {
            m_camera = new PhotonCamera(VisionConstants.cameraNames[0]);
            m_cameraToRobot = new Transform3d(
                new Translation3d(VisionConstants.cameraOffsets[0].xOffset, VisionConstants.cameraOffsets[0].yOffset, 0.0),
                new Rotation3d(0, 0, VisionConstants.cameraOffsets[0].angleOffset));
            m_fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            m_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_cameraToRobot);
            SmartDashboard.putBoolean("PhotonVisionV2 Initialized", true);
        } catch (Exception e) {
            DriverStation.reportError("Error initializing PhotonVisionV2: " + e.getMessage(), e.getStackTrace());
            SmartDashboard.putBoolean("PhotonVisionV2 Initialized", false);
            m_fieldLayout = null;
        }
    }

    /**
     * Enable or disable vision-based pose updates
     * @param enabled Whether vision-based pose updates should be enabled
     */
    public void setVisionPoseEnabled(boolean enabled) {
        m_visionPoseEnabled = enabled;
        SmartDashboard.putBoolean("Vision Pose Enabled", enabled);
    }

    /**
     * Updates the robot's pose using PhotonPoseEstimator.
     */
    public void updatePose() {
        if (!m_visionPoseEnabled || m_camera == null || m_fieldLayout == null) {
            return;
        }

        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update(result);
            if (estimatedPose.isPresent()) {
                Pose2d robotPose = estimatedPose.get().estimatedPose.toPose2d();
                m_drivetrain.resetOdometry(robotPose);
                SmartDashboard.putNumber("Estimated X", robotPose.getX());
                SmartDashboard.putNumber("Estimated Y", robotPose.getY());
                SmartDashboard.putNumber("Estimated Rotation", robotPose.getRotation().getDegrees());
            }
        }
    }
} 