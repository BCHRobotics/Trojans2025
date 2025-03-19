package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    public PhotonCamera m_cameraMain; // PhotonVision camera for detecting AprilTags
    public PhotonCamera m_cameraSecondary; // PhotonVision camera for detecting AprilTags
    private Transform3d m_cameraMainToRobot; // Transform from the robot center to the camera
    private Transform3d m_cameraSecondaryToRobot; // Transform from the robot center to the camera
    private AprilTagFieldLayout m_fieldLayout; // Layout of AprilTags on the field
    private PhotonPoseEstimator m_poseEstimator; // Estimator for calculating robot pose
    private PhotonPoseEstimator m_poseEstimatorSecondary; // Estimator for calculating robot pose
    private boolean m_visionPoseEnabled = true; // Flag to enable/disable vision-based updates
    private final Field2d field2d = new Field2d();
    

    /**
     * Creates a new PhotonVisionPoseV2 subsystem.
     * 
     * @param drivetrain The drivetrain subsystem to update with vision measurements
     */
    public PhotonVisionPoseV2(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        try {
            // Initialize the cameras using the camera names from constants
            m_cameraMain = new PhotonCamera(VisionConstants.cameraNames[0]);
            // m_cameraSecondary = new PhotonCamera(VisionConstants.cameraNames[1]); // Uncomment for secondary camera
            // Create the camera to robot transform using offsets from constants
            m_cameraMainToRobot = new Transform3d(
                new Translation3d(VisionConstants.cameraOffsets[0].xOffset, VisionConstants.cameraOffsets[0].yOffset,0 ), // Units.inchesToMeters(10)
                new Rotation3d(0, 0, VisionConstants.cameraOffsets[0].angleOffset));

            m_cameraSecondaryToRobot = new Transform3d(
                new Translation3d(VisionConstants.cameraOffsets[0].xOffset, VisionConstants.cameraOffsets[0].yOffset, 0.0),
                new Rotation3d(0, 0, VisionConstants.cameraOffsets[0].angleOffset));
            // Load the default field layout for AprilTags
            m_fieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
            // Initialize the pose estimator with the field layout, strategy, and camera transform
            m_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_cameraMainToRobot);
            m_poseEstimatorSecondary = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_cameraMainToRobot);
            // Indicate successful initialization on the SmartDashboard
            SmartDashboard.putBoolean("PhotonVisionV2 Initialized", true);
        } catch (Exception e) {
            // Report initialization error to the DriverStation and SmartDashboard
            DriverStation.reportError("Error initializing PhotonVisionV2: " + e.getMessage(), e.getStackTrace());
            SmartDashboard.putBoolean("PhotonVisionV2 Initialized", false);
            m_fieldLayout = null; // Set field layout to null on error
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
     * Returns whether vision-based pose updates are enabled
     * @return True if vision pose updates are enabled, false otherwise
     */
    public boolean isVisionPoseEnabled() {
        return m_visionPoseEnabled;
    }

    /**
     * Periodic method that runs every scheduler cycle.
     * This method updates the robot's pose using PhotonPoseEstimator.
     */
    @Override
    public void periodic() {
        updatePose();
    }

    public Supplier<Pose2d> estimatedPose(){

            Supplier<Pose2d> poseSupplier = this::updatePose;

            return poseSupplier;
    }

    /**
     * 
     * @param newPose 
     * @param currentPose
     * @return
     * This is a helper function for smoothing the position. We don't know how well this works since it hasn't been tested yet
     */
    ///WARNING!!! ANGLE WRAPPING IS NOT IMPLEMENTED IN THIS FUNCTION- ndykstra
    /// 
    private Pose2d smoothPose(Pose2d newPose, Pose2d currentPose) {
    double alpha = 0.7; // 0.0 = trust old pose, 1.0 = trust new pose
    double smoothedX = alpha * newPose.getX() + (1 - alpha) * currentPose.getX();
    double smoothedY = alpha * newPose.getY() + (1 - alpha) * currentPose.getY();
    double smoothedTheta = alpha * newPose.getRotation().getRadians() + 
                           (1 - alpha) * currentPose.getRotation().getRadians();

    return new Pose2d(smoothedX, smoothedY, new Rotation2d(smoothedTheta));
}
    public Command generatePathToPose2d(Pose2d targetPose){
        

        // Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation
    

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            1.0, 2.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                
            return pathfindingCommand;
    }

    /**
     * Updates the robot's pose using PhotonPoseEstimator.
     * This method processes all unread results from the camera, estimates the robot's pose
     * using the PhotonPoseEstimator, and updates the drivetrain's odometry if a valid pose is found.
     */
    private Pose2d updatePose() {
        // Check if vision updates are enabled and if the camera and field layout are initialized
        if (!m_visionPoseEnabled || m_cameraMain == null || m_fieldLayout == null) {
            return null; // Exit if any condition is not met
        }

        // Retrieve all unread pipeline results from the camera
        List<PhotonPipelineResult> results = m_cameraMain.getAllUnreadResults();

        // List<PhotonPipelineResult> secondaryResults = m_cameraSecondary.getAllUnreadResults();

        double ambiguity = Double.MAX_VALUE;
        double ambiguitySecondary;

        for (PhotonPipelineResult result : results) {
            // Update the pose estimator with the current result and get the estimated pose
            Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update(result);

            if (estimatedPose.isPresent()) {
                // Convert the estimated pose to Pose2d and update the drivetrain's odometry
                Pose2d robotPose = estimatedPose.get().estimatedPose.toPose2d();

                double ambiguityToPrint = result.getBestTarget().getPoseAmbiguity();
                 
                // Reject poses with high ambiguity
                if (ambiguityToPrint > 0.2) {
                    continue; // Skip this measurement
                }
                    

                m_drivetrain.resetOdometry(robotPose);
                // Display the estimated pose on the SmartDashboard
                SmartDashboard.putNumber("Estimated X", robotPose.getX());
                SmartDashboard.putNumber("Estimated Y", robotPose.getY());
                SmartDashboard.putNumber("Estimated Rotation", robotPose.getRotation().getDegrees());
                field2d.setRobotPose(robotPose);
                SmartDashboard.putNumber("Tag Seen", result.getBestTarget().getFiducialId());
                SmartDashboard.putData("Field",field2d);
                SmartDashboard.putNumber("Ambiguity",ambiguityToPrint);
                //System.out.println(result.getBestTarget().getFiducialId());
                return new Pose2d(robotPose.getX(),robotPose.getY(),robotPose.getRotation());
            }
             
        }
        /* 

        for (PhotonPipelineResult result : secondaryResults) {
            // Update the pose estimator with the current result and get the estimated pose
            ambiguitySecondary = result.getBestTarget().getPoseAmbiguity();
            // Reject poses with high ambiguity
            if (ambiguitySecondary > 0.2) {
                continue; // Skip this measurement
            }
            Optional<EstimatedRobotPose> estimatedPoseSecondary = m_poseEstimator.update(result);
            Pose2d robotPose = estimatedPoseSecondary.get().estimatedPose.toPose2d();
            if (estimatedPoseSecondary.isPresent()) {
                if (ambiguitySecondary > ambiguity && result.getBestTarget().getPoseAmbiguity()<0.2){
                    m_drivetrain.resetOdometry(robotPose);
                }
            }
            
            
        }
            */
        return null;
    }
} 