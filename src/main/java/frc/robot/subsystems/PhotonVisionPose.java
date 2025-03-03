// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.List;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.CameraTransform;
import frc.utils.TagTransform;

/**
 * PhotonVisionPose subsystem for updating robot odometry using AprilTags.
 * 
 * This subsystem uses the PhotonVision library to detect AprilTags and estimate
 * the robot's position on the field. It integrates with the Drivetrain subsystem
 * to update the robot's odometry.
 * 
 * While PhotonVision does provide a built-in pose estimation library (PhotonPoseEstimator),
 * this implementation uses a simplified approach that gives us more control over
 * the pose estimation process.
 */
public class PhotonVisionPose extends SubsystemBase {
  private final Drivetrain m_drivetrain;
  
  // PhotonVision camera
  private PhotonCamera m_camera;
  
  // Camera to robot transform
  private Transform3d m_cameraToRobot;
  
  // Field layout for AprilTags
  private AprilTagFieldLayout m_fieldLayout;
  
  // Timestamp of the last pose update
  private double m_lastEstTimestamp = 0;
  
  // Flag to enable/disable vision-based updates
  private boolean m_visionPoseEnabled = true;
  
  // Maximum distance to trust the vision measurement (in meters)
  private final double MAX_VISION_DISTANCE = 4.0;
  
  // Maximum ambiguity value to trust the vision measurement (0-1)
  private final double MAX_AMBIGUITY = 0.5;

  /** 
   * Creates a new PhotonVisionPose subsystem.
   * 
   * @param drivetrain The drivetrain subsystem to update with vision measurements
   */
  public PhotonVisionPose(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    
    try {
      // Initialize the camera with the first camera name from constants
      m_camera = new PhotonCamera(VisionConstants.cameraNames[0]);
      
      // Create the camera to robot transform (assuming a "Center" camera position)
      CameraTransform camTransform = VisionConstants.cameraOffsets[0];
      m_cameraToRobot = new Transform3d(
          new Translation3d(camTransform.xOffset, camTransform.yOffset, 0.0),
          new Rotation3d(0, 0, camTransform.angleOffset));
      
      // Create the AprilTag field layout - using the field layout
      //REQUIRES WPILIB 2025.3.1
      m_fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
      

      SmartDashboard.putBoolean("PhotonVision Initialized", true);
    } catch (Exception e) {
      DriverStation.reportError("Error initializing PhotonVision: " + e.getMessage(), e.getStackTrace());
      SmartDashboard.putBoolean("PhotonVision Initialized", false);
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
   * Gets all unread PhotonPipelineResults from the camera
   * @return Collection of unread pipeline results
   */
  public List<PhotonPipelineResult> getAllUnreadPipelineResults() {
    return m_camera.getAllUnreadResults();
  }

  /**
   * Finds the nearest AprilTag across all unread pipeline results
   * @return The nearest tracked target, null if no targets
   */
  public PhotonTrackedTarget getNearestTag() {
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
    
    PhotonTrackedTarget nearestTarget = null;
    double nearestDistance = Double.MAX_VALUE;
    
    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) {
        continue;
      }
      
      for (PhotonTrackedTarget target : result.getTargets()) {
        Transform3d targetTransform = target.getBestCameraToTarget();
        double distance = Math.sqrt(
            Math.pow(targetTransform.getX(), 2) + 
            Math.pow(targetTransform.getY(), 2));
        
        if (distance < nearestDistance) {
          nearestDistance = distance;
          nearestTarget = target;
        }
      }
    }
    
    return nearestTarget;
  }
  
  /**
   * Checks if a tag is within the maximum vision distance
   * @param target The tracked target to check
   * @return True if the target is within the max distance, false otherwise
   */
  private boolean isTagWithinRange(PhotonTrackedTarget target) {
    Transform3d targetTransform = target.getBestCameraToTarget();
    double distance = Math.sqrt(
        Math.pow(targetTransform.getX(), 2) + 
        Math.pow(targetTransform.getY(), 2));
        
    return distance <= MAX_VISION_DISTANCE;
  }

  /**
   * Updates the robot's pose using a detected AprilTag.
   * 
   * This method leverages the known positions of AprilTags on the field from the
   * field layout to calculate the robot's position. The calculation is as follows:
   * 
   * 1. Get the target's position on the field from the field layout
   * 2. Use the camera's measurement of the target to calculate camera position
   * 3. Apply the camera-to-robot transform to get robot position
   * 
   * @param target The AprilTag target detected by the camera
   * @return true if pose was updated, false otherwise
   */
  private boolean updatePoseWithTag(PhotonTrackedTarget target) {
    // Skip if field layout wasn't initialized properly
    if (m_fieldLayout == null) {
      return false;
    }
    
    // Skip if target is invalid
    if (target == null) {
      return false;
    }
    
    int tagId = target.getFiducialId();
    
    // Check the tag's ambiguity (confidence level)
    double ambiguity = target.getPoseAmbiguity();
    if (ambiguity > MAX_AMBIGUITY) {
      return false;
    }
    
    // Try to get the tag pose from the field layout
    Optional<Pose3d> tagPose = m_fieldLayout.getTagPose(tagId);
    if (!tagPose.isPresent()) {
      return false;
    }
    
    // Get the camera-to-target transform
    Transform3d camToTarget = target.getBestCameraToTarget();
    
    // Calculate camera pose in field coordinates
    // Camera pose = Tag pose ⊖ Camera-to-Target transform
    Pose3d cameraPose = tagPose.get().transformBy(camToTarget.inverse());
    
    // Transform from camera to robot center
    // Robot pose = Camera pose ⊖ Camera-to-Robot transform
    Pose3d robotPose = cameraPose.transformBy(m_cameraToRobot.inverse());
    
    // Update the robot's odometry with the new pose
    m_drivetrain.resetOdometry(robotPose.toPose2d());
    
    // Log diagnostic information
    SmartDashboard.putNumber("Vision Tag ID", tagId);
    SmartDashboard.putNumber("Vision Ambiguity", ambiguity);
    SmartDashboard.putNumber("Estimated X", robotPose.getX());
    SmartDashboard.putNumber("Estimated Y", robotPose.getY());
    SmartDashboard.putNumber("Estimated Rotation", 
        robotPose.getRotation().toRotation2d().getDegrees());
    
    return true;
  }

  /**
   * Main periodic loop for the vision subsystem.
   * 
   * This method is called approximately every 20ms (50 times per second).
   * It orchestrates the process of:
   * 1. Getting new camera data
   * 2. Finding the best AprilTag to use
   * 3. Calculating robot pose based on that tag
   * 4. Updating the odometry system
   * 
   * Vision updates are conditionally applied only when:
   * - Vision updates are enabled
   * - The camera is properly initialized
   * - New frames are available with visible tags
   * - Tags have low ambiguity (high confidence)
   */
  @Override
  public void periodic() {
    // Skip processing if vision is disabled or camera failed to initialize
    if (!m_visionPoseEnabled || m_camera == null || m_fieldLayout == null) {
      return;
    }

    // Get all unread pipeline results
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
    
    // Only proceed if we have new camera frames to process
    if (!results.isEmpty()) {
      // Find the most recent timestamp among all results
      double latestTimestamp = 0;
      for (PhotonPipelineResult result : results) {
        if (result.getTimestampSeconds() > latestTimestamp) {
          latestTimestamp = result.getTimestampSeconds();
        }
      }
      
      // Only update if we have a new timestamp to avoid redundant processing
      if (latestTimestamp != m_lastEstTimestamp) {
        m_lastEstTimestamp = latestTimestamp;
        
        // Get the nearest AprilTag from all unread frames
        PhotonTrackedTarget nearestTag = getNearestTag();
        
        // Update pose estimation using the nearest tag if it's valid and close enough
        if (nearestTag != null && isTagWithinRange(nearestTag)) {
          updatePoseWithTag(nearestTag);
        }
      }
    }
    
    // Update dashboard with debug information
    updateDashboard();
  }
  
  /**
   * Updates the SmartDashboard with debug information
   */
  private void updateDashboard() {
    PhotonTrackedTarget nearestTag = getNearestTag();
    
    if (nearestTag != null) {
      SmartDashboard.putNumber("Nearest Tag ID", nearestTag.getFiducialId());
      SmartDashboard.putNumber("Tag Distance", 
          Math.sqrt(
              Math.pow(nearestTag.getBestCameraToTarget().getX(), 2) + 
              Math.pow(nearestTag.getBestCameraToTarget().getY(), 2)));
      SmartDashboard.putNumber("Tag Ambiguity", nearestTag.getPoseAmbiguity());
    } else {
      SmartDashboard.putNumber("Nearest Tag ID", -1);
    }
    
    SmartDashboard.putBoolean("Vision Has Target", nearestTag != null);
  }
}
