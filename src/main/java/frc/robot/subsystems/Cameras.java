package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Cameras extends SubsystemBase{


    private static final double camPitch = Units.degreesToRadians(30.0);
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));
    //private final PhotonPoseEstimator photonEstimator;

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


    public PhotonCamera camera1;
    public PhotonCamera camera2;
    public PhotonCamera camera3;

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, kRobotToCam);

    public Cameras(String cam1Name, String cam2Name, String cam3Name) {
        camera1 = new PhotonCamera(cam2Name);
        camera2 = new PhotonCamera(cam2Name);
        camera3 = new PhotonCamera(cam3Name);
    }

    public void updateCameraPose() {
        var result = camera1.getLatestResult();
        if (result.getBestTarget().getPoseAmbiguity() < 0.2) {
            Transform3d fieldToCamera = result.getBestTarget().getBestCameraToTarget();
        }
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        var result = camera1.getLatestResult();
        return photonPoseEstimator.update(result);
    }


}