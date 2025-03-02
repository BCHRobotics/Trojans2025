package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

    public PhotonCamera camera1;
    public PhotonCamera camera2;
    public PhotonCamera camera3;

    public Cameras(String cam1Name, String cam2Name, String cam3Name) {
        camera1 = new PhotonCamera(cam2Name);
        camera2 = new PhotonCamera(cam2Name);
        camera3 = new PhotonCamera(cam3Name);

        //Transform3d cam1Transform3d = new Transform3d(null, null);

        //photonEstimator = new PhotonPoseEstimator(cam1Transform3d, kTagLayout);
    }


}