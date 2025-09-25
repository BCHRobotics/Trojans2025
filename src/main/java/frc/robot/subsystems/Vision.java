package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.TargetSet;

/**
 * PhotonVisionPoseV2 subsystem for updating robot odometry using PhotonPoseEstimator.
 * 
 * This subsystem uses the PhotonVision library's PhotonPoseEstimator to detect AprilTags and estimate
 * the robot's position on the field. It integrates with the Drivetrain subsystem
 * to update the robot's odometry.
 */
public class Vision extends SubsystemBase {
    public String[] cameraNames = {
        "FrontLeft",
    };

    public PhotonCamera[] cameras;

    public List<TargetSet> cameraTargets; 

    public Vision() {
        // define all camera refs
        cameras = new PhotonCamera[cameraNames.length];
        cameraTargets = new LinkedList<TargetSet>();

        for (int i = 0; i < cameraNames.length; i++) {
            cameras[i] = new PhotonCamera(cameraNames[i]);
            cameraTargets.add(new TargetSet());
        }
    }

    @Override
    public void periodic() {
        // update the targets for each camera
        for (int i = 0; i < cameras.length; i++) {
            List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();
            if (results.size() == 0) {continue;}
            cameraTargets.get(i).setTargets(results.get(0).getTargets());
        }

        printToDashboard();
    }

    void printToDashboard() {
        // goal: print the distance of a target
        // not the euclidean distance, the projected distance (or as I call it, "floor distance")

        // only going from [0..1] because I only care about ONE OF THE front cameras
        for (int i = 0; i < 1; i++) {
            if (cameraTargets.get(i).getTargets() == null) {continue;}
            if (cameraTargets.get(i).getTargets().size() > 0) {
                // only want to measure the distance to algae (id 0)
                if (cameraTargets.get(i).getTargets().get(0).objDetectId == 0) {
                    
                    List<TargetCorner> corners = cameraTargets.get(i).getTargets().get(0).getMinAreaRectCorners();
                    

                    double pitch = (corners.get(0).y - 480 / 2) / 480 * -1 * 55;
                    double yaw = cameraTargets.get(i).getTargets().get(0).yaw;

                    // measured, meters
                    double camHeight = 0.203;

                    // the idea here is that we have a RIGHT triangle where the hypotenuse is the vector to the target,
                    // the base is the floor and the vertical side is the distance from the camera to the floor
                    // (we're looking for the base)

                    // we can use sin law here
                    
                    // sin(far angle) / vertical side
                    double ratio = Math.sin(pitch * Math.PI / 180) / camHeight;
                    double nearAngle = 180 - 90 - pitch;
                    double base = Math.sin(nearAngle * Math.PI / 180) / ratio;
                    
                    double perpDist = Math.tan(yaw * Math.PI / 180) * base;
                    
                    //for now, just log it
                    System.out.println("Distance to algae: " + base + "," + perpDist);
                }
            }
        }
    }
} 