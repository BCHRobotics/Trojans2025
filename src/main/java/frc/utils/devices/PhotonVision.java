package frc.utils.devices;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;


public class PhotonVision{
    PhotonCamera cameraRight = new PhotonCamera("Right");
    static PhotonCamera cameraLeft = new PhotonCamera("Left");
        PhotonCamera cameraCenter = new PhotonCamera("Center");
        
        PhotonPipelineResult rightResult = cameraRight.getLatestResult();
         static PhotonPipelineResult leftResult = cameraLeft.getLatestResult();
         PhotonPipelineResult centerResult = cameraCenter.getLatestResult();
         
         boolean rightHasTargets = rightResult.hasTargets();
         boolean leftHasTargets = leftResult.hasTargets();
         boolean centerHasTargets = centerResult.hasTargets();
         
             // Get a list of currently tracked targets.
         List<PhotonTrackedTarget> rightTargets = rightResult.getTargets();
         List<PhotonTrackedTarget> leftTargets = leftResult.getTargets();
         List<PhotonTrackedTarget> centerTargets = centerResult.getTargets();
         
         PhotonTrackedTarget closestRightTarget = rightResult.getBestTarget();
         static PhotonTrackedTarget closestLeftTarget = leftResult.getBestTarget();
        PhotonTrackedTarget closestCenterTarget = centerResult.getBestTarget();
        
        double rightYaw = closestRightTarget.getYaw();
        public static double leftYaw = closestLeftTarget.getYaw();

    
    

}
