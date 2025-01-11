package frc.utils.devices;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;



public class PhotonVision{
    static PhotonCamera cameraRight = new PhotonCamera("Right");
    static PhotonCamera cameraLeft = new PhotonCamera("Left");
    static PhotonCamera cameraCenter = new PhotonCamera("Center");
            
    static PhotonPipelineResult rightResult = cameraRight.getLatestResult();
    static PhotonPipelineResult leftResult = cameraLeft.getLatestResult();
    static PhotonPipelineResult centerResult = cameraCenter.getLatestResult();
                 
    boolean rightHasTargets = rightResult.hasTargets();
    boolean leftHasTargets = leftResult.hasTargets();
    boolean centerHasTargets = centerResult.hasTargets();
                 
    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> rightTargets = rightResult.getTargets();
    List<PhotonTrackedTarget> leftTargets = leftResult.getTargets();
    List<PhotonTrackedTarget> centerTargets = centerResult.getTargets();
                 
    static PhotonTrackedTarget closestRightTarget = rightResult.getBestTarget();
    static PhotonTrackedTarget closestLeftTarget = leftResult.getBestTarget();
    static PhotonTrackedTarget closestCenterTarget = centerResult.getBestTarget();
                 
    public static double rightYaw = closestRightTarget.getYaw();
    public static double leftYaw = closestLeftTarget.getYaw();
    public static double centerYaw = closestCenterTarget.getYaw();
    
    

}
