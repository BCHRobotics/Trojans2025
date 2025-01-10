package frc.utils.devices;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision{
    PhotonCamera cameraRight = new PhotonCamera("Right");
    PhotonCamera cameraLeft = new PhotonCamera("Left");
    PhotonCamera cameraCenter = new PhotonCamera("Center");

    PhotonPipelineResult rightResult = cameraRight.getLatestResult();
    PhotonPipelineResult leftResult = cameraLeft.getLatestResult();
    PhotonPipelineResult centerResult = cameraCenter.getLatestResult();

    boolean rightHasTargets = rightResult.hasTargets();
    boolean leftHasTargets = leftResult.hasTargets();
    boolean centerHasTargets = centerResult.hasTargets();

    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> rightTargets = rightResult.getTargets();
    List<PhotonTrackedTarget> leftTargets = leftResult.getTargets();
    List<PhotonTrackedTarget> centerTargets = centerResult.getTargets();

    PhotonTrackedTarget closestRightTarget = rightResult.getBestTarget();
    PhotonTrackedTarget closestLeftTarget = leftResult.getBestTarget();
    PhotonTrackedTarget closestCenterTarget = centerResult.getBestTarget();

    double rightYaw = closestRightTarget.getYaw();
    double leftYaw = closestLeftTarget.getYaw();
    
}
