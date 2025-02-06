package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.VisionUtils;

public class Cameras extends SubsystemBase {
    // the cameras, it's an array cuz we're gonna have like 5 and that's too many variables
    private PhotonCamera[] cameras;
    // results for each camera (this array should have the same length as above!!)
    private PhotonPipelineResult[] results;

    // how many apriltags there are on the field
    // (we're using last year's family, so there are only 16)
    private int tagCount = 22;

    // test stuff
    private Drivetrain driveSubsystem;

    // IF YOU WANT TO DISABLE THE VISION CODE USE THIS!!!
    // USEFUL IF THE COPROCESSOR ISN'T PLUGGED IN!!!
    // -----
    public boolean isVisionActive = true;
    public boolean periodicPoseEstimation = true;
    // -----

    private double lastPoseEstimate = 0;
    private double estimateFreqency = 0.5;
    
    public Cameras() {

        // initialize and fill any necessary arrays
        cameras = new PhotonCamera[VisionConstants.cameraNames.length];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.cameraNames[i]);
        }

        results = new PhotonPipelineResult[cameras.length];
    }

    // since the cameras need to pull (odometry) data from the drivetrain,
    // we need a reference to the drivetrain
    public void setDriveSubsystem(Drivetrain subsystem) {
        driveSubsystem = subsystem;
    }

    @Override
    public void periodic() {
        if (isVisionActive) {
            // get the data from the photonvision camera(s)
            updateCameraResults();

            // print any relevant debug data to the dashboard
            //printToDashboard();

            if (periodicPoseEstimation && Timer.getFPGATimestamp() > lastPoseEstimate + estimateFreqency) {
                updateOdometry();
                lastPoseEstimate = Timer.getFPGATimestamp();
            }
        }
    }

    /*
     * apply the estimated pose to the robot odometry
     */
    public void updateOdometry() {
        if (canSeeAnyTags()) {
            Pose2d visionPose = estimateRobotPoseManual();
            Pose2d currentPose = driveSubsystem.getPose();
            
            if (currentPose.getX() < 0.1) {return;}

            Transform2d offset = visionPose.minus(currentPose);

            if (Math.abs(visionPose.getX() - currentPose.getX()) > 0.05 || 
            Math.abs(visionPose.getY() - currentPose.getY()) > 0.05) {
                driveSubsystem.setOdometryOffset(offset);
            }
        }
    }
    
    /*
     * whether the camera can see any tags at all
     * (check all the tags and see if one is visible)
     */
    public boolean canSeeAnyTags() {
        if (!isVisionActive) {return false;}

        for (int i = 1; i < tagCount; i++) {
            if (canSeeTag(i)) {
                return true;
            }
        }

        return false;
    }

    /*
     * silly function
     */
    public Pose2d estimateRobotPoseManual() {
        Transform2d[] fieldRelativeOffsets = getAllFieldRelativeOffsets();
        
        float finalX = 0;
        float finalY = 0;
        float finalRot = 0;
        
        int balance = 0;
        int largeAngles = 0;

        int tagCount = 0;

        for (int i = 1; i < fieldRelativeOffsets.length; i++) {
            if (fieldRelativeOffsets[i] != null) {
                tagCount++;
            }
        }

        for (int i = 1; i < fieldRelativeOffsets.length; i++) {
            if (fieldRelativeOffsets[i] != null) {
                Pose2d offset = new Pose2d(fieldRelativeOffsets[i].getX(), 
                fieldRelativeOffsets[i].getY(), 
                fieldRelativeOffsets[i].getRotation());

                Pose2d tagPosition = VisionConstants.tagTransforms[i].getPosition();

                Transform2d estimatedPosition = new Transform2d(
                    tagPosition.getX() - offset.getX(),
                    tagPosition.getY() - offset.getY(),
                    tagPosition.getRotation().minus(offset.getRotation())
                );

                finalX += estimatedPosition.getX() / tagCount;
                finalY += estimatedPosition.getY() / tagCount;
                
                if (estimatedPosition.getRotation().getRadians() >= 0) {
                    balance++;
                    finalRot += estimatedPosition.getRotation().getRadians();
                }
                else {
                    balance--;
                    finalRot += estimatedPosition.getRotation().getRadians();
                }

                if (Math.abs(estimatedPosition.getRotation().getRadians()) > Math.PI / 2) {
                    largeAngles++;
                }
            }
        }

        if (Math.abs(balance) < tagCount) {
            // we have a mix of positive and negative numbers
            if (largeAngles > tagCount / 2) {
                finalRot += Math.PI;
            }
        } 

        Pose2d finalPose = new Pose2d(
        finalX,
        finalY,
        Rotation2d.fromRadians(finalRot/tagCount)
        );
        
        //figuring out the field-relative position of the camera relative to the bot
        Transform2d robotToCamera = VisionConstants.cameraOffsets[0].getTransform();
        Translation2d fieldRelativeRobotToCamera = VisionUtils.applyRotationMatrix(robotToCamera.getTranslation(), finalPose.getRotation().getRadians());
        
        //subtracting that from the estimated pose to get the position of bot center
        finalPose = finalPose.plus(new Transform2d(fieldRelativeRobotToCamera, new Rotation2d()));
        return finalPose;
    }

    /*
     * get every single field relative offsets for the tags on the field,
     * used for pose estimation
     */
    public Transform2d[] getAllFieldRelativeOffsets() {
        Transform2d[] toReturn = new Transform2d[tagCount+1];

        for (int i = 1; i <= tagCount; i++) {
            if (getFieldOrientedTagOffset(i) != null) {
                toReturn[i] = getFieldOrientedTagOffset(i);
            }
        }
        return toReturn;
    }

    /*
     * printing debug stuff to the dashboard
     */
    public void printToDashboard() {
    }

    /*
     * whether or not the camera can see a tag with a specific id
     */
    public boolean canSeeTag(int tagId) {
        if (!isVisionActive) {return false;}
        
        for (int i = 0; i < results.length; i++) {
            for (int j = 0; j < results[i].getTargets().size(); j++) {
                if (results[i].getTargets().get(j).fiducialId == tagId) {
                    return true;
                }
            }
        }

        return false;
    }

    /*
     * get the field oriented offset for a tag with a specific id
     */
    public Transform2d getFieldOrientedTagOffset(int tagId) {
        Transform3d rawOffset = null;
        int cameraIndex = -1;

        for (int i = 0; i < results.length; i++) {
            if (results[i]==null){continue;}
            for (int j = 0; j < results[i].getTargets().size(); j++) {
                if (results[i].getTargets().get(j).fiducialId == tagId) {
                    rawOffset = results[i].getTargets().get(j).getBestCameraToTarget();
                    
                    cameraIndex = i;
                }
            }
        }

        if (cameraIndex == -1){ return null;}

        return VisionUtils.rawToFieldOriented(tagId, rawOffset, VisionConstants.cameraOffsets[cameraIndex]);
    }

    /*
     * get the raw 3d vector (literally just the data that photonvision spits out)
     */
    public Transform3d getRawTagOffset(int tagId) {
        for (int i = 0; i < results.length; i++) {
            for (int j = 0; j < results[i].getTargets().size(); j++) {
                if (results[i].getTargets().get(j).fiducialId == tagId) {
                    return results[i].getTargets().get(j).getBestCameraToTarget();
                }
            }
        }

        return null;
    }
    
    /*
     * update all the camera data from photonvision
     */
    public void updateCameraResults() {
        for (int i = 0; i < cameras.length; i++) {
            List<PhotonPipelineResult> currentResults = cameras[i].getAllUnreadResults();

            if (currentResults.size() > 0) {
                results[i] = currentResults.get(0);
            }
        }
    }

    /*
     * check if a given camera has any targets
     */
    public boolean hasTargets(int cameraIndex) {
        return results[cameraIndex].hasTargets();
    }

    /*
     * get the best target for a given camera index
     * not super useful, usually you are looking for a specific tag index
     */
    public PhotonTrackedTarget getBestTarget(int cameraIndex) {
        return results[cameraIndex].getBestTarget();
    }

    /*
     * get all the targets (just like raw photonvision data) from a given camera
     */
    public List<PhotonTrackedTarget> getAllTargets(int cameraIndex) {
        return results[cameraIndex].getTargets();
    }
}
