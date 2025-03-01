package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.VisionUtils;


public class Cameras extends SubsystemBase {
    // the cameras, it's an array cuz we're gonna have like 5 and that's too many variables
    private PhotonCamera[] cameras;
    // results for each camera (this array should have the same length as above!!)
    private PhotonPipelineResult[] results; // this is for each individual camera

    //private int[] bestResultsIDs; // this is for cameras 1, 2, and 3
    //private double[] bestResultsIDsDistances; // this is the distance from each of the tag IDs to the robot

    // how many apriltags there are on the field
    // (we're using last year's family, so there are only 16)
    private int tagCount = 22;

    // test stuff
    private Drivetrain driveSubsystem;

    // IF YOU WANT TO DISABLE THE VISION CODE USE THIS!!!
    // USEFUL IF THE COPROCESSOR ISN'T PLUGGED IN!!!
    // -----
    public boolean isVisionActive = true;
    // -----

    private boolean targetingRight;
    
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

    public double getOffsetX() {
        return 0.575;
    }

    public double getOffsetY() {
        return targetingRight ? -0.165 : 0.165;
    }

    public void switchOffset(boolean isRight) {
        targetingRight = isRight;
    }

    @Override
    public void periodic() {
        if (isVisionActive) {
            // get the data from the photonvision camera(s)
            updateCameraResults();

            // print any relevant debug data to the dashboard
            printToDashboard();

            SmartDashboard.putNumber("robot heading DEG", driveSubsystem.getPose().getRotation().getDegrees());
            SmartDashboard.putNumber("tag heading DEG", VisionConstants.tagTransforms[22].headingAngle);
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

    /**
     * Gets a best-guess of where the robot is on the field, using vision
     * this is helpful to correct odometry during both auto and tele, because it can become wrong
     * 
     * @return a best guess of the robot's field relative pose, obtained from vision measurements
     */
    public Pose2d estimateRobotPoseManual(boolean distanceThreshold) {
        // avoid non-reef tags 1, 2, 13, 12, 4, 5, 14, 15

        Transform2d[] fieldRelativeOffsets = getAllFieldRelativeOffsets();
        
        // variables for keeping track of the final x, y, rot
        // we define these here because we adjust the estimated position iteratively in a loop
        // so basically these variables are just added to again and again
        float finalX = 0;
        float finalY = 0;
        float finalRot = 0;
        
        // the following two variables are to deal with angle wrapping
        // --------------------------------------------------------------

        // an estimated pose is found by averaging results from multiple tags,
        // but two measurements of -179 and 179 would average to 0 instead of 180 which it should be

        // the total number of tags that can be seen by the cameras
        int visibleTagCount = 0;
        int firstTagId = -1;

        // first we do a loop through all the tags to figure out which ones we can see, and count them up
        // necessary because each result is divided by the total number of results
        for (int i = 1; i < fieldRelativeOffsets.length; i++) {
            if (fieldRelativeOffsets[i] != null && isTagValid(i)) {
                if ((getDistanceToTag(i) > 3.0 || fieldRelativeOffsets[i].getX() > 3.0) && distanceThreshold) {continue;}

                // adding to the total tag count
                visibleTagCount++;
                if (firstTagId == -1) {
                    firstTagId = i;
                }
            }
        }

        // looping through all the results again to actually add up the measurements
        for (int i = 1; i < fieldRelativeOffsets.length; i++) {
            if (fieldRelativeOffsets[i] != null && isTagValid(i)) {
                if ((getDistanceToTag(i) > 3.0 || fieldRelativeOffsets[i].getX() > 3.0) && distanceThreshold) {continue;}

                // the idea here is to figure out where the tag is (which is static),
                // then figure out where the robot thinks it is relative to the tag,
                // then add the two vectors to guess at where the robot is on the field

                // defining positions and offsets
                // -----------------------------------

                // first define how the tag is transformed relative to the robot (in field space)
                Pose2d offset = new Pose2d(fieldRelativeOffsets[i].getX(), 
                fieldRelativeOffsets[i].getY(), 
                fieldRelativeOffsets[i].getRotation());
                
                // then define where the tag is in field space
                Pose2d tagPosition = VisionConstants.tagTransforms[i].getPosition();

                // then combine them, subtracting to get from the tag's position to where the robot thinks it is
                Transform2d estimatedPosition = new Transform2d(
                    tagPosition.getX() - offset.getX(),
                    tagPosition.getY() - offset.getY(),
                    tagPosition.getRotation().minus(offset.getRotation())
                );

                // dealing with the x and y estimate
                // -----------------

                // add the current x and y estimate to the total, 
                // dividing by the total number of measurements to eventually get an average
                finalX += estimatedPosition.getX() / visibleTagCount;
                finalY += estimatedPosition.getY() / visibleTagCount;
                
                // dealing with the rotational estimate
                // ----------------------
                
                // ONLY ONE rotation measurement is used, and it's just the first tag the robot sees
                if (finalRot == 0) {
                    finalRot += estimatedPosition.getRotation().getRadians();
                }
            }
        }

        // setting up the final pose
        // ----------------------------------------

        if (visibleTagCount == 0) {return new Pose2d(0, 0, new Rotation2d());}

        // defining the FINAL ESTIMATED POSE
        // since we already divided each of the x and y results inside of the loop, we can use the sum as-is
        // rotation, however, needs to be divided still
        Pose2d finalPose = new Pose2d(
        finalX,
        finalY,
        Rotation2d.fromRadians(finalRot)
        );

        // this is now our final pose which can be returned
        return finalPose;
    }
    
    // avoid non-reef tags 1, 2, 13, 12, 4, 5, 14, 15
    public boolean isTagValid(int id) {
        if (id != 1 && id != 2 && id != 13 && id != 12 && id != 4 && id != 5 && id != 14 && id != 15) {
            return true;
        }
        else {
            return false;
        }
    }

    // get the difference between 
    public Transform2d getPoseEstimatedOffset() {
        Pose2d estimatedPose = estimateRobotPoseManual(true);
        Pose2d odometryPose = driveSubsystem.getPose();

        return new Transform2d(
            estimatedPose.getX() - odometryPose.getX(), 
            estimatedPose.getY() - odometryPose.getY(),
            Rotation2d.fromDegrees(0));
    }

    // get the distance between where the tag SHOULD BE on the field, and where the robot thinks it is
    public double getDistanceToTag(int tagId) {
        Pose2d staticTagPose = VisionConstants.tagTransforms[tagId].getPosition();
        Pose2d robotPose = driveSubsystem.getPose();

        return Math.abs(staticTagPose.getX() - robotPose.getX());
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
        //SmartDashboard.putBoolean("Left Cam", cameras[0].isConnected());
        SmartDashboard.putBoolean("Center Cam", cameras[0].isConnected());
        //SmartDashboard.putBoolean("Right Cam", cameras[2].isConnected());

        //SmartDashboard.putNumber("x dist", VisionConstants.tagTransforms[18].xPosition - driveSubsystem.getPose().getX());

        // f is x, g is y, h is rot
        SmartDashboard.putNumber("f", estimateRobotPoseManual(false).getX());
        SmartDashboard.putNumber("g", estimateRobotPoseManual(false).getY());
        SmartDashboard.putNumber("h", estimateRobotPoseManual(false).getRotation().getDegrees());
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
        List<Integer> cameraIndices = new LinkedList<Integer>();
        List<Transform3d> rawOffsets = new LinkedList<Transform3d>();

        for (int i = 0; i < results.length; i++) {
            if (results[i]==null){continue;}
            for (int j = 0; j < results[i].getTargets().size(); j++) {
                if (results[i].getTargets().get(j).fiducialId == tagId) {
                    cameraIndices.add(i);
                    rawOffsets.add(results[i].getTargets().get(j).getBestCameraToTarget());
                }
            }
        }

        if (cameraIndices.size() == 0){ return null;}

        for (int i = 0; i < cameraIndices.size(); i++) {
            Transform3d rawOffsetWithoutCameraOffset = rawOffsets.get(i);
            // accounting for an offseted camera
            // -----------------------------
            
            //figuring out the field-relative position of the camera relative to the bot
            Transform2d robotToCamera = VisionConstants.cameraOffsets[cameraIndices.get(i)].getTransform();
            
            //subtracting that from the estimated pose to get the position of bot center
            // this is done MANUALLY because WPILib's built-in functions are terrible :(
            rawOffsets.set(i, new Transform3d(
                rawOffsetWithoutCameraOffset.getX() + robotToCamera.getX(),
                rawOffsetWithoutCameraOffset.getY() + robotToCamera.getY(),
                rawOffsetWithoutCameraOffset.getZ(),
                rawOffsetWithoutCameraOffset.getRotation()
            ));
        }

        Transform3d rawOffset = new Transform3d(0, 0, 0, new Rotation3d());

        for (int i = 0; i < cameraIndices.size(); i++) {
            rawOffset = new Transform3d(
                rawOffset.getX() + rawOffsets.get(i).getX() / cameraIndices.size(),
                rawOffset.getY() + rawOffsets.get(i).getY() / cameraIndices.size(),
                rawOffset.getZ() + rawOffsets.get(i).getZ() / cameraIndices.size(),
                rawOffsets.get(0).getRotation()
            );
        }

        Transform2d fieldRelativeOffset = VisionUtils.rawToFieldOriented(tagId, rawOffset);

        return fieldRelativeOffset;
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

    public int getBestTargetID(PhotonTrackedTarget target){
        return target.fiducialId;
    }

    /*
     * get all the targets (just like raw photonvision data) from a given camera
     */
    public List<PhotonTrackedTarget> getAllTargets(int cameraIndex) {
        return results[cameraIndex].getTargets();
    }

    /*
     * get whichever tag index is closest
     */
    public int getClosestTagId() {
        int closetId = -1;
        double closestDistance = 0;

        for (int i = 1; i <= tagCount; i++) {
            if (getRawTagOffset(i) != null) {
                if (closetId == -1 || getRawTagOffset(i).getX() < closestDistance) {
                    closetId = i;
                    closestDistance = getRawTagOffset(i).getX();
                }
            }
        }
        
        // WARNING: this will be -1 if you can't see any tags!!
        return closetId;
    }
}