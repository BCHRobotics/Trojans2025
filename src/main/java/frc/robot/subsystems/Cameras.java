package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.VisionUtils;

import java.util.Arrays;


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
    public boolean periodicPoseEstimation = false;
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
            printToDashboard();

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

    /**
     * Gets a best-guess of where the robot is on the field, using vision
     * this is helpful to correct odometry during both auto and tele, because it can become wrong
     * 
     * @return a best guess of the robot's field relative pose, obtained from vision measurements
     */
    public Pose2d estimateRobotPoseManual() {
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

        // this variable specifically keeps track of how many negative and positive angles there are,
        // every time we add a result we add 1 if its a positive angle and subtract 1 if its a negative angle
        int balance = 0;

        // this one keeps track of large angles, greater than 90 degrees
        // this is because two readings of -2 and 2 SHOULD average to zero, but -100 and 100 should average to 180
        int largeAngles = 0;

        // the total number of tags that can be seen by the cameras
        int tagCount = 0;

        // first we do a loop through all the tags to figure out which ones we can see, and count them up
        // necessary because each result is divided by the total number of results
        for (int i = 1; i < fieldRelativeOffsets.length; i++) {
            if (fieldRelativeOffsets[i] != null) {
                // adding to the total tag count
                tagCount++;
            }
        }

        // looping through all the results again to actually add up the measurements
        for (int i = 1; i < fieldRelativeOffsets.length; i++) {
            if (fieldRelativeOffsets[i] != null) {
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
                finalX += estimatedPosition.getX() / tagCount;
                finalY += estimatedPosition.getY() / tagCount;
                
                // dealing with the rotational estimate
                // ----------------------

                if (estimatedPosition.getRotation().getRadians() >= 0) {
                    // if the angle is greater than 0 degrees, adjust the balance value by 1
                    // (see above as to why)
                    balance++;
                }
                else {
                    // if the angle is less than 0 degrees, adjust the balance value by -1
                    // (see above as to why)
                    balance--;
                }
                
                // no matter if the angle is greater or less than 0, 
                // we add it to the total AND DO NOT DIVIDE IT,
                // the dividing is done at the end for rotation
                finalRot += estimatedPosition.getRotation().getRadians();
                
                // incrementing the large angles variable if the angle is large (> 90 degrees)
                // as said above we need to keep track of this to make sure the rotation averages properly
                if (Math.abs(estimatedPosition.getRotation().getRadians()) > Math.PI / 2) {
                    largeAngles++;
                }
            }
        }

        // setting up the final pose
        // ----------------------------------------

        // if the balance is NOT either equal to the tag count or the tag count multiplied by -1,
        // then we have a mix of positive and negative angles
        if (Math.abs(balance) < tagCount) {
            // if we have a mix of angles AND the majority (more than half) are large (> 90 degrees),
            // then we should add 180 degrees (pi radians) so that the averaged value actually reflects the measurements
            
            // I cannot think of a scenario where this would give the wrong result, but it is possible
            // so I guess TODO: stress-test this

            if (largeAngles > tagCount / 2) {
                finalRot += Math.PI;
            }
        } 

        // defining the FINAL ESTIMATED POSE
        // since we already divided each of the x and y results inside of the loop, we can use the sum as-is
        // rotation, however, needs to be divided still
        Pose2d finalPose = new Pose2d(
        finalX,
        finalY,
        Rotation2d.fromRadians(finalRot/tagCount)
        );

        // this is now our final pose which can be returned
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
        SmartDashboard.putBoolean("Left Cam", cameras[0].isConnected());
        SmartDashboard.putBoolean("Right Cam", cameras[1].isConnected());

        SmartDashboard.putNumber("x dist", VisionConstants.tagTransforms[18].xPosition - driveSubsystem.getPose().getX());
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

        // TODO: if multiple cameras see the tag, average the results

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

        Transform2d fieldRelativeOffset = VisionUtils.rawToFieldOriented(tagId, rawOffset);

        SmartDashboard.putNumber("offseted x", fieldRelativeOffset.getX());

        // accounting for an offseted camera
        // -----------------------------

        SmartDashboard.putNumber("robot heading", driveSubsystem.getHeading() / 180 * Math.PI);
        
        //figuring out the field-relative position of the camera relative to the bot
        Transform2d robotToCamera = VisionConstants.cameraOffsets[0].getTransform();
        Translation2d fieldRelativeRobotToCamera = VisionUtils.applyRotationMatrix(robotToCamera.getTranslation(), driveSubsystem.getHeading() / 180 * Math.PI);

        SmartDashboard.putNumber("offseted cam", fieldRelativeRobotToCamera.getX());
        
        //subtracting that from the estimated pose to get the position of bot center
        // this is done MANUALLY because WPILib's built-in functions are terrible :(
        fieldRelativeOffset = new Transform2d(
            fieldRelativeOffset.getX() + fieldRelativeRobotToCamera.getX(),
            fieldRelativeOffset.getY() + fieldRelativeRobotToCamera.getY(),
            fieldRelativeOffset.getRotation()
        );

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

    // public int getBestAprilTag(){
    //     for(int i=0; i<bestResultsIDs.length; i++){
    //         bestResultsIDsDistances[i] = PhotonUtils
    //                                         .getDistanceToPose(
    //                                             driveSubsystem.getPose(),
    //                                             VisionConstants.tagTransforms[i].getPosition());    
    //     }
    //     double smallestDistance = Arrays.stream(bestResultsIDsDistances).min().getAsDouble();
    //     List<double[]> tempList = Arrays.asList(bestResultsIDsDistances);

        
    //     return bestResultsIDs[tempList.indexOf(smallestDistance)];
    // }

}