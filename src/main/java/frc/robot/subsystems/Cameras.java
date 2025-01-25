package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private PhotonPipelineResult[] results;

    // how many apriltags there are on the field
    // (we're using last year's family, so there are only 16)
    private int tagCount = 16;

    // test stuff
    private Pose2d[] tagTestPositions;
    private Drivetrain driveSubsystem;

    // IF YOU WANT TO DISABLE THE VISION CODE USE THIS!!!
    // USEFUL IF THE COPROCESSOR ISN'T PLUGGED IN!!!
    // -----
    private boolean isVisionActive = false;
    private boolean useTagTestPositions = true;
    // ----
    
    public Cameras() {
        cameras = new PhotonCamera[VisionConstants.cameraNames.length];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.cameraNames[i]);
        }

        results = new PhotonPipelineResult[cameras.length];

        tagTestPositions = new Pose2d[tagCount+1];
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
            
            // update the "test positions" for the tags, basically reverse-engineering tag position from offset data and odometry
            // this allows us to align to tags without a defined field layout
            // NOTE - we do need to have predefined tag headings though
            updateTagTestPositions();
        }
    }

    /*
     * retrieve one of the tag test positions from the array
     * see periodic for more info
     */
    public Pose2d getTagTestPosition(int tagId) {
        if (tagTestPositions[tagId] == null) {return null;}
        else {return tagTestPositions[tagId];}
    }

    /*
     * update the "test positions"
     * see periodic for more info
     */
    public void updateTagTestPositions() {
        for (int i = 0; i < tagCount; i++) {
            Transform2d fieldOffset = getFieldOrientedTagOffset(i);
            if (fieldOffset == null) {continue;}
            tagTestPositions[i] = new Pose2d(
                driveSubsystem.getPose().getX() + fieldOffset.getX(),
                driveSubsystem.getPose().getY() + fieldOffset.getY(),
                fieldOffset.getRotation()
            );
        }
    }
    
    /*
     * whether the camera can see any tags at all
     * (check all the tags and see if one is visible)
     */
    public boolean canSeeAnyTags() {
        for (int i = 0; i < tagCount; i++) {
            if (canSeeTag(i)) {
                return true;
            }
        }

        return false;
    }

    /*
     * WARNING: THIS IS AN UNTESTED FUNCTION
     * figure out where the robot is based on camera data
     */
    public Pose2d estimateRobotPose() {
        Transform2d[] fieldRelativeOffsets = getAllFieldRelativeOffsets();
        Pose2d finalPose = new Pose2d(0, 0, new Rotation2d());

        for (int i = 0; i < fieldRelativeOffsets.length; i++) {
            finalPose.plus(tagTestPositions[i].minus(new Pose2d(fieldRelativeOffsets[i].getX(), fieldRelativeOffsets[i].getY(), new Rotation2d())));
        }
        finalPose.div(fieldRelativeOffsets.length);

        return finalPose;
    }

    /*
     * get every single field relative offsets for the tags on the field,
     * used for pose estimation
     */
    public Transform2d[] getAllFieldRelativeOffsets() {
        ArrayList<Transform2d> toReturn = new ArrayList<Transform2d>();

        for (int i = 0; i < tagCount; i++) {
            if (getFieldOrientedTagOffset(i) != null) {
                toReturn.add(getFieldOrientedTagOffset(i));
            }
        }

        return toReturn.toArray(new Transform2d[toReturn.size()]);
    }

    /*
     * printing debug stuff to the dashboard
     */
    public void printToDashboard() {
        //SmartDashboard.putBoolean("Tag Visible", canSeeTag(4));

        // SmartDashboard.putNumber("odometry x", driveSubsystem.getPose().getX());
        // SmartDashboard.putNumber("odometry y", driveSubsystem.getPose().getY());

        // SmartDashboard.putNumber("camera x", estimateRobotPose().getX());
        // SmartDashboard.putNumber("camera y", estimateRobotPose().getY());

        SmartDashboard.putNumber("odometry x", driveSubsystem.getPose().getX());
        SmartDashboard.putNumber("odometry y", driveSubsystem.getPose().getY());
        SmartDashboard.putNumber("odometry rot", driveSubsystem.getHeading());

        if (canSeeTag(1) && tagTestPositions[1] != null) {
            logTag(1);
        }

        // if (getTagTestPosition(4) != null) {
        //     SmartDashboard.putNumber("Tag X", getTagTestPosition(4).getX());
        //     SmartDashboard.putNumber("Tag Y", getTagTestPosition(4).getY());

        //     if (getFieldOrientedTagOffset(4) != null) {
        //         SmartDashboard.putNumber("Offset X", getFieldOrientedTagOffset(4).getX());
        //         SmartDashboard.putNumber("Offset Y", getFieldOrientedTagOffset(4).getY());
        //     }

        //     SmartDashboard.putNumber("Pose X", driveSubsystem.getPose().getX());
        //     SmartDashboard.putNumber("Pose Y", driveSubsystem.getPose().getY());
        // }
    }

    public void logTag(int index) {
        SmartDashboard.putNumber("tag x", tagTestPositions[index].getX());
        SmartDashboard.putNumber("tag y",tagTestPositions[index].getY());
        SmartDashboard.putNumber("tag rot",tagTestPositions[index].getRotation().getDegrees());
    }

    /*
     * whether or not the camera can see a tag with a specific id
     */
    public boolean canSeeTag(int tagId) {
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
