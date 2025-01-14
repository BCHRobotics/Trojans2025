package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.devices.VisionUtils;

public class Cameras extends SubsystemBase {
    private PhotonCamera[] cameras;
    private PhotonPipelineResult[] results;

    private int tagCount = 16;

    // test stuff
    private Pose2d[] tagTestPositions;
    private Drivetrain driveSubsystem;
    
    public Cameras() {
        cameras = new PhotonCamera[VisionConstants.cameraNames.length];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = new PhotonCamera(VisionConstants.cameraNames[i]);
        }

        results = new PhotonPipelineResult[cameras.length];

        tagTestPositions = new Pose2d[tagCount];
    }

    public void setDriveSubsystem(Drivetrain subsystem) {
        driveSubsystem = subsystem;
    }

    @Override
    public void periodic() {
        updateCameraResults();
        printToDashboard();

        updateTagTestPositions();
    }

    public Pose2d getTagTestPosition(int tagId) {
        if (tagTestPositions[tagId] == null) {return null;}
        else {return tagTestPositions[tagId];}
    }

    public void updateTagTestPositions() {
        for (int i = 0; i < 16; i++) {
            Transform2d fieldOffset = getFieldOrientedTagOffset(i);
            if (fieldOffset == null) {continue;}
            tagTestPositions[i] = driveSubsystem.getPose().plus(fieldOffset);
        }
    }

    // public Pose2d estimateRobotPose() {
    //     Transform2d[] fieldRelativeOffsets = getAllFieldRelativeOffsets();
    // }

    // public Transform2d[] getAllFieldRelativeOffsets() {

    // }

    public void printToDashboard() {
        //SmartDashboard.putBoolean("Tag Visible", canSeeTag(4));

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

    public void updateCameraResults() {
        for (int i = 0; i < cameras.length; i++) {
            List<PhotonPipelineResult> currentResults = cameras[i].getAllUnreadResults();
            if (currentResults.size() > 0) {
                results[i] = currentResults.get(0);
            }
        }
    }

    public boolean hasTargets(int cameraIndex) {
        return results[cameraIndex].hasTargets();
    }
    
    public PhotonTrackedTarget getBestTarget(int cameraIndex) {
        return results[cameraIndex].getBestTarget();
    }

    public List<PhotonTrackedTarget> getAllTargets(int cameraIndex) {
        return results[cameraIndex].getTargets();
    }
}
