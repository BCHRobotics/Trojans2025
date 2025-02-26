package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;

/*
 * point of interest for use in autos
 */
public class AutoPOI {
    public int tagId; // the tag to align with (starts at 1, not an array index!!)

    // the field position associated with the POI
    public Pose2d position; 

    // an offset vector that dedfines how the robot should be lined up relative to the tag
    // NOTE - this is only used if the POI involves vision, like for the reef,
    // (tagId != -1)
    public Translation2d desiredTagOffset;
    
    // the name of the POI
    public String name;

    // this constructor should not really be used unless creating the class and filling it out separately
    // use the below one to avoid null errors
    public AutoPOI() {
    }

    // initializing a POI with vision
    public AutoPOI(Pose2d position, String name, int tagId, Translation2d tagOffset) {
        this.position = position;
        this.name = name;
        this.tagId = tagId;
        this.desiredTagOffset = tagOffset;
    }

    // initializing a POI without vision
    public AutoPOI(Pose2d position, String name) {
        this.position = position;
        this.name = name;
        tagId = -1;
    }

    // static function for creating a an AutoPOI BASED OFF OF A TAG, 
    // basically you pass in an offset and it sets up the field relative position for you
    public static AutoPOI createPOIFromTag(String name, int tagId, Translation2d tagOffset) {

        // since the provided offset is local, 
        // we pass it through a rotation matrix to make it field relative
        Translation2d fieldRelativeOffset = VisionUtils.applyRotationMatrix(
            tagOffset, VisionConstants.tagTransforms[tagId].headingAngle / 180 * Math.PI);

        // grab the field relative position of the tag
        Pose2d tagFieldPose = VisionConstants.tagTransforms[tagId].getPosition();

        // we define the field relative position of the POI by just adding that offset to the tag position
        Pose2d fieldPoseWithOffset = new Pose2d(
            tagFieldPose.getX() + fieldRelativeOffset.getX(),
            tagFieldPose.getY() + fieldRelativeOffset.getY(),
            tagFieldPose.getRotation().plus(Rotation2d.fromDegrees(180))
        );

        // return the position, with the other data provided as an AutoPOI
        return new AutoPOI(
            fieldPoseWithOffset,
            name,
            tagId,
            tagOffset
        );
    }
}