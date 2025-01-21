package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

/*
 * This script is for helper functions related to vision and related math
 * There should only be public static functions in here
 */
public class VisionUtils {
    /**
     * Since FRC provides tag position in inches, this function corrects them to meters
     * @param inputTransform
     * @return The corrected tag transform (meters instead of inches)
     */
    public static TagTransform correctTagUnits(TagTransform inputTransform) {
        return new TagTransform(
            Units.inchesToMeters(inputTransform.xPosition), 
            Units.inchesToMeters(inputTransform.yPosition), 
            inputTransform.zPosition, 
            inputTransform.headingAngle, 
            inputTransform.elevationAngle);
    }

    /**
     * A function that, given a transform2d as a vector, rotates it by an angle
     * TODO: make this use a translation2d because the rotation component just isn't used
     * @param inputMatrix The input vector
     * @param angle Angle to rotate by IN RADIANS
     * @return the vector rotated by the angle
     */
    public static Transform2d applyRotationMatrix(Transform2d inputMatrix, double angle) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(angle);
        double cosHeading = Math.cos(angle);

        // Create field-relative coordinates using the heading and robot-relative coords
        double fieldX = inputMatrix.getX() * cosHeading + inputMatrix.getY() * -sinHeading;
        double fieldY = inputMatrix.getX() * sinHeading + inputMatrix.getY() * cosHeading;

        // Create the transform2d object
        Transform2d rotatedVector = new Transform2d(fieldX, fieldY, inputMatrix.getRotation());

        return rotatedVector;
    }

    /**
     * Takes a Transform3d and turns it into a transform2d by getting rid of y and x rotation,
     * as well as the z position (height)
     * @param rawOffset 3d position
     * @return The position projected onto the xy plane
     */
    public static Transform2d projectOntoHorizontalPlane(Transform3d rawOffset) {
        return new Transform2d(new Translation2d(rawOffset.getX(), rawOffset.getY()), Rotation2d.fromRadians(rawOffset.getRotation().getZ()));
    }

    /*
    * This function maybe shouldn't exist
    */
    public static Transform2d correctRotation(Transform2d inputOffset, double cameraHeading) {
        return new Transform2d(inputOffset.getX(), inputOffset.getY(), inputOffset.getRotation().plus(Rotation2d.fromRadians(cameraHeading)));
    }

    /**
     * Figure out the heading of the camera based on what tag it sees and what the offset is from that tag
     * @param tagId The index of the tag seen by the camera
     * @param tagHeadingOffset The tag's reported rotational offset from the camera IN RADIANS
     * @return A rotation2d class with the camera's heading
     */
    public static Rotation2d constructCameraHeading(int tagId, double tagHeadingOffset) {
        Rotation2d tagFieldHeading = Rotation2d.fromDegrees(VisionConstants.tagTransforms[tagId-1].headingAngle);

        return tagFieldHeading.minus(Rotation2d.fromRadians(tagHeadingOffset));
    }

    /**
     * Construct a field oriented offset from the tag based on the recorded local offset
     * @param tagId The index of the tag seen
     * @param rawOffset The local 3d offset from the tag, as reported by the camera
     * @param camTransform The transform data of the camera
     * @return The 2D field oriented offset from the tag
     */
    public static Transform2d rawToFieldOriented(int tagId, Transform3d rawOffset, CameraTransform camTransform) {
        // first, we figure out where the camera is facing based on the static tag heading and the offset
        // this allows us to get a robot heading without the gyro, and is important for pose estimation
        double cameraHeading = constructCameraHeading(tagId, rawOffset.getRotation().getZ()).getRadians();

        // just getting rid of the z (height) component of the raw offset and the x and y rotation (z heading stays)
        Transform2d projectedOffset = projectOntoHorizontalPlane(rawOffset);
        // apply the rotation matrix to switch from robot-relative to field relative
        Transform2d offsetWithRotationMatrix = applyRotationMatrix(projectedOffset, cameraHeading);

        // not sure if this is necessary, either way this variable isn't being used rn
        Transform2d fieldOrientedOffset = correctRotation(offsetWithRotationMatrix, cameraHeading);

        // return the transformed vector
        return fieldOrientedOffset;
    }
}
