package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

/*
 * This script is for helper functions related to vision and related math
 * There should only be public static functions in here
 */
public class VisionUtils {
    /**
     * A function that, given a transform2d as a vector, rotates it by an angle
     * TODO: make this use a translation2d because the rotation component just isn't used
     * @param inputMatrix The input vector
     * @param angle Angle to rotate by IN RADIANS
     * @return the vector rotated by the angle
     */
    public static Translation2d applyRotationMatrix(Translation2d inputMatrix, double angle) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(angle);
        double cosHeading = Math.cos(angle);

        // Create field-relative coordinates using the heading and robot-relative coords
        double fieldX = inputMatrix.getX() * cosHeading + inputMatrix.getY() * -sinHeading;
        double fieldY = inputMatrix.getX() * sinHeading + inputMatrix.getY() * cosHeading;

        // Create the transform2d object
        Translation2d rotatedVector = new Translation2d(fieldX, fieldY);

        return rotatedVector;
    }
}