package frc.utils.devices;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import frc.utils.CameraTransform;

public class VisionUtils {
    /**
     * A function that converts the supplied Transform2d in robot relative coordinates 
     * into a Transform2d in field relative coordinates 
     * (flips the output because the camera is on the back).
     * @param objectTransform The transform of the object
     * @param heading The heading of the robot (radians)
     * @return the transform of the object in field coordinates
     */
    public static Transform2d applyRotationMatrix(Transform2d inputMatrix, double heading) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(heading);
        double cosHeading = Math.cos(heading);

        // Create field-relative coordinates using the heading and robot-relative coords
        double fieldX = inputMatrix.getX() * cosHeading + inputMatrix.getY() * -sinHeading;
        double fieldY = inputMatrix.getX() * sinHeading + inputMatrix.getY() * cosHeading;

        // Create the transform2d object
        Transform2d fieldTransform = new Transform2d(fieldX, fieldY, inputMatrix.getRotation());

        return fieldTransform;
    }

    public static Transform2d projectOntoHorizontalPlane(Transform3d rawOffset) {
        return new Transform2d(new Translation2d(rawOffset.getX(), rawOffset.getY()), Rotation2d.fromRadians(rawOffset.getRotation().getZ()));
    }

    public static Transform2d correctRotation(Transform2d inputOffset, double cameraHeading) {
        return new Transform2d(inputOffset.getX(), inputOffset.getY(), inputOffset.getRotation().plus(Rotation2d.fromRadians(cameraHeading)));
    }

    public static Rotation2d constructCameraHeading(int tagId, double tagHeadingOffset) {
        Rotation2d tagFieldHeading = Rotation2d.fromDegrees(VisionConstants.tagHeadings[tagId-1]);

        return tagFieldHeading.minus(Rotation2d.fromRadians(tagHeadingOffset));
    }

    public static Transform2d rawToFieldOriented(int tagId, Transform3d rawOffset, CameraTransform camTransform) {
        double cameraHeading = constructCameraHeading(tagId, rawOffset.getRotation().getZ()).getRadians();

        Transform2d projectedOffset = projectOntoHorizontalPlane(rawOffset);
        Transform2d offsetWithRotationMatrix = applyRotationMatrix(projectedOffset, -cameraHeading);

        Transform2d fieldOrientedOffset = correctRotation(offsetWithRotationMatrix, cameraHeading);

        return fieldOrientedOffset;
    }
}
