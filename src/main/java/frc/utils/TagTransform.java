package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/*
 * Info on how a tag is positioned on the field
 */
public class TagTransform {
    public double headingAngle;
    public double elevationAngle;

    public double xPosition;
    public double yPosition;
    public double zPosition;

    public TagTransform(double xPos, double yPos, double zPos, double zRot, double yRot) {
        headingAngle = zRot;
        elevationAngle = yRot;

        xPosition = xPos;
        yPosition = yPos;
        zPosition = zPos;
    }

    /*
     * returns the 2D position of a tag, field-relative
     */
    public Pose2d getPosition() {
        return new Pose2d(xPosition, yPosition, Rotation2d.fromDegrees(headingAngle));
    }
}
