package frc.utils;

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
}
