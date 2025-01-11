package frc.utils;

public class CameraTransform {
    public double xOffset;
    public double yOffset;

    public double angleOffset;

    public CameraTransform(double x, double y, double theta) {
        xOffset = x;
        yOffset = y;
        angleOffset = theta;
    }
}
