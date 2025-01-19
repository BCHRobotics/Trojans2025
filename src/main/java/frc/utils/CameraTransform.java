package frc.utils;

/*
 * Info on how the camera is positioned relative to the center of the bot
 */
public class CameraTransform {
    // positional offset IN METERS from the center of the bot
    public double xOffset;
    public double yOffset;

    // angle IN RADIANS from the bot heading to the camera heading
    public double angleOffset;

    public CameraTransform(double x, double y, double theta) {
        xOffset = x;
        yOffset = y;
        angleOffset = theta;
    }
}
