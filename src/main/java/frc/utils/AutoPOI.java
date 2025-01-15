package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;

/*
 * point of interest for use in autos
 */
public class AutoPOI {
    public Pose2d position;
    public String name;

    // this constructor should not really be used unless creating the class and filling it out separately
    // use the below one to avoid null errors
    public AutoPOI() {
    }

    public AutoPOI(Pose2d pose, String name) {
        position = pose;
        this.name = name;
    }
}
