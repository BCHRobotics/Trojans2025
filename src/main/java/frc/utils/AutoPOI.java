package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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
}
