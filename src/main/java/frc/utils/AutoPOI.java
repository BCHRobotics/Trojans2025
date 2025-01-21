package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;

/*
 * point of interest for use in autos
 */
public class AutoPOI {
    public int tagId; // the tag to align with (starts at 1, not an array index!!)

    // if tagId = -1, the field position and rotation
    // if tagId is defined (>= 0) then it represents the offset from the tag,
    // where all coordinates are from the tag's perspective (x is away, etc.)
    public Pose2d position; 
    
    // the name of the POI
    public String name;

    // this constructor should not really be used unless creating the class and filling it out separately
    // use the below one to avoid null errors
    public AutoPOI() {
    }

    public AutoPOI(Pose2d position, String name, int tagId) {
        this.position = position;
        this.name = name;
        this.tagId = tagId;
    }
}
