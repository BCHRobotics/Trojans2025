package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraMode;

// A class with a bunch of assorted vision functions
// Everything from rotation matrix stuff to vision profiles to gamepiece tracking
// TODO: condense these functions and make them better
public class VisionUtils {

  //_____________ Apriltag Management Functions ____________//

  // Tracking and storing apriltag positions in a modular way
  // This replaces the old ampTargetPose/speakerTargetPose system
  public static int apriltagCount = 16;
  public static Pose2d[] tagPositions = new Pose2d[apriltagCount];
  public static int[] flaggedTags = new int[] {5,6,4,7};

  // Grab the position of a tag using a camera mode
  public static Pose2d getPose(CameraMode _mode, boolean _isRed) {
    return tagPositions[_mode.getIndex(_isRed)];
  }

  // Get the cameraMode associated with a tag using that tag's index
  public static CameraMode getModeFromTagIndex(int _tagIndex, boolean _isRed) {
    for (int i = 0; i < CameraMode.values().length; i++) {
      if (_tagIndex == CameraMode.values()[i].getIndex(_isRed)) {
        return CameraMode.values()[i];
      }
    }

    return null;
  }

  // Get the heading of a tag using that tag's index
  public static double getHeadingFromTagIndex(int _tagIndex, boolean _isRed) {
      for (int i = 0; i < CameraMode.values().length; i++) {
        if (_tagIndex == CameraMode.values()[i].getIndex(_isRed)) {
          return CameraMode.values()[i].getHeading(_isRed);
        }
      }

      return 0;
  }

    //_____________ Coordinate Conversion Functions ____________//

  /**
   * A function that converts the supplied Transform2d in robot relative coordinates 
   * into a Transform2d in field relative coordinates 
   * (flips the output because the camera is on the back).
   * @param objectTransform The transform of the object
   * @param heading The heading of the robot (degrees)
   * @return the transform of the object in field coordinates
   */
  public static Transform2d robotToField(Transform2d objectTransform, double heading) {
      // Multiply the heading by PI/180 to convert to radians
      double sinHeading = Math.sin(heading * (Math.PI / 180));
      double cosHeading = Math.cos(heading * (Math.PI / 180));

      // Create field-relative coordinates using the heading and robot-relative coords
      double fieldX = objectTransform.getX() * cosHeading + objectTransform.getY() * -sinHeading;
      double fieldY = objectTransform.getX() * sinHeading + objectTransform.getY() * cosHeading;

      // Create the transform2d object
      Transform2d fieldTransform = new Transform2d(-fieldX, -fieldY, objectTransform.getRotation());

      return fieldTransform;
  }

  /**
   * A function that converts the supplied Transform2d in tag relative coordinates
   * into a Transform2d in field relative coordinates (this is different from robotToField 
   * because it doesn't flip the output).
   * @param objectTransform The transform of the object
   * @param heading The heading of the robot (degrees)
   * @return the transform of the object in field coordinates
   */
  public static Transform2d tagToField(Transform2d objectTransform, double heading) {
    // Multiply the heading by PI/180 to convert to radians
    double sinHeading = Math.sin(heading * (Math.PI / 180));
    double cosHeading = Math.cos(heading * (Math.PI / 180));

    // Create field-relative coordinates using the heading and tag-relative coords
    double fieldX = objectTransform.getX() * cosHeading + objectTransform.getY() * -sinHeading;
    double fieldY = objectTransform.getX() * sinHeading + objectTransform.getY() * cosHeading;

    // Create the transform2d object
    Transform2d fieldTransform = new Transform2d(fieldX, fieldY, objectTransform.getRotation());

    return fieldTransform;
  }

  //_____________ Trajectory Prediction ____________//

  /**
   * @param vi The initial, scalar, velocity of the note (m/s)
   * @param h The height of the shooter (meters)
   * @param shootAngle The vertical angle of the shooter (radians)
   * @param robotHeading The horizontal angle of the shooter, usually robotHeading (deg)
   * @param targetPose The 2d field-relative position of the target, which isn't what you think it is (meters x and y)
   * @param robotPose The pose of the robot (meters x and y)
   * @return
   */
  public static boolean isReadyToShoot(double vi, double h, double shootAngle, double robotHeading, Pose2d targetPose, Pose2d robotPose, double xv, double yv) {
    // Shift the robot pose based on velocity
    robotPose = robotPose.plus(new Transform2d(xv * 0.5, yv * 0.5, new Rotation2d()));

    // Separating the initial velocity into up and forward components
    double initialVelocityScalar = vi;
    double initialVelocityUp = initialVelocityScalar * Math.sin(shootAngle);
    double initialVelocityForward = initialVelocityScalar * Math.cos(shootAngle);

    // Apply a rotation matrix to the velocity using the robot heading to split it into x and y
    Transform2d initialShootVelocity = robotToField(new Transform2d(initialVelocityForward, 0, new Rotation2d()), robotHeading);
    Transform2d initialVelocityFieldRelative = new Transform2d(initialShootVelocity.getX() + xv, initialShootVelocity.getY() + yv, new Rotation2d());

    // acceleration due to gravity
    double g = 9.81;

    // Height of the target (speaker)
    // double th = 2.1;
    double th = 2.1;

    // a values for the parabolas
    double ax = g / (2 * Math.pow(Math.sqrt(Math.pow(initialVelocityFieldRelative.getX(), 2) + Math.pow(initialVelocityUp, 2)), 2) * Math.pow(Math.cos(Math.atan(initialVelocityUp / initialVelocityFieldRelative.getX())), 2));
    double ay = g / (2 * Math.pow(Math.sqrt(Math.pow(initialVelocityFieldRelative.getY(), 2) + Math.pow(initialVelocityUp, 2)), 2) * Math.pow(Math.cos(Math.atan(initialVelocityUp / initialVelocityFieldRelative.getY())), 2));

    // discriminants
    double discriminantX = Math.sqrt(Math.pow(initialVelocityUp / initialVelocityFieldRelative.getX(), 2) - 4 * -ax * (h - th));
    double discriminantY = Math.sqrt(Math.pow(initialVelocityUp / initialVelocityFieldRelative.getY(), 2) - 4 * -ay * (h - th));

    // Where is the speaker (bounds in meters) (relative to the robot)
    // Only works for red alliance rn
    double minX = targetPose.getX() - robotPose.getX() - 0.35;
    double maxX = targetPose.getX() - robotPose.getX() - 0.1;
    double minY = targetPose.getY() - robotPose.getY() - 0.5;
    double maxY = targetPose.getY() - robotPose.getY() + 0.5;

    // Where is (0, 0) on the floor (relative to the robot)
    // double minX = -robotPose.getX() - 0.2;
    // double maxX = -robotPose.getX() + 0.2;
    // double minY = -robotPose.getY() - 0.2;
    // double maxY = -robotPose.getY() + 0.2;

    // check if there are possible solutions
    // Recall for parabolas:
    // <0 means no solutions (success)
    // 0 means one solution (fail)
    // >0 means two solutions (success)
    if (discriminantX > 0 || discriminantY > 0) {
      double xval1 = (initialVelocityUp / initialVelocityFieldRelative.getX() * -1 + discriminantX) / (2 * -ax);
      double xval2 = (initialVelocityUp / initialVelocityFieldRelative.getX() * -1 - discriminantX) / (2 * -ax);

      double yval1 = (initialVelocityUp / initialVelocityFieldRelative.getY() * -1 + discriminantY) / (2 * -ay);
      double yval2 = (initialVelocityUp / initialVelocityFieldRelative.getY() * -1 - discriminantY) / (2 * -ay);

      double xval;
      double yval;

      // all this logic to take the closest to 0 out of x and y values
      if (initialVelocityFieldRelative.getX() > 0) {
        xval = xval1 > xval2 ? xval2 : xval1;
      }
      else {
        xval = xval1 > xval2 ? xval1 : xval2;
      }

      if (initialVelocityFieldRelative.getY() > 0) {
        yval = yval1 > yval2 ? yval2 : yval1;
      }
      else {
        yval = yval1 > yval2 ? yval1 : yval2;
      }

      if (xval < maxX && xval > minX && yval < maxY && yval > minY) {
        return true;
      }
      else {
        SmartDashboard.putNumber("X1", robotPose.getX() + xval1);
        SmartDashboard.putNumber("Y1", robotPose.getY() + yval1);
        SmartDashboard.putNumber("X2", robotPose.getX() + xval2);
        SmartDashboard.putNumber("Y2", robotPose.getY() + yval2);
        return false;
      }
    }
    else { // At least one discriminant fails
      SmartDashboard.putNumber("Shooting Status X", -1);
      SmartDashboard.putNumber("Shooting Status Y", -1);
      return false;
    }

  }

  //____________ Utility Functions ____________//

  /**
   * @param inputValue The value that will be capped
   * @param cap The number the value should be capped to (+/-)
   * @return
   */
  public static double capValue(double inputValue, double cap) {
    return (inputValue < 0) ? 
    Math.max(inputValue, -cap) :
    Math.min(inputValue, cap);
  }

  //_____________ Vision Profiles ____________//

  // an all-in-one function for generating vision commands (UNFINISHED)
  // maybe change the targetpose param to an int for tag id? + add visionprofile class
  public static Transform2d alignWithApriltag(Pose2d targetPose, Pose2d robotPose, Transform2d offset) {
    return null;
  }

  // no idea if this works yet
  public static Transform2d alignWithTagTangent(Pose2d targetPose, Pose2d robotPose, double desiredDistance) {
    if (targetPose != null) {

      Transform2d robotToTag = targetPose.minus(robotPose);
      Transform2d robotToTagRelative = tagToField(robotToTag, -targetPose.getRotation().getDegrees());

      Transform2d command = tagToField(new Transform2d(robotToTagRelative.getX(), 0, new Rotation2d()), targetPose.getRotation().getDegrees());

      double xCommand = command.getX();
      double yCommand = command.getY();

      // Align the robot's rotation with the tag heading (face the same direction as the tag)
      Rotation2d tagRotation = Rotation2d.fromDegrees(targetPose.getRotation().getDegrees());
      Rotation2d robotRotation = robotPose.getRotation();

      double rotCommand = tagRotation.minus(robotRotation).getDegrees();

      // -- cap the values -- //

      // x axis command
      xCommand = capValue(rotCommand, VisionConstants.kVisionSpeedLimit);
      // y axis command
      yCommand = capValue(rotCommand, VisionConstants.kVisionSpeedLimit);
      // Rotational command
      rotCommand = capValue(rotCommand, VisionConstants.kVisionTurningLimit);

      // Set the x command to full speed if far enough
      if (Math.abs(xCommand) > VisionConstants.kTagSlowdownDistance) {
        xCommand = (xCommand < 0) ? 
        -VisionConstants.kVisionSpeedLimit : 
        VisionConstants.kVisionSpeedLimit;
      }

      // Set the y command to full speed if far enough
      if (Math.abs(yCommand) > VisionConstants.kTagSlowdownDistance) {
        yCommand = (yCommand < 0) ? 
        -VisionConstants.kVisionSpeedLimit : 
        VisionConstants.kVisionSpeedLimit;
      }

      // Check if robot is within acceptable boundaries
      boolean rotFinished = Math.abs(tagRotation.minus(robotRotation).getDegrees()) < VisionConstants.kTagRotationThreshold;
      boolean xFinished = Math.abs(command.getX()) < VisionConstants.kTagDistanceThreshold;
      boolean yFinished = Math.abs(command.getY()) < VisionConstants.kTagDistanceThreshold;

      if (rotFinished) { rotCommand = 0; }
      if (xFinished) { xCommand = 0; }
      // Do not zero the y command because that should be as exact as possible

      if (rotFinished && xFinished && yFinished) {
        return null;
      }
      else {
        // TODO: Maybe get rid of the * 0.3?
        return new Transform2d(xCommand, yCommand, Rotation2d.fromDegrees(rotCommand * 0.3));
      }
    }
    else {
      return null;
    }
  }

  /**
   * A function for calculating a the movement towards a given apriltag's 
   * EXACT POSITION plus an offset
   * @param targetPose The (field relative) pose of the apriltag
   * @param robotPose The (field relative) pose of the robot
   * @param desiredOffset The desired x and y offset from the apriltag
   * @return
   */
  public static Transform2d alignWithTagExact(Pose2d targetPose, Pose2d robotPose, Transform2d desiredOffset) {
    if (targetPose != null) {
      // Calculate the x and y commands based on the direction to the tag
      double xCommand = targetPose.getX() + desiredOffset.getX() - robotPose.getX();
      double yCommand = targetPose.getY() + desiredOffset.getY() - robotPose.getY();

      // Get the heading of the tag based on the camera mode (red/blue)
      Rotation2d tagRotation = targetPose.getRotation();
      // Robot heading
      Rotation2d robotRotation = robotPose.getRotation();
      // Commanded rotation
      double rotCommand = tagRotation.minus(robotRotation).getDegrees() * (double)0.2;

      // -- cap the values -- //

      // x axis command
      xCommand = capValue(rotCommand, VisionConstants.kVisionSpeedLimit);
      // y axis command
      yCommand = capValue(rotCommand, VisionConstants.kVisionSpeedLimit);
      // Rotational command
      rotCommand = capValue(rotCommand, VisionConstants.kVisionTurningLimit);

      // Set the x command to full speed if far enough
      if (Math.abs(xCommand) > VisionConstants.kTagSlowdownDistance) {
          xCommand = (xCommand < 0) ? 
          -VisionConstants.kVisionSpeedLimit : 
          VisionConstants.kVisionSpeedLimit;
      }

      // Set the y command to full speed if far enough
      if (Math.abs(yCommand) > VisionConstants.kTagSlowdownDistance) {
          yCommand = (yCommand < 0) ? 
          -VisionConstants.kVisionSpeedLimit : 
          VisionConstants.kVisionSpeedLimit;
      }

      // Check if robot is within acceptable boundaries
      boolean rotFinished = Math.abs(tagRotation.minus(robotRotation).getDegrees()) < VisionConstants.kTagRotationThreshold;
      boolean xFinished = Math.abs(targetPose.getX() + desiredOffset.getX() - robotPose.getX()) < VisionConstants.kTagDistanceThreshold;
      boolean yFinished = Math.abs(targetPose.getY() + desiredOffset.getY() - robotPose.getY()) < VisionConstants.kTagDistanceThreshold;

      if (rotFinished) { rotCommand = 0; }
      if (xFinished) { xCommand = 0; }
      // Do not zero the y command because that should be as exact as possible

      if (rotFinished && xFinished && yFinished) {
        return null;
      }
      else {
        return new Transform2d(xCommand, yCommand, Rotation2d.fromDegrees(rotCommand * (double)0.3));
      }
    }
    else {
      return null;
    }
  }

  /**
   * A function for calculating a the movement towards a 
   * CIRCLE with a given apriltag at it's center
   * @param targetPose The (field relative) pose of the apriltag
   * @param robotPose The (field relative) pose of the robot
   * @param desiredRadius The desired distance to the target apriltag
   * @return
   */
  public static Transform2d alignWithTagRadial(Pose2d targetPose, Pose2d robotPose, double desiredRadius) {
    // Apriltag alignment code
    if (targetPose != null) {
      double distToTag = robotPose.getTranslation().getDistance(targetPose.getTranslation());
      
      // Calculate the x and y commands, based on whether the robot should travel away or towards the tag
      // this is an absolute mess of ternary operators
      double xCommand = (distToTag > desiredRadius) ? (targetPose.getX() - robotPose.getX() < 0 ? -1 : 1) * Math.abs(distToTag - desiredRadius) * 2 : (targetPose.getX() - robotPose.getX() < 0 ? -1 : 1) * -2 * Math.abs(distToTag - desiredRadius);
      double yCommand = (distToTag > desiredRadius) ? (targetPose.getY() - robotPose.getY() < 0 ? -1 : 1) * Math.abs(distToTag - desiredRadius) * 2 : (targetPose.getY() - robotPose.getY() < 0 ? -1 : 1) * -2 * Math.abs(distToTag - desiredRadius);
        
      Rotation2d errorRot = Rotation2d.fromRadians(-Math.atan2(targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY())).minus(Rotation2d.fromDegrees(90));
      Rotation2d command = errorRot.minus(robotPose.getRotation());
      // Commanded rotation based on the tag angle
      double rotCommand = command.getDegrees() * 0.2;

      // -- cap the values -- //

      // x axis command
      xCommand = capValue(rotCommand, VisionConstants.kVisionSpeedLimit);
      // y axis command
      yCommand = capValue(rotCommand, VisionConstants.kVisionSpeedLimit);
      // Rotational command
      rotCommand = capValue(rotCommand, VisionConstants.kVisionTurningLimit);

      // Set the x command to full speed if far enough
      if (Math.abs(xCommand) > VisionConstants.kTagSlowdownDistance) {
          xCommand = (xCommand < 0) ? 
          -VisionConstants.kVisionSpeedLimit : 
          VisionConstants.kVisionSpeedLimit;
      }

      // Set the y command to full speed if far enough
      if (Math.abs(yCommand) > VisionConstants.kTagSlowdownDistance) {
          yCommand = (yCommand < 0) ? 
          -VisionConstants.kVisionSpeedLimit : 
          VisionConstants.kVisionSpeedLimit;
      }

      // Check if robot is within acceptable boundaries
      boolean rotFinished = Math.abs(command.getDegrees()) < VisionConstants.kTagRotationThreshold;
      boolean posFinished = Math.abs(distToTag - desiredRadius) < VisionConstants.kTagDistanceThreshold;

      if (rotFinished) {rotCommand = 0;}
      if (posFinished) {xCommand = 0; yCommand = 0;}

      if (rotFinished && posFinished) {
        return null;
      }

      // TODO: Maybe get rid of the * 0.3?
      return new Transform2d(xCommand, yCommand, Rotation2d.fromDegrees(rotCommand * 0.3));
    }
    else {
      return null;
    }
  }
}
