// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.utils.AutoPOI;
import frc.utils.CameraTransform;
import frc.utils.TagTransform;
import frc.utils.VisionUtils;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class ElevatorConstants {

    // possible positions for the elevator,
    // an enum is a bit nicer than separate variables
    public enum ElevatorPosition { // rotations
      L1(0),
      L2(7),
      L3(15),
      L4(20),
      INTAKE(15);
      
      
      // ------
      private final double setpoint;
      ElevatorPosition(double setpoint) {
          this.setpoint = setpoint;
      }
  
      public double getSetpoint() {
          return setpoint;
      }
    }

    // elevator CAN IDs
    public static final int leftElevatorID = 21;
    public static final int rightElevatorID = 20;

    // PID terms for elevator up/down motion
    public static final double ElevatorkP = 0.065;
    public static final double ElevatorkI = 0.001;
    public static final double ElevatorkD = 0.000;

    // supplied current limit
    public static final double maxOutput = 0.5;
    // speed limit
    public static final double maxVelocity = 300;
    
    // not used yet
    public static final double chainDiameter = 1.751; //inch
    public static final double gearRatio = 5; //5:1 gear ratio
    //defining the constant for encoder counts per inch 
    public static final double countsPerInch = (gearRatio)/(Math.PI*chainDiameter);
  }

  public static final class PhotoElectricSensorConstants {
    public static final int kPhotoElectricSensorPort = 0;
  }

  public static final class HarpoonConstants {

    public enum HarpoonPosition {
      L1(100), //degrees
      L2(120),
      L3(120),
      L4(130),
      INTAKE(55);
    
      // ------
      private final double setpoint;
      HarpoonPosition(double setpoint) {
          this.setpoint = setpoint;
      }
  
      public double getSetpoint() {
          return setpoint;
      }
    }

    public static final int kIntakeMotorCANID = 30; // TO be changed to the actual intake motor ID
    public static final int kRotationMotorCANID = 31; // TO be changed to the actual rotation motor ID

    public static final double harpoonP = 0.1;
    public static final double harpoonI = 0;
    public static final double harpoonD = 0;

    public static final double gearConversionFactor = 9; // This is the conversion factor from degrees to rotations

    public static final double scoreL1Angle = 100; // to be changed to actual value
    public static final double scoreL2Angle = 130;
    public static final double scoreL3Angle = 130;
    public static final double scoreL4Angle = 130;
  }
  
  public static final class ClimberConstants{
    public static final int kLeftCanID = 40;
    public static final int kRightCanID = 41;

    public static final double climberP = 0.1;
    public static final double climberI = 0.01;
    public static final double climberD = 0.001;

    public static final double gearConversionFactor = 5; // This is the conversion factor from degrees to rotations
    public static final double fullyRotatedPositionDegrees = 360;
  }

  public static final class CONTROLLER {
    public static final int DRIVER_CONTROLLER_PORT = 0;
}


  public static final class DriveConstants {
    // Ways to drive the robot
    public enum DriveModes {
      MANUAL,
      ALIGNTELE,
      HEADINGLOCK,
      ALIGNAUTO,
    }

    // Driving Parameters - Note that these are not the maximum and minimum capable speeds of
    // the robot, rather the allowed maximum and minimum speeds.
    public static final double maxSpeedFast = 3.3; // 4.1
    public static final double maxSpeedNormal = 2; // 3.3
    public static final double maxSpeedSlow = 0.9; // 1.6
    public static final double maxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 3; // radians per second
    public static final double kMagnitudeSlewRate = 3.5; // percent per second (1 = 100%) // 3.6
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%) // 3.0

    // aggressive rate limits
    // public static final double kDirectionSlewRate = 6;
    // public static final double kMagnitudeSlewRate = 7;
    // public static final double kRotationalSlewRate = 3.5;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 12;
    public static final int kRearLeftDrivingCanId = 10;
    public static final int kFrontRightDrivingCanId = 14;
    public static final int kRearRightDrivingCanId = 16; 

    public static final int kFrontLeftTurningCanId = 13;
    public static final int kRearLeftTurningCanId = 11;
    public static final int kFrontRightTurningCanId = 15;
    public static final int kRearRightTurningCanId = 17;

    public static final boolean kGyroReversed = true;
  }

  public static final class VisionConstants{
    // how far the robot can be from the setpoint during vision alignment
    // these CANNOT be zero, because then commands won't cancel because the robot cannot be perfect
    public static final double allowedXError = 0.1;
    public static final double allowedYError = 0.025; 

    // camera names, transforms, etc.
    public static final String[] cameraNames = new String[] {"Front Left", 
    "Front Right"
  };
    public static final CameraTransform[] cameraOffsets = 
    new CameraTransform[] {
      new CameraTransform(Units.inchesToMeters(14), Units.inchesToMeters(5), 0),
      new CameraTransform(Units.inchesToMeters(14), Units.inchesToMeters(-4.5), 0),
    };

    // these are the actual ones for 2025
    // TagTransform class for holding position, heading, y-rot, etc.

    // since First decided to provide units in inches, I've left the raw units here
    // and just decided to make a function to correct them
    public static final TagTransform[] tagTransforms = new TagTransform[] {
      VisionUtils.correctTagUnits(new TagTransform(0, 0, 0, 0, 0)), // null tag so indexing makes sense
      VisionUtils.correctTagUnits(new TagTransform(657.37, 25.80, 58.50, 126, 0)), // 1
      VisionUtils.correctTagUnits(new TagTransform(657.37, 291.20, 58.50, 234, 0)), // 2
      VisionUtils.correctTagUnits(new TagTransform(455.15, 317.15, 51.25, 270, 0)), // 3
      VisionUtils.correctTagUnits(new TagTransform(365.20, 241.64, 73.54, 0, 30)), // 4
      VisionUtils.correctTagUnits(new TagTransform(365.20, 75.39, 12.13, 0, 30)), // 5
      VisionUtils.correctTagUnits(new TagTransform(530.49, 130.17, 12.13, 300, 0)), // 6
      VisionUtils.correctTagUnits(new TagTransform(546.87, 158.50, 12.13, 0, 0)), // 7
      VisionUtils.correctTagUnits(new TagTransform(530.49, 186.83, 12.13, 60, 0)), // 8
      VisionUtils.correctTagUnits(new TagTransform(497.77, 186.83, 12.13, 120, 0)), // 9
      VisionUtils.correctTagUnits(new TagTransform(481.39, 158.50, 12.13, 180, 0)), // 10
      VisionUtils.correctTagUnits(new TagTransform(497.77, 130.17, 12.13, 240, 0)), // 11
      VisionUtils.correctTagUnits(new TagTransform(33.51, 25.80, 58.50, 54, 0)), // 12
      VisionUtils.correctTagUnits(new TagTransform(33.51, 291.20, 58.50, 306, 0)), // 13
      VisionUtils.correctTagUnits(new TagTransform(325.68, 241.64, 73.54, 180, 30)), // 14
      VisionUtils.correctTagUnits(new TagTransform(325.68, 75.39, 73.54, 180, 30)), // 15
      VisionUtils.correctTagUnits(new TagTransform(235.73, -0.15, 51.25, 90, 0)), // 16
      VisionUtils.correctTagUnits(new TagTransform(160.39, 130.17, 12.13, 240, 0)), // 17
      VisionUtils.correctTagUnits(new TagTransform(144.00, 158.50, 12.13, 180, 0)), // 18
      VisionUtils.correctTagUnits(new TagTransform(160.39, 186.83, 12.13, 120, 0)), // 19
      VisionUtils.correctTagUnits(new TagTransform(193.10, 186.83, 12.13, 60, 0)), // 20
      VisionUtils.correctTagUnits(new TagTransform(209.49, 158.50, 12.13, 0, 0)), // 21
      VisionUtils.correctTagUnits(new TagTransform(193.10, 130.17, 12.13, 300, 0)), // 22
    };

    // the constants used for moving the bot towards the apriltag during vision alignment
    // NOTE - y axis doesn't go through PID, this is only for x
    public static double kAlignP = 1;
    public static double kAlignI = 0.05;
    public static double kAlignD = 0;

    // the constants used for rotation alignment (heading lock and vision alignment)
    public static double kRotP = 0.018;
    public static double kRotI = 0.00001;
    public static double kRotD = 0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.073025;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.07;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0.01;
    public static final double kDrivingFF = 0.25; //1 / kDriveWheelFreeSpeedRps; // why is this comment here it isn't true?????
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.65;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kMainControllerPort = 0;
    public static final int kBackupControllerPort = 1;

    public static final double kDriveDeadband = 0.05;
    public static final double kTurnDeadband = 0.12;
    public static final double kTwistDeadband = 0.5;

    public static final boolean kFieldRelative = true;
    public static final boolean kRateLimited = true;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AutoConstants {
    public static final PathConstraints defaultGlobalContstraints = new PathConstraints(1,0.5, 540, 720);

    public static final PIDConstants translationConstants = new PIDConstants(2, 1, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(1, 0, 0);

    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    //distance from robot center to furthest module
    public static final double kDriveBase = Units.inchesToMeters((Math.sqrt(Math.pow(DriveConstants.kTrackWidth, 2) 
        + Math.pow(DriveConstants.kWheelBase, 2))) / 2);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // POI numbers:
    // Start_ (1, 2, 3, 4, 5, 6) -> fallback starting positions
    // Reef_ (1, 2, 3, 4, 5, 6) -> sides of the reef
    // Station_ (1, 2) -> coral stations

    // test fallback positions
    public static final AutoPOI[] fallbackPositions = new AutoPOI[] {
      // tag indices are set to -1 because these poses have nothing to do with apriltags
      new AutoPOI(new Pose2d(8.000, 4.045, Rotation2d.fromDegrees(180)), "Start1"), // centered on the start line, per actual field
    };

    // should be 28 total
    // TODO: red side
    public static final AutoPOI[] fieldPOIs = new AutoPOI[] {
      // BLUE corals
      new AutoPOI(new Pose2d(1.199, 7.010, Rotation2d.fromDegrees(126)), "BlueCoral1"),
      new AutoPOI(new Pose2d(1.199, 0.968, Rotation2d.fromDegrees(-126)), "BlueCoral2"),

      // BLUE reefs (offset of 0.432 on x, +/-0.165 on y)
      AutoPOI.createPOIFromTag("BlueReef1Left", 18, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("BlueReef1Right", 18, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("BlueReef2Left", 19, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("BlueReef2Right", 19, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("BlueReef3Left", 20, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("BlueReef3Right", 20, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("BlueReef4Left", 21, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("BlueReef4Right", 21, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("BlueReef5Left", 22, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("BlueReef5Right", 22, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("BlueReef6Left", 17, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("BlueReef6Right", 17, new Translation2d(0.432, 0.165)),
      
      // -----------------------

      // RED corals
      new AutoPOI(new Pose2d(16.351, 1.016, Rotation2d.fromDegrees(-54)), "RedCoral1"),
      new AutoPOI(new Pose2d(16.351, 7.046, Rotation2d.fromDegrees(54)), "RedCoral2"),

      // RED reefs (offset of 0.432 on x, +/-0.165 on y)
      AutoPOI.createPOIFromTag("RedReef1Left", 7, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("RedReef1Right", 7, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("RedReef2Left", 6, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("RedReef2Right", 6, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("RedReef3Left", 11, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("RedReef3Right", 11, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("RedReef4Left", 10, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("RedReef4Right", 10, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("RedReef5Left", 9, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("RedReef5Right", 9, new Translation2d(0.432, 0.165)),

      AutoPOI.createPOIFromTag("RedReef6Left", 8, new Translation2d(0.432, -0.165)),
      AutoPOI.createPOIFromTag("RedReef6Right", 8, new Translation2d(0.432, 0.165)),
    };
  }
}
