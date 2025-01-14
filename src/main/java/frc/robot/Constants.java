// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.utils.CameraTransform;
import frc.utils.TagTransform;
import frc.utils.devices.VisionUtils;


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
  public static final class DriveConstants {
    // Ways to drive the robot
    public enum DriveModes {
      MANUAL,
      ALIGNREEF,
      HEADINGLOCK
    }

    // Driving Parameters - Note that these are not the maximum and minimum capable speeds of
    // the robot, rather the allowed maximum and minimum speeds.
    public static final double kMaxSpeedMetersPerSecond = 3.3; // 4.1
    public static final double a = 1.6; // 3.3
    public static final double kMinSpeedMetersPerSecond = 0.9; // 1.6
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 3.5; // percent per second (1 = 100%) // 3.6
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%) // 3.0

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
    // camera names, transforms, etc.
    public static final String[] cameraNames = new String[] {"Front"};
    public static final CameraTransform[] cameraOffsets = 
    new CameraTransform[] {
      new CameraTransform(0, 0, 0),
    };

    // these are the headings we're using for vision testing
    public static final double[] testTagHeadings = new double[] {
      0,
      0,
      0,
      180,
    };  

    // these are the actual ones for 2025
    // TagTransform class for holding position, heading, y-rot, etc.

    // since First decided to provide units in inches, I've left the raw units here
    // and just decided to make a function to correct them
    public static final TagTransform[] tagTransforms = new TagTransform[] {
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

    public static double kAlignP = 0.25;
    public static double kAlignI = 0;
    public static double kAlignD = 0;

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
    public static final double kWheelDiameterMeters = 0.0762;
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
    public static final double kDrivingFF = 0.25; //1 / kDriveWheelFreeSpeedRps;
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

  public static final class AutoConstants {
    public static final PIDConstants translationConstants = new PIDConstants(8, 8, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0);

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
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
