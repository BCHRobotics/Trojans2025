// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.utils.AutoUtils;
import frc.utils.SwerveUtils;
import frc.utils.VisionCapture;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  // A percentage value (0-1) for the linear speed of the robot
  private double m_maxSpeed = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // boolean variable for switching between 3 diff robot speeds (slow, medium, fast)
  private boolean m_slowMode = false;
  private boolean m_fastMode = false;

  // boolean for keeping track of robot alliance (used for flipping auto path)
  public boolean isRedAlliance;
  
  // enum for keeping track of how the robot is driving
  private DriveModes driveMode = DriveModes.MANUAL;

  private Translation2d odometryOffset = new Translation2d(0, 0);

  private List<VisionCapture> visionPoses = new LinkedList<VisionCapture>();
  private Transform2d visionPose = null ;

  private double visionFadeTime = 2;
  private double visionUpdateTime = 2;
  private double lastVisionUpdate;

  private double visionCoefficient = 0.5;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    this.initializeAuto();
  }

  // Very important drive mode functions that make everything work
  public DriveModes getDriveMode() {
    return driveMode;
  }

  public void setDriveMode(DriveModes modeToSet) {
    driveMode = modeToSet;
  }

  public void addVisionMeasurement(Pose2d visionMeasurement) {
    visionPoses.add(new VisionCapture(visionMeasurement, getPose()));
  }

  public void removeOldVisionMeasurements() {
    for (int i = 0; i < visionPoses.size(); i++) {
      if (visionPoses.get(i).timeTaken < Timer.getFPGATimestamp() - visionFadeTime) {
        visionPoses.remove(i);
        break;
      }
    }
  }

  public void updateVisionPose() {
    if (visionPoses.size() == 0) {return;}
    
    Transform2d averageVisionPose = new Transform2d(0, 0, new Rotation2d());
    for (int i = 0; i < visionPoses.size(); i++) {
      VisionCapture capture = visionPoses.get(i);
      Transform2d visionOffset = capture.visionMeasurement.minus(capture.odometryMeasurement);

      averageVisionPose = averageVisionPose.plus(visionOffset);
    }

    averageVisionPose = averageVisionPose.div(visionPoses.size());

    visionPose = averageVisionPose;
  }

  @Override
  public void periodic() {
    // get rid of any outdated pose estimation measurements
    removeOldVisionMeasurements();

    if (Timer.getFPGATimestamp() > lastVisionUpdate + visionUpdateTime) {
      updateVisionPose();
      lastVisionUpdate = Timer.getFPGATimestamp();
    }

    // Set the max speed of the bot
    setSpeedPercent();

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Print debug values to smartDashboard
    this.printToDashboard();
  }

  /*
   * telling the drivetrain how to offset the odometry to align with the tag
   */
  public void setOdometryOffset(Translation2d offset) {
    odometryOffset = offset;
  }

  /*
   * get the pose, but with LYING involved
   */
  public Pose2d getOffsetedPose() {
    return m_odometry.getPoseMeters().plus(new Transform2d(odometryOffset, new Rotation2d()));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /*
   * odometry and vision
   */
  public Pose2d getCompositePose() {
    if (visionPose != null) {
      return m_odometry.getPoseMeters().plus(visionPose.times(visionCoefficient));
    }
    else {
      return m_odometry.getPoseMeters();
    }
      
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /*
   * A function for setting the alliance of the robot
   */
  public void setAlliance(boolean isRed) {
    isRedAlliance = isRed;
  }

  /*
   * A function for getting the alliance of the robot
   * if true, the alliance is RED
   */
  public boolean getAlliance() {
    return isRedAlliance;
  }
  
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param maxSpeed      A 0-1 multiplier for the x and y speed of the robot.
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // Some small number to avoid floating-point errors with equality checking
          // Keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    /*
     * Convert the commanded speeds into the correct units for the drivetrain,
     * using the given max speed
     */
    double xSpeedDelivered = xSpeedCommanded * m_maxSpeed;
    double ySpeedDelivered = ySpeedCommanded * m_maxSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.maxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-this.m_odometry.getPoseMeters().getRotation().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
 
    this.setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   * This does not set the brake mode of the motors.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the swerve ModuleStates.
   *
   * @return The current SwerveModule states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    // Change this after
    resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -Infinity to Infinity
   */
  public double getHeading() {
    /* I'm multiplying the navx heading by -1 
    * because WPILib uses CCW as the positive direction
    * and NavX uses CW as the positive direction
    */ 
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Enables and disables slow mode.
   *
   * @param mode Whether to enable slow mode on or off.
   */
  public void setSlowMode(boolean mode) {
    this.m_slowMode = mode;
    setSpeedPercent();
  }

  /**
   * Enables and disables fast mode.
   *
   * @param mode Whether to enable fast mode on or off.
   */
  public void setFastMode(boolean mode) {
    this.m_fastMode = mode;
    setSpeedPercent();
  }

  /**
   * Sets the speed of the robot to a desired m/s value
   *
   * @param percent The desired speed in metres per second
   */
  public void setSpeedPercent() {
    if (m_slowMode) {
      m_maxSpeed = DriveConstants.maxSpeedSlow;
    } else if (m_fastMode) {
      m_maxSpeed = DriveConstants.maxSpeedFast;
    } else {
      m_maxSpeed = DriveConstants.maxSpeedNormal;
    }
  }

  // /**
  //  * Initializes the auto builder using PathPlannerLib.
  //  */
  public void initializeAuto() {
    RobotConfig robotConfig = AutoUtils.geRobotConfig();

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getChassisSpeeds, 
      this::setChassisSpeeds, 
      new PPHolonomicDriveController(
        Constants.AutoConstants.translationConstants, 
        Constants.AutoConstants.rotationConstants, 0.02), 
        robotConfig, 
        this::getAlliance, 
        this);
  }   
  
  /**
   * Sets the speed of the robot chassis.
   * @param speed The new chassis speed.
   */
  public void setChassisSpeeds(ChassisSpeeds speed, DriveFeedforwards ff) {
    this.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speed));
  }

  /**
   * Gets the speed of the robot chassis.
   * @return The current chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(this.getModuleStates());
  }

  /** Prints all values to the dashboard. */
  public void printToDashboard() {
  }
}
