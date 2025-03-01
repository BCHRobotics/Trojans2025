package frc.robot.commands.vision;

import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.utils.VisionUtils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.Constants.VisionConstants;


public class AlignTeleopCommand extends Command{
   private Drivetrain driveSubsystem;
   private Cameras cameraSubsystem;
   
   PIDController pid = new PIDController(VisionConstants.kAlignP,VisionConstants.kAlignI,VisionConstants.kAlignD);
   PIDController pidRot = new PIDController(VisionConstants.kRotP,VisionConstants.kRotI,VisionConstants.kRotD);

   Boolean isFieldRelative;
   Boolean isRateLimited;
   
   int tagId;
   
   DoubleSupplier offsetX;
   DoubleSupplier offsetY;

   Pose2d tagPosition;

   boolean lockedIn;
   boolean isDone;

   BooleanSupplier joystickInput;

   public AlignTeleopCommand(int targetTagId, Boolean fieldRelative, Boolean rateLimit, Drivetrain driveSubsystem, Cameras cameraSubsystem, DoubleSupplier offsetX, DoubleSupplier offsetY, BooleanSupplier joystickInput){
        //tagId  = cameraSubsystem.getBestTargetID(cameraSubsystem.getBestTarget(1)); // to be changed. We need to reference the 3 front cameras 
        tagId = targetTagId;
        
        this.offsetX = offsetX;
        this.offsetY = offsetY;

        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        this.driveSubsystem = driveSubsystem;
        this.cameraSubsystem = cameraSubsystem;

        addRequirements(driveSubsystem);

        lockedIn = false;
        isDone = false;

        this.joystickInput = joystickInput;
   } 

   @Override
   public void initialize() {
    System.out.println("ALIGN ON! Tag ID: " + tagId);

    // Set the drive mode
    driveSubsystem.setDriveMode(DriveModes.ALIGNTELE);
   }

   @Override
   public void execute() {
    if (tagId <= 0) { return;}
        // since the vector provided is a local vector, we make it field-relative
        Translation2d fieldRelativeTagOffset = VisionUtils.applyRotationMatrix(new Translation2d(offsetX.getAsDouble(), offsetY.getAsDouble()), VisionConstants.tagTransforms[tagId].headingAngle * Math.PI / 180);

        Translation2d pushbackOffset = VisionUtils.applyRotationMatrix(new Translation2d(0.5, 0), VisionConstants.tagTransforms[tagId].headingAngle * Math.PI / 180);

        // how far the robot is from the tag
        Transform2d fieldRelativeRobotToTag = cameraSubsystem.getFieldOrientedTagOffset(tagId);

        // by default we do not use fast mode
        driveSubsystem.setFastMode(false);

        // making sure the var isn't null (something may have gone wrong in the previous step)
        if (fieldRelativeRobotToTag != null) {
            Transform2d actualOffset = new Transform2d(0, 0, new Rotation2d());

            if (!lockedIn) {
                actualOffset = new Transform2d(
                fieldRelativeRobotToTag.getX() + fieldRelativeTagOffset.getX() + pushbackOffset.getX(),
                fieldRelativeRobotToTag.getY() + fieldRelativeTagOffset.getY() + pushbackOffset.getY(),
                fieldRelativeRobotToTag.getRotation()
                );
            }
            else {
                actualOffset = new Transform2d(
                fieldRelativeRobotToTag.getX() + fieldRelativeTagOffset.getX(),
                fieldRelativeRobotToTag.getY() + fieldRelativeTagOffset.getY(),
                fieldRelativeRobotToTag.getRotation()
                );
            }

            // figure out what speeds to command to the drivetrain on 2 axis
            double commandedX = pid.calculate(-actualOffset.getX(), 0);
            double commandedY = actualOffset.getY();

            if (Math.abs(actualOffset.getY()) < 0.1) {
                commandedY = actualOffset.getY() * 0.2;
            }  

            // TODO: test on practice field @ 11:30
            
            // the rotational speed
            double commandedRot = pidRot.calculate(
                Rotation2d.fromDegrees(driveSubsystem.getPose().getRotation().getDegrees()).
                minus(Rotation2d.fromDegrees(180)).getDegrees(), 
                VisionConstants.tagTransforms[tagId].headingAngle);

            // ---

            // clamp x and y speeds for testing, don't want the robot hitting anything
            commandedX = MathUtil.clamp(commandedX, -VisionConstants.speedLimitX, VisionConstants.speedLimitX);
            commandedY = MathUtil.clamp(commandedY, -VisionConstants.speedLimitY, VisionConstants.speedLimitY);
            // ditto with rotational commanded speed
            commandedRot = MathUtil.clamp(commandedRot, -VisionConstants.speedLimitRot, VisionConstants.speedLimitRot);

            if (Math.abs(actualOffset.getY()) < VisionConstants.allowedYError) {
                commandedY = 0;
            }

            // pass all values to the drivetrain
            driveSubsystem.drive(commandedX, commandedY, commandedRot, isFieldRelative, isRateLimited);
            
            if (lockedIn && VisionUtils.hasReachedPosition(tagId, actualOffset.getTranslation(), driveSubsystem, cameraSubsystem)) {
                isDone = true;
            }
            if (!lockedIn && VisionUtils.hasReachedPosition(tagId, actualOffset.getTranslation(), driveSubsystem, cameraSubsystem)) {
                lockedIn = true;
            }

            tagPosition = new Pose2d(
                driveSubsystem.getPose().getX() + actualOffset.getX(),
                driveSubsystem.getPose().getY() + actualOffset.getY(),
                driveSubsystem.getPose().getRotation().plus(actualOffset.getRotation())
            );
        }
        else if (lockedIn) {
            Transform2d actualOffset = new Transform2d(
                tagPosition.getX() - driveSubsystem.getPose().getX(),
                tagPosition.getY() - driveSubsystem.getPose().getY(),
                tagPosition.getRotation().minus(driveSubsystem.getPose().getRotation())
            );

            // figure out what speeds to command to the drivetrain on 2 axis
            double commandedX = pid.calculate(-actualOffset.getX(), 0);
            double commandedY = actualOffset.getY(); 

            if (Math.abs(actualOffset.getY()) < 0.1) {
                commandedY = actualOffset.getY() * 0.2;
            }  
            
            // the rotational speed
            // double commandedRot = pidRot.calculate(
            //     Rotation2d.fromDegrees(driveSubsystem.getPose().getRotation().getDegrees()).
            //     minus(Rotation2d.fromDegrees(180)).getDegrees(), 
            //     VisionConstants.tagTransforms[tagId].headingAngle);

            double commandedRot = 0;

            // clamp x and y speeds for testing, don't want the robot hitting anything
            commandedX = MathUtil.clamp(commandedX, -0.5, 0.5);
            commandedY = MathUtil.clamp(commandedY, -0.5, 0.5);

            commandedRot = MathUtil.clamp(commandedRot, -0.3, 0.3);

            if (Math.abs(actualOffset.getY()) < VisionConstants.allowedYError) {
                commandedY = 0;
            }

            // pass all values to the drivetrain
            driveSubsystem.drive(commandedX, commandedY, commandedRot, isFieldRelative, isRateLimited);
            
            if (VisionUtils.hasReachedPosition(tagId, actualOffset.getTranslation(), driveSubsystem, cameraSubsystem)) {
                isDone = true;
            }
        }
        else {
            driveSubsystem.drive(0, 0, 0, isFieldRelative, isRateLimited);
        }
   }

   @Override
   public void end(boolean interrupted) {
    if (interrupted) {
        System.out.println("ALIGN INTERRUPT!");
    }
    else {
        System.out.println("ALIGN OFF");
    }
   }

   @Override
   public boolean isFinished() {
       return driveSubsystem.getDriveMode() != DriveModes.ALIGNTELE || isDone || joystickInput.getAsBoolean() || tagId <= 0;
   }
}