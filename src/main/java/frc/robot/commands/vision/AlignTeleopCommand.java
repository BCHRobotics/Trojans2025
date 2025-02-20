package frc.robot.commands.vision;

import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.utils.VisionUtils;
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
   Translation2d desiredOffset;

   public AlignTeleopCommand(int targetTagId, Boolean fieldRelative, Boolean rateLimit, Drivetrain driveSubsystem, Cameras cameraSubsystem, Translation2d offset){
        tagId  = targetTagId;
        desiredOffset = offset;

        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        this.driveSubsystem = driveSubsystem;
        this.cameraSubsystem = cameraSubsystem;

        addRequirements(driveSubsystem);
   } 
   @Override
   public void initialize() {
    System.out.println("ALIGN ON");
       // Set the drive mode
       driveSubsystem.setDriveMode(DriveModes.ALIGNTELE);
   }

   @Override
   public void execute() {
        // since the vector provided is a local vector, we make it field-relative
        Translation2d fieldRelativeTagOffset = VisionUtils.applyRotationMatrix(desiredOffset, -VisionConstants.tagTransforms[tagId].headingAngle * Math.PI / 180);

        // how far the robot is from the tag
        Transform2d fieldRelativeRobotToTag = cameraSubsystem.getFieldOrientedTagOffset(tagId);

        // by default we do not use fast mode
        driveSubsystem.setFastMode(false);

        // if we cannot see the tag, use the last known location of the tag
        if (fieldRelativeRobotToTag == null) {
            // if null, cannot see tag
            Pose2d tagPose = VisionConstants.tagTransforms[tagId].getPosition();
            if (tagPose != null) {fieldRelativeRobotToTag = tagPose.minus(driveSubsystem.getPose());}
            
            // push the desired offset back a meter so we can get a view of the tag
            fieldRelativeTagOffset=fieldRelativeTagOffset.plus(VisionUtils.applyRotationMatrix(
                new Translation2d(1, 0), -VisionConstants.tagTransforms[tagId].headingAngle * Math.PI / 180));
            
            // because we pushed the offset back, there's less risk of collision and we can go fast
            driveSubsystem.setFastMode(true);
        }

        // combine the robotToTag with the tagOffset
        Transform2d actualOffset = fieldRelativeRobotToTag.plus(new Transform2d(fieldRelativeTagOffset.times(-1), new Rotation2d()));

        // making sure the var isn't null (something may have gone wrong in the previous step)
        if (fieldRelativeRobotToTag != null) {
            // figure out what speeds to command to the drivetrain on 2 axis
            double commandedX = pid.calculate(-actualOffset.getX(), 0);
            double commandedY = actualOffset.getY();
            
            // the rotational speed
            double commandedRot = pidRot.calculate(
                Rotation2d.fromDegrees(VisionConstants.tagTransforms[tagId].headingAngle).
                minus(Rotation2d.fromDegrees(driveSubsystem.getHeading())).
                plus(Rotation2d.fromDegrees(180))
                .getDegrees(), 0);

            // clamp x and y speeds for testing, don't want the robot hitting anything
            commandedX = MathUtil.clamp(commandedX, -0.2, 0.2);
            commandedY = MathUtil.clamp(commandedY, -0.2, 0.2);

            // pass all values to the drivetrain
            driveSubsystem.drive(commandedX, commandedY, -commandedRot, isFieldRelative, isRateLimited);
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
       return (driveSubsystem.getDriveMode() != DriveModes.ALIGNTELE || VisionUtils.hasReachedPosition(tagId, desiredOffset, driveSubsystem, cameraSubsystem));
   }
}