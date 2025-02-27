package frc.robot.commands.vision;

import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.utils.VisionUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

   boolean lockedIn;

   public AlignTeleopCommand(int targetTagId, Boolean fieldRelative, Boolean rateLimit, Drivetrain driveSubsystem, Cameras cameraSubsystem, Translation2d offset){
        //tagId  = cameraSubsystem.getBestTargetID(cameraSubsystem.getBestTarget(1)); // to be changed. We need to reference the 3 front cameras 
        tagId = targetTagId;
        desiredOffset = offset;

        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        this.driveSubsystem = driveSubsystem;
        this.cameraSubsystem = cameraSubsystem;

        addRequirements(driveSubsystem);

        lockedIn = false;
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
        Translation2d fieldRelativeTagOffset = VisionUtils.applyRotationMatrix(desiredOffset, VisionConstants.tagTransforms[tagId].headingAngle * Math.PI / 180);

        // how far the robot is from the tag
        Transform2d fieldRelativeRobotToTag = cameraSubsystem.getFieldOrientedTagOffset(tagId);

        // by default we do not use fast mode
        driveSubsystem.setFastMode(false);

        // making sure the var isn't null (something may have gone wrong in the previous step)
        if (fieldRelativeRobotToTag != null) {
            Transform2d actualOffset = new Transform2d(
                fieldRelativeRobotToTag.getX() + fieldRelativeTagOffset.getX(),
                fieldRelativeRobotToTag.getY() + fieldRelativeTagOffset.getY(),
                fieldRelativeRobotToTag.getRotation()
            );

            lockedIn = true;

            // figure out what speeds to command to the drivetrain on 2 axis
            double commandedX = pid.calculate(-actualOffset.getX(), 0);
            double commandedY = actualOffset.getY() < 0 ? -0.1 : 0.1;

            if (Math.abs(actualOffset.getY()) < 0.1) {
                commandedY = actualOffset.getY() * 0.5;
            }  
            
            // the rotational speed
            double commandedRot = pidRot.calculate(
                driveSubsystem.getPose().getRotation().getDegrees(), 
                VisionConstants.tagTransforms[tagId].headingAngle - 180);

            // clamp x and y speeds for testing, don't want the robot hitting anything
            commandedX = MathUtil.clamp(commandedX, -0.15, 0.15);
            commandedY = MathUtil.clamp(commandedY, -0.15, 0.15);

            commandedRot = MathUtil.clamp(commandedRot, -0.2, 0.2);

            if (Math.abs(actualOffset.getY()) < VisionConstants.allowedYError) {
                commandedY = 0;
            }

            // pass all values to the drivetrain
            driveSubsystem.drive(commandedX, commandedY, commandedRot, isFieldRelative, isRateLimited);
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
       return (driveSubsystem.getDriveMode() != DriveModes.ALIGNTELE);
   }
}