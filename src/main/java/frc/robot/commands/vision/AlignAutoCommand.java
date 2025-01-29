package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.utils.VisionUtils;

public class AlignAutoCommand extends Command {
   private Drivetrain driveSubsystem;
   private Cameras cameraSubsystem;

   int tagId;
   Translation2d desiredOffset;

   public AlignAutoCommand(int targetTagId, Translation2d offset, Drivetrain driveSubsystem, Cameras cameraSubsystem){
        tagId = targetTagId;
        desiredOffset = offset;

        this.driveSubsystem = driveSubsystem;
        this.cameraSubsystem = cameraSubsystem;
   } 

   @Override
   public void initialize() {
    System.out.println("AUTO-ALIGN ON");
       // Set the drive mode
       driveSubsystem.setDriveMode(DriveModes.ALIGNAUTO);
       
       System.out.println("----------");
       System.out.println(desiredOffset);
       System.out.println("----------");
   }

   @Override
   public void execute() {
        // set the odometry offset that is going to guide the robot towards the tag
        Transform2d tagOffset = cameraSubsystem.getFieldOrientedTagOffset(tagId);

        tagOffset = tagOffset.plus(new Transform2d(VisionUtils.applyRotationMatrix(desiredOffset, -driveSubsystem.getHeading()), new Rotation2d()));

        driveSubsystem.setOdometryOffset(tagOffset.getTranslation().times(-1));
    }

   @Override
   public void end(boolean interrupted) {
    if (interrupted) {
        System.out.println("AUTO-ALIGN INTERRUPT!");
    }
    else {
        System.out.println("AUTO-ALIGN OFF");
    }
   }

   @Override
   public boolean isFinished() {
        return (driveSubsystem.getDriveMode() != DriveModes.ALIGNAUTO || VisionUtils.hasReachedPosition(tagId, desiredOffset, driveSubsystem, cameraSubsystem));
   }
}
