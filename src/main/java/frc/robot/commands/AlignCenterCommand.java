package frc.robot.commands;

import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants.DriveModes;

import frc.robot.Constants.VisionConstants;


public class AlignCenterCommand extends Command{
   private Drivetrain driveSubsystem;
   private Cameras cameraSubsystem;
   
   PIDController pid = new PIDController(VisionConstants.kAlignP,VisionConstants.kAlignI,VisionConstants.kAlignD);
   PIDController pidRot = new PIDController(VisionConstants.kRotP,VisionConstants.kRotI,VisionConstants.kRotD);

   Boolean isFieldRelative;
   Boolean isRateLimited;
   


   public AlignCenterCommand(Boolean fieldRelative, Boolean rateLimit, Drivetrain driveSubsystem, Cameras cameraSubsystem){
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
       driveSubsystem.setDriveMode(DriveModes.ALIGNREEF);
   }

   @Override
   public void execute() {
        // 4 is just the test tag we're working with rn
        Transform2d tagOffset = cameraSubsystem.getFieldOrientedTagOffset(4);
        double xOff = 0.3;
        driveSubsystem.setFastMode(false);

        if (tagOffset == null) {
            // if null, cannot see tag
            Pose2d cameraPose = cameraSubsystem.getTagTestPosition(4);
            if (cameraPose != null) {tagOffset = cameraPose.minus(driveSubsystem.getPose());}

            xOff = 1.5;
            driveSubsystem.setFastMode(true);
        }

        if (tagOffset != null) {
            double commandedX = pid.calculate(-(tagOffset.getX() - xOff), 0);
            double commandedY = tagOffset.getY();

            double commandedRot = pidRot.calculate(
                Rotation2d.fromDegrees(VisionConstants.testTagHeadings[3]).
                minus(Rotation2d.fromDegrees(driveSubsystem.getHeading())).
                plus(Rotation2d.fromDegrees(180))
                .getDegrees(), 0);

            commandedX = MathUtil.clamp(commandedX, -1, 1);
            commandedY = MathUtil.clamp(commandedY, -1, 1);

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
        // 4 is just the test tag we're working with rn
        Transform2d tagOffset = cameraSubsystem.getFieldOrientedTagOffset(4);

        if (tagOffset == null) {
            // if null, cannot see tag
            Pose2d cameraPose = cameraSubsystem.getTagTestPosition(4);
            if (cameraPose != null) {tagOffset = cameraPose.minus(driveSubsystem.getPose());}
        }

       return (driveSubsystem.getDriveMode() != DriveModes.ALIGNREEF || (Math.abs(tagOffset.getX() - 0.3) < 0.1 && Math.abs(tagOffset.getY()) < 0.05));
   }
}
