package frc.robot.commands.vision;

import frc.robot.subsystems.Drivetrain;


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
   //private Cameras cameraSubsystem;
   
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

   public AlignTeleopCommand(int targetTagId, Boolean fieldRelative, Boolean rateLimit, Drivetrain driveSubsystem, DoubleSupplier offsetX, DoubleSupplier offsetY, BooleanSupplier joystickInput){
        this.driveSubsystem = driveSubsystem;
        //this.cameraSubsystem = cameraSubsystem;
        this.isFieldRelative = fieldRelative;
        this.isRateLimited = rateLimit;
        this.tagId = targetTagId;
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.joystickInput = joystickInput;
   } 

   @Override
   public void initialize() {

   }

   @Override
   public void execute() {

   }

   @Override
   public void end(boolean interrupted) {

   }

   @Override
   public boolean isFinished() {

    return true;
   }
}