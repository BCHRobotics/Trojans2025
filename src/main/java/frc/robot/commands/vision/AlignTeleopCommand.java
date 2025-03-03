package frc.robot.commands.vision;

import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants.VisionConstants;


public class AlignTeleopCommand extends Command{
   private Drivetrain driveSubsystem;
   private PoseEstimatorSubsystem poseEstimatorSubsystem;
   
   PIDController pid = new PIDController(VisionConstants.kAlignP,VisionConstants.kAlignI,VisionConstants.kAlignD);
   PIDController pidRot = new PIDController(VisionConstants.kRotP,VisionConstants.kRotI,VisionConstants.kRotD);

   Boolean isFieldRelative;
   Boolean isRateLimited;
   
   int tagId;
   
   DoubleSupplier offsetX;
   DoubleSupplier offsetY;

   Pose2d tagPosition 
   = new Pose2d(657.37, 25.80, new Rotation2d(Units.degreesToRadians(0))); // this is tag 1 for now

   boolean lockedIn;
   boolean isDone;

   BooleanSupplier joystickInput;

   double currentX;
   double currentY;
   double differenceInX;;
   double differenceInY;
   double xOutput;
   double yOutput;

   public AlignTeleopCommand(int targetTagId, PoseEstimatorSubsystem poseEstimatorSubsystem, Boolean fieldRelative, Boolean rateLimit, Drivetrain driveSubsystem, DoubleSupplier offsetX, DoubleSupplier offsetY, BooleanSupplier joystickInput){
        this.driveSubsystem = driveSubsystem;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
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
      currentX = poseEstimatorSubsystem.getCurrentPose().getX();
      currentY = poseEstimatorSubsystem.getCurrentPose().getY();

      differenceInX = 
         tagPosition.getX() + offsetX.getAsDouble() - currentX;
      differenceInY = 
         tagPosition.getY() + offsetY.getAsDouble() - currentY;

      xOutput = pid.calculate(differenceInX);
      yOutput = pid.calculate(differenceInY);

      driveSubsystem.drive(xOutput, yOutput, 0, isFieldRelative, isRateLimited);
   }

   @Override
   public void end(boolean interrupted) {

   }

   @Override
   public boolean isFinished() {
      if (currentX < VisionConstants.allowedXError && currentY < VisionConstants.allowedYError){
         return true;
      }
   return joystickInput.getAsBoolean();
   }
}