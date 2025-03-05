package frc.robot.commands.vision;

import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVisionPoseV2;
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
   private PhotonVisionPoseV2 poseEstimator;
   
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

   public AlignTeleopCommand( Boolean fieldRelative, Boolean rateLimit, Drivetrain driveSubsystem, PhotonVisionPoseV2 poseEstimator, Cameras cameraSubsystem, DoubleSupplier offsetX, DoubleSupplier offsetY, BooleanSupplier joystickInput){
        //tagId  = cameraSubsystem.getBestTargetID(cameraSubsystem.getBestTarget(1)); // to be changed. We need to reference the 3 front cameras 
 
        this.offsetX = offsetX;
        this.offsetY = offsetY;

        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        this.driveSubsystem = driveSubsystem;
        this.cameraSubsystem = cameraSubsystem;

        this.addRequirements(driveSubsystem);

        lockedIn = false;
        isDone = false;

        this.joystickInput = joystickInput;

        this.poseEstimator = poseEstimator;
        tagId = poseEstimator.m_camera.getLatestResult().getBestTarget().getFiducialId();
   } 

   @Override
   public void initialize() {

   }

   @Override
   public void execute() {
   
   }

   @Override
   public void end(boolean interrupted) {
    if (interrupted) {
        System.out.println("ALIGN INTERRUPT!");
    }

   }

   @Override
   public boolean isFinished() {
        return true;   
    }
}