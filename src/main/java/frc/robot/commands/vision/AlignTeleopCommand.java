package frc.robot.commands.vision;


import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVisionPoseV2;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants.VisionConstants;


public class AlignTeleopCommand extends Command{
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

   private Command path;

   PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
// PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    HashMap<Integer, Pose2d> tagPositions = VisionConstants.getTagPositions();



   public AlignTeleopCommand( Drivetrain driveSubsystem, PhotonVisionPoseV2 poseEstimator, BooleanSupplier joystickInput) {

    this.poseEstimator = poseEstimator;
    this.joystickInput = joystickInput;

    this.addRequirements(driveSubsystem);

    lockedIn = false;
    isDone = false;
    
}


   @Override
   public void initialize() {
    // DO THIS FIRST
    

    var result = poseEstimator.m_camera.getLatestResult();
    if (result.hasTargets()) {
        tagId = result.getBestTarget().getFiducialId();
        tagPosition = tagPositions.get(result.getBestTarget().getFiducialId()); // Implement this method in PhotonVisionPoseV2

        System.out.println("Aligning to Tag: " + tagId);

        // Generate a trajectory to the tag using PathPlanner
        Command path = poseEstimator.generatePathToPose2d(new Pose2d(tagPosition.getX()-0.2,tagPosition.getY(),tagPosition.getRotation()));
        path.schedule();
    } else {
        System.out.println("No valid tags detected.");
        isDone = true;
        
    }
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
        return path.isFinished() || path == null || joystickInput.getAsBoolean();   
    }
}