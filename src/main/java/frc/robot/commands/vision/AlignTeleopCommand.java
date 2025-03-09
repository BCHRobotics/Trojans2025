package frc.robot.commands.vision;


import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PhotonVisionPoseV2;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SetLEDCommand;


public class AlignTeleopCommand extends Command{
   private PhotonVisionPoseV2 poseEstimator;
   private LED led1;
    private LED led2;
    private PhotonPipelineResult result;
   
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

   double offsetBack = -0.65; // Negative is behind the tag
   double offsetSide; // Negative is to the right

    private String tagSide;

   PathConstraints constraints = new PathConstraints(1.0, 0.5, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
// PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    HashMap<Integer, Pose2d> tagPositions = VisionConstants.getTagPositions();



   public AlignTeleopCommand(Drivetrain driveSubsystem, PhotonVisionPoseV2 poseEstimator, BooleanSupplier joystickInput, String tagSide, LED led1, LED led2) {
    this.tagSide = tagSide;
    this.poseEstimator = poseEstimator;
    this.joystickInput = joystickInput;

    this.addRequirements(driveSubsystem);

    isDone = false;
    
}


   @Override
   public void initialize() {
    new SetLEDCommand(led1, led2, -0.65);
    
    // DO THIS FIRST
    if (tagSide == "LEFT") {
        offsetSide = -0.125; // Negative is to the right
       
    } else if (tagSide == "RIGHT") {
        offsetSide = 0.125; // Positive is to the left
        
    } 

    else if (tagSide == "ALGAE"){
        offsetSide = 0.165;
    }
    if (!poseEstimator.m_cameraMain.getAllUnreadResults().isEmpty()){
        result = poseEstimator.m_cameraMain.getAllUnreadResults().get(0); // FIFO so the first index is the latest result
    }
    else{
        result = null;
        System.out.println("No valid tags detected.");
        isDone = true;
    }
    
    if (result != null && result.hasTargets()) {
        tagId = result.getBestTarget().getFiducialId();
        tagPosition = tagPositions.get(result.getBestTarget().getFiducialId()); // Implement this method in PhotonVisionPoseV2

        System.out.println("Aligning to Tag: " + tagId);

        double newX = tagPosition.getX() - (offsetBack * Math.cos(tagPosition.getRotation().getRadians())) + (offsetSide * Math.sin(tagPosition.getRotation().getRadians()));
        double newY = tagPosition.getY() - (offsetBack * Math.sin(tagPosition.getRotation().getRadians())) - (offsetSide * Math.cos(tagPosition.getRotation().getRadians()));

        // Generate a trajectory to the tag using PathPlanner
        // we want the robot facing the tag so we just flip the rotation
        Command path = poseEstimator.generatePathToPose2d(new Pose2d(newX,newY,new Rotation2d(Units.degreesToRadians(tagPosition.getRotation().getDegrees()-180))));
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
    
    new SetLEDCommand(led1,led2,0.67).schedule();

    if (interrupted) {
        System.out.println("ALIGN INTERRUPT!");
    }


   }

   @Override
   public boolean isFinished() {
    System.out.println("Finished Alignment");

    if (path != null){
        return path.isFinished();
    }
        return path == null || joystickInput.getAsBoolean() || isDone;   
    }
}