package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.subsystems.Drivetrain;
import frc.utils.MathUtils;
import frc.utils.Polygon;
import frc.utils.Vector2;

public class FollowPointsCommand extends Command{
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    // These two are settings, whether to use ratelimiting and whether to interpret the commands as FS (true) or LS (false)
    BooleanSupplier isRateLimited;
    
    // the "points" that the robot needs to follow for the trajectory
    public Pose2d[] poses;  
    // the index of the pose that the robot is currently driving towards
    public int targetIndex;
    private double acceptableDistance = 0.1;

    public Polygon[] obstacles;

    private boolean isFinished;

    public FollowPointsCommand(BooleanSupplier rateLimit, Drivetrain subsystem) {
        // Assign the variables that point to input values
        isRateLimited = rateLimit;

        driveSubsystem = subsystem;

        // TODO: define where the obstacles are
        obstacles = new Polygon[] {new Polygon(new Vector2[] {new Vector2(3, 1), new Vector2(3, -1), new Vector2(1, -1), new Vector2(1, 1)})};
        
        // for now, just hard-coding these
        poses = MathUtils.calculatePath(new Pose2d(0, 0, new Rotation2d()), new Pose2d(4, 0, new Rotation2d()), obstacles);
        // we start by driving to the first point
        targetIndex = 0;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        addRequirements(driveSubsystem);

        isFinished = false;
    }

    @Override
    public void initialize() {
        // Set the drive mode
        driveSubsystem.setDriveMode(DriveModes.POINT);
        // Tell the driver that manual driving has been enabled
        System.out.println("POINT DRIVING ENGAGED");

        System.out.println("target :" + targetIndex);
    }

    @Override
    public void execute() {
        // // instead of actually doing anything, just log the points
        // String debugMsg = "calculated points: ";
        // for (int i = 0; i < poses.length; i++) {debugMsg += "(" + poses[i].getX() + "," + poses[i].getY() + ")   ";}
        // System.out.println(debugMsg);

        if (targetIndex >= poses.length) {return;}
        
        Pose2d currentPose = driveSubsystem.getPose();

        //first, if we've made it to the desired pose, target the next one
        if (MathUtils.getDistance(currentPose, poses[targetIndex]) < acceptableDistance) {
            targetIndex++;
            System.out.println("new target :" + targetIndex);
            if (targetIndex >= poses.length) {
                // we have reached the last point, so command over
                isFinished = true;
            }
        }

        if (targetIndex >= poses.length) {return;}

        Pose2d targetPose = poses[targetIndex];
        
        double xCommand = targetPose.getX() - currentPose.getX();
        double yCommand = targetPose.getY() - currentPose.getY();
        
        driveSubsystem.drive(xCommand * 0.5, yCommand * 0.5, 0, true, isRateLimited.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            // This will happen most of the time, e.g. when switching to vision
            System.out.println("POINT INTERRUPT!");
        }
        else {
            // This happens when the driving mode switches off manual
            // Doesn't usually happen, for example when a vision command is triggered 
            // it is setup before the drive mode is set
            // So the program interrupts this command (because of the new one)
            // before it realizes the driveMode isn't manual anymore
            System.out.println("POINT DRIVING OFF");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not point mode
        return driveSubsystem.getDriveMode() != DriveModes.POINT || isFinished;
    }
}
