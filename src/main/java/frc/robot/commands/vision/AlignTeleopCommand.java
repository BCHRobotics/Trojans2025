package frc.robot.commands.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.commands.ToggleMechanismCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PhotonVisionPoseV2;
import frc.utils.MathUtils;

public class AlignTeleopCommand extends Command {
    // reference to the drivetrain, so the command can actually move the bot
    Drivetrain driveSubsystem;
    // reference to the vision subsystem
    PhotonVisionPoseV2 visionSubsystem;
    // reference to the elevator subsystem
    Elevator elevatorSubsystem;
    // claw subsystem
    Harpoon harpoonSubsystem;

    // LED subsystems, which isn't used right now, but we can use it to look at levels
    LED led1;
    LED led2;

    // pid controllers, one for position and one for rotation
    PIDController pid = new PIDController(VisionConstants.kAlignP,VisionConstants.kAlignI,VisionConstants.kAlignD);
    PIDController pidRot = new PIDController(VisionConstants.kRotP,VisionConstants.kRotI,VisionConstants.kRotD);

    // boolean variables used in the drive() function for driving the robot
    Boolean isFieldRelative;
    Boolean isRateLimited;
   
    // storing the tag id that we're trying to align to
    int tagId;

    // if this is true, cancel the vision command
    BooleanSupplier joystickInput;

    // starts off false, then swaps to true when the joysticks are let go of
    // command only cancels when this is set to true
    // this allows us to activate vision while moving, and not cancel immediately because of joystick input
    boolean waitingForInput = false;

    // storing the field relative tag pose in a variable, so that we can deal with losing sight of it
    private Pose2d fieldRelativeTagPose;
    private int desiredTagId;
    private boolean mechActive;

    public AlignTeleopCommand(LED led1, LED led2, Elevator elevatorSubsystem, Harpoon harpoonSubsystem, 
    Drivetrain driveSubsystem, BooleanSupplier joystickInput, int tagId, PhotonVisionPoseV2 visionSubsystem) {
        // setting subsystem references
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.harpoonSubsystem = harpoonSubsystem;

        // LEDs, currently not doing much
        this.led1 = led1;
        this.led2 = led2;

        this.joystickInput = joystickInput;
        this.tagId = tagId;

        this.addRequirements(driveSubsystem);
        
        // defining these values here, for now, not in the constructor
        isFieldRelative = true;
        isRateLimited = true;
    }

    @Override
    public void initialize() {
        // malke sure to use the drive mode system
        driveSubsystem.setDriveMode(DriveModes.ALIGNTELE);

        // until we see a tag, mark the tag pose as null
        fieldRelativeTagPose = null;

        // on initialize, we grab the closest tag id from the pose estimator 
        desiredTagId = visionSubsystem.getClosestTagID();

        mechActive = false;
        // don't cancel the command until we let go of the joystick, when we do this bool flips to true
        waitingForInput = false;
    }

    @Override
    public void execute() {
        if (!joystickInput.getAsBoolean()) {
            waitingForInput = true;
        }

        if (desiredTagId == -1) {
            desiredTagId = visionSubsystem.getClosestTagID();
            return;
        }
        
        // first, we define the desired position
        // we do this periodically because the desired left/right offset might change
        fieldRelativeTagPose = visionSubsystem.getTagPoseOfId(desiredTagId);

        Translation2d tagRelativeDesiredOffset = new Translation2d(visionSubsystem.getXOffset(), visionSubsystem.getYOffset());
        Translation2d fieldRelativeDesiredOffset = MathUtils.applyRotationMatrix(tagRelativeDesiredOffset, fieldRelativeTagPose.getRotation().getRadians());

        // here is the position on the field where we want to drive to
        fieldRelativeTagPose = new Pose2d(
            fieldRelativeTagPose.getX() + fieldRelativeDesiredOffset.getX(),
            fieldRelativeTagPose.getY() + fieldRelativeDesiredOffset.getY(),
            fieldRelativeTagPose.getRotation());

        if (fieldRelativeTagPose == null) {
            return;
        }

        // where the robot currently is on the field
        Pose2d fieldRelativeRobotPose = driveSubsystem.getPose();

        // now we actually command the drive subsystem to drive to the pose
        double commandedX = pid.calculate(fieldRelativeTagPose.getX() - fieldRelativeRobotPose.getX(), 0);
        double commandedY = fieldRelativeTagPose.getY() - fieldRelativeRobotPose.getY();

        double robotAngle = MathUtils.fixAngle(fieldRelativeRobotPose.getRotation().getDegrees());
        double tagAngle = fieldRelativeTagPose.getRotation().minus(Rotation2d.fromDegrees(180)).getDegrees();
        if (tagAngle == -180 && robotAngle > 0) {
            tagAngle = 180;
        }
        if (tagAngle == 180 && robotAngle < 0) {
            tagAngle = 180;
        }
        
        // for now
        double commandedRot = pidRot.calculate(
            robotAngle, 
            tagAngle);

        // clamp the values for safety, also multiply them by -1 because the PID controller will be commanding the wrong sign
        commandedX = MathUtil.clamp(commandedX * -1, -0.2, 0.2);
        commandedY = MathUtil.clamp(commandedY * 1, -0.2, 0.2);
        commandedRot = MathUtil.clamp(commandedRot * 1, -0.5, 0.5);

        if (Math.abs(commandedX) < VisionConstants.allowedXError) {
            commandedX = 0;
        }
        if (Math.abs(commandedY) < VisionConstants.allowedYError) {
            commandedY = 0;
        }

        // set the 2 zeros back to commandedX and commandedY when finished testing
        driveSubsystem.drive(commandedX, commandedY, commandedRot, true, true);

        // moving the mech
        if (MathUtils.getDistance(fieldRelativeRobotPose, fieldRelativeTagPose) < 2 && !mechActive) {
            new ToggleMechanismCommand(led1, led2, elevatorSubsystem, harpoonSubsystem).schedule();
            mechActive = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return waitingForInput && joystickInput.getAsBoolean();
    }
}