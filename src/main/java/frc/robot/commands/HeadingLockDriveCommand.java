package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

/*
 * the whole essence of this command is to use PIDs to keep the robot facing a certain direction,
 * while driving normally
 */
public class HeadingLockDriveCommand extends Command{
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    // These three are inputs for x, y, and rot speed from the driver
    // NOTE - these are NOT DOUBLES, they are DOUBLE SUPPLIERS, which point to a double value
    // this is the easiest way to get the updated speeds, instead of assigning the variables over and over
    DoubleSupplier commandX;
    DoubleSupplier commandY;
    DoubleSupplier commandRot;

    // these constants work okay, but they also kind of suck
    PIDController pid = new PIDController(VisionConstants.kRotP,VisionConstants.kRotI,VisionConstants.kRotD);

    // These two are settings, whether to use ratelimiting and whether to interpret the commands as FS (true) or LS (false)
    BooleanSupplier isFieldRelative;
    BooleanSupplier isRateLimited;

    private boolean hasPressedRotationJoystick;
    private Rotation2d targetAngle;

    public HeadingLockDriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed, BooleanSupplier fieldRelative, BooleanSupplier rateLimit, Drivetrain subsystem) {
        // Assign the variables that point to input values
        commandX = xSpeed;
        commandY = ySpeed;
        commandRot = rotSpeed;
        
        // boolean variables used for the drive function
        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        driveSubsystem = subsystem;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the drive mode
        driveSubsystem.setDriveMode(DriveModes.HEADINGLOCK);
        // Tell the driver that heading lock driving has been enabled
        System.out.println("HEADING LOCK ENGAGED");

        targetAngle = Rotation2d.fromDegrees(0);
    }

    @Override
    public void execute() {
        // call the drive function, passing is x and y inputs as normal,
        // and the result of the pid function for the rotation
        driveSubsystem.drive(
            commandX.getAsDouble(), 
            commandY.getAsDouble(), 
            pid.calculate(driveSubsystem.getHeading(), targetAngle.getDegrees()), 
            isFieldRelative.getAsBoolean(), 
            isRateLimited.getAsBoolean());

        // changi g the target angle of rotation whenever joystick is pressed,
        // but only once per press, hence the boolean variable
        // the boolean variable stops the angle from being updated after the first time
        if (commandRot.getAsDouble() > 0.25 && !hasPressedRotationJoystick) {
            hasPressedRotationJoystick = true;
            targetAngle = Rotation2d.fromDegrees(targetAngle.getDegrees()
             + 360/6); // using 360/6 because the gyro is in degrees (why I have no idea???) and the reef has six sides
        }
        if (commandRot.getAsDouble() < -0.25 && !hasPressedRotationJoystick) {
            hasPressedRotationJoystick = true;
            targetAngle = Rotation2d.fromDegrees(targetAngle.getDegrees()
             - 360/6);
        }
        // when not pushing down the joystick, tell the code that you're not pushing the joystick
        if (Math.abs(commandRot.getAsDouble()) < 0.25 ) {
            hasPressedRotationJoystick = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            System.out.println("HEADING LOCK INTERRUPT!");
        }
        else {
            System.out.println("HEADING LOCK OFF");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not heading lock mode
        // this way all another piece of code has to do is change the drive mode and this is auto-cancelled
        return driveSubsystem.getDriveMode() != DriveModes.HEADINGLOCK;
    }
}
