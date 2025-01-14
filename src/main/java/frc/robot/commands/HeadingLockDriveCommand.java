package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

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
        // Tell the driver that manual driving has been enabled
        System.out.println("HEADING LOCK ENGAGED");

        targetAngle = Rotation2d.fromDegrees(0);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(
            commandX.getAsDouble(), 
            commandY.getAsDouble(), 
            pid.calculate(driveSubsystem.getHeading(), targetAngle.getDegrees()), 
            isFieldRelative.getAsBoolean(), 
            isRateLimited.getAsBoolean());

        if (commandRot.getAsDouble() > 0.25 && !hasPressedRotationJoystick) {
            hasPressedRotationJoystick = true;
            targetAngle = Rotation2d.fromDegrees(targetAngle.getDegrees() + 360/6);
        }
        if (commandRot.getAsDouble() < -0.25 && !hasPressedRotationJoystick) {
            hasPressedRotationJoystick = true;
            targetAngle = Rotation2d.fromDegrees(targetAngle.getDegrees() - 360/6);
        }
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
        // End if the drive mode is not manual
        return driveSubsystem.getDriveMode() != DriveModes.HEADINGLOCK;
    }
}
