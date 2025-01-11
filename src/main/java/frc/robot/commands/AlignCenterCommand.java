package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.Drivetrain;
import frc.utils.devices.PhotonVision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants.DriveModes;

import frc.robot.Constants.AutoConstants;


public class AlignCenterCommand extends Command{
   private Drivetrain driveSubsystem;
   PIDController pid = new PIDController(AutoConstants.kPXController,AutoConstants.kPYController,AutoConstants.kPThetaController);

   Double commandRot;
   BooleanSupplier isFieldRelative;
   BooleanSupplier isRateLimited;
   


   public AlignCenterCommand(Double rotationSpeed,BooleanSupplier fieldRelative, BooleanSupplier rateLimit, Drivetrain subsystem){
        commandRot = rotationSpeed;
        rotationSpeed = pid.calculate(PhotonVision.centerYaw,0);
        isFieldRelative = fieldRelative;
        isRateLimited = rateLimit;

        driveSubsystem = subsystem;

        addRequirements(driveSubsystem);
   } 
   @Override
   public void initialize() {
       // Set the drive mode
       driveSubsystem.setDriveMode(DriveModes.alignLeftReef);
   }

   @Override
   public void execute() {
       // include the actual rotation lock into this
       driveSubsystem.drive(0, 0, commandRot, isFieldRelative.getAsBoolean(), isRateLimited.getAsBoolean());
   }

   @Override
   public void end(boolean interrupted) {
   }

   @Override
   public boolean isFinished() {
       return (driveSubsystem.getDriveMode() != DriveModes.alignLeftReef || Math.abs(commandRot) < 0.04);
   }
}
