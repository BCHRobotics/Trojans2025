// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.utils.devices.AutoUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();

    // Driving controller
    CommandXboxController m_mainController = new CommandXboxController(OIConstants.kMainControllerPort);
    CommandXboxController m_backupController = new CommandXboxController(OIConstants.kBackupControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureNamedCommands();
        
        // the input field for typing in the auto
        SmartDashboard.putString("Auto Command", "");
        
        // setting up a dropdown for switching between xbox and playstation
        SendableChooser<String> controllerOptions = new SendableChooser<String>();
        controllerOptions.addOption("Xbox Controller", "XBOX");
        controllerOptions.addOption("Playstation Controller", "PS");
        SmartDashboard.putData("Controller Select", controllerOptions);
    }

    // Sets up the drivetrain for teleoperated driving
    public void configureDriveMode(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;

        CommandXboxController controller = SmartDashboard.getData("Controller Select").toString() == "XBOX" ? m_mainController : m_backupController;
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(controller.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(controller.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(controller.getRightX(), 0.05), // getLeftTriggerAxis()
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        
        // Setup the commands associated with all buttons on the controller
        // Driver controller
        configureButtonBindingsDriver(isRedAlliance, controller);

        // Set the alliance to either red or blue (to invert controls if necessary)
        m_robotDrive.setAlliance(isRedAlliance);
    }

    /**
     * Method for configuring named commands 
     * (used during autos)
     */
    public void configureNamedCommands() {
    }

    /**
     * Binding for driver xbox controller buttons
     * NOTE - Things are configured for the SHSM event, vision is (ofc) not being used for this
     */
    private void configureButtonBindingsDriver(boolean isRedAlliance, CommandXboxController controller) {
        // NOTE FOR SLOW/FAST MODE COMMANDS
        // These commands don't have requirements else they interrupt the drive command (TeleopDriveCommand)

        // Slow mode command (Left Bumper)
        controller.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
        controller.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

        // Fast mode command (Right Bumper)
        controller.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
        controller.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));
        
        // Reset Gyro
        controller.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     * @throws ParseException 
     * @throws IOException 
     * @throws FileVersionException 
     */ 
    public Command getAutonomousCommand() throws FileVersionException, IOException, ParseException {
        // using the string provided by the user to build and run an auto
        return AutoUtils.BuildAutoFromCommands(AutoUtils.SeparateCommandString(SmartDashboard.getString("Auto Command", "")), m_robotDrive);
    }

    /**
     * Initializes the LEDs
     */
    public void initLEDs() {
    }
}
