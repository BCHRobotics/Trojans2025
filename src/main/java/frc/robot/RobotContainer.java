// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.vision.AlignTeleopCommand;
import frc.robot.commands.drive.HeadingLockDriveCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.utils.AutoUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
    private final Cameras m_cameras = new Cameras();

    // Driving controller
    CommandPS5Controller m_mainController = new CommandPS5Controller(OIConstants.kMainControllerPort);
    CommandXboxController m_backupController = new CommandXboxController(OIConstants.kBackupControllerPort);

    SendableChooser<String> controllerOptions;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_cameras.setDriveSubsystem(m_robotDrive);

        configureNamedCommands();
        
        // the input field for typing in the auto
        SmartDashboard.putString("Auto Command", "");
        
        // setting up a dropdown for switching between xbox and playstation
        controllerOptions = new SendableChooser<String>();
        controllerOptions.addOption("Xbox Controller", "XBOX");
        controllerOptions.addOption("Playstation Controller", "PS");
        SmartDashboard.putData("Controller Select", controllerOptions);
    }

    // Sets up the drivetrain for teleoperated driving
    public void configureDriveMode(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;

        String controller = controllerOptions.getSelected();
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        if (controller == "XBOX") {
            m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(m_backupController.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_backupController.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_backupController.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        }
        else {
            m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(m_mainController.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_mainController.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_mainController.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        }
        
        // Setup the commands associated with all buttons on the controller
        // Driver controller
        configureButtonBindingsDriver(isRedAlliance, controller == "XBOX");

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
    private void configureButtonBindingsDriver(boolean isRedAlliance, boolean useBackup) {
        final double invert = isRedAlliance ? -1 : 1;

        if (!useBackup) {
            // NOTE FOR SLOW/FAST MODE COMMANDS
            // These commands don't have requirements else they interrupt the drive command (TeleopDriveCommand)

            // Slow mode command (Left Bumper)
            m_mainController.L1().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            m_mainController.L1().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            m_mainController.R1().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            m_mainController.R1().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));
            
            // Reset Gyro
            m_mainController.triangle().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

            m_mainController.square().onTrue(
                new InstantCommand(() -> {
                    if(m_cameras.isVisionActive) {
                        new AlignTeleopCommand(
                            18, 
                            true, 
                            true, 
                            m_robotDrive, 
                            m_cameras, 
                            new Translation2d(0.3, 0)
                            ).schedule();
                        }
                    })
                    );

            m_mainController.circle().onTrue(new HeadingLockDriveCommand(
                () -> -MathUtil.applyDeadband(m_mainController.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_mainController.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_mainController.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive
            ));
        }
        else {
            // Reset Gyro
            m_backupController.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

            m_backupController.x().onTrue(
                new InstantCommand(() -> {
                    if(m_cameras.isVisionActive) {
                        new AlignTeleopCommand(
                            18, 
                            true, 
                            true, 
                            m_robotDrive, 
                            m_cameras, 
                            new Translation2d(0.3, 0)
                            ).schedule();
                        }
                    })
                    );

            m_backupController.b().onTrue(new HeadingLockDriveCommand(
                () -> -MathUtil.applyDeadband(m_backupController.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_backupController.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_backupController.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive
            ));

            // Slow mode command (Left Bumper)
            m_backupController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            m_backupController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            m_backupController.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            m_backupController.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            m_backupController.leftTrigger().whileFalse(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
        }
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
        return AutoUtils.actuallyBuildAutoFromCommands(
            "", m_robotDrive, m_cameras, 0
        );
    }

    /**
     * Initializes the LEDs
     */
    public void initLEDs() {
    }
}
