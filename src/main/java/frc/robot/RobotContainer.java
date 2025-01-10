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

    // Flightstick controller
    //CommandJoystick m_driverFlightstickController = new CommandJoystick(OIConstants.kFlightstickPort);
    // Driving controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDrivingControllerXBoxPort);
    // Operator controller

    // The auto chooser
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureNamedCommands();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Sets up the drivetrain for teleoperated driving
    public void configureDriveMode(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        
        // Setup the commands associated with all buttons on the controller
        // Driver controller
        configureButtonBindingsDriver(isRedAlliance);

        // Set the alliance to either red or blue (to invert controls if necessary)
        m_robotDrive.setAlliance(isRedAlliance);
    }

    /**
     * Method for configuring named commands 
     * (used during autos)
     */
    public void configureNamedCommands() {
        // Enabling and disabling note vision in auto
    }

    /**
     * Binding for driver xbox controller buttons
     * 
     * NOTE - Things are configured for the SHSM event, vision is (ofc) not being used for this
     */
    private void configureButtonBindingsDriver(boolean isRedAlliance) {
        // NOTE FOR SLOW/FAST MODE COMMANDS
        // These commands don't have requirements else they interrupt the drive command (TeleopDriveCommand)

        // Slow mode command (Left Bumper)
        this.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
        this.m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

        // Fast mode command (Right Bumper)
        this.m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
        this.m_driverController.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));
        
        // Reset Gyro
        this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
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
        //return new PathPlannerAuto("Example Auto");

        return AutoUtils.BuildAutoFromCommands(AutoUtils.SeparateCommandString("1,2,3"), m_robotDrive);
    }

    /**
     * Initializes the LEDs
     */
    public void initLEDs() {
    }
}
