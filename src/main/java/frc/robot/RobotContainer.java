// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.HarpoonConstants.HarpoonPosition;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.PrepareElevatorCommand;
import frc.robot.commands.elevator.ToggleElevatorCommand;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.PrepareHarpoonCommand;
import frc.robot.commands.harpoon.ShootCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private final Elevator elevator = new Elevator();
    private final Harpoon harpoon = new Harpoon();
    private final LED ledRight = new LED(0);
    private final LED ledLeft = new LED(1);

    // Driving controller
    CommandPS5Controller driverController_PS5 = new CommandPS5Controller(OIConstants.kMainControllerPort);
    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);

    // operator controller
    CommandPS5Controller operatorController_PS5 = new CommandPS5Controller(OIConstants.kBackupControllerPort);
    CommandXboxController operatorController_XBOX = new CommandXboxController(OIConstants.kBackupControllerPort);

    // selecting a driver controller
    SendableChooser<String> controllerOptions_driver;
    // selecting an operator controller
    SendableChooser<String> controllerOptions_operator;

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
        m_cameras.setDriveSubsystem(m_robotDrive);
        
        configureNamedCommands();
        
        // setting up a dropdown for switching between xbox and playstation
        // FOR DRIVER
        controllerOptions_driver = new SendableChooser<String>();
        controllerOptions_driver.addOption("Xbox", "XBOX");
        controllerOptions_driver.addOption("Playstation", "PS");
        SmartDashboard.putData("Controller Select", controllerOptions_driver);

        // setting up a dropdown for switching between xbox and playstation
        // FOR OPERATOR
        controllerOptions_operator = new SendableChooser<String>();
        controllerOptions_operator.addOption("Xbox", "XBOX");
        controllerOptions_operator.addOption("Playstation", "PS");
        SmartDashboard.putData("Controller Select", controllerOptions_driver);
    }

    /**
     * Set up the joystick controls for the main and backup controller, called on teleopInit()
     * @param isRedAlliance is the robot on the RED SIDE OR BLUE SIDE, used for inverting controls
     */
    public void configureDriveMode(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;

        String driverController = controllerOptions_driver.getSelected();
        String operatorController = controllerOptions_operator.getSelected();
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        if (driverController == "XBOX") {
            m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        }
        else {
            m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(driverController_PS5.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_PS5.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_PS5.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        }
        
        // Setup the commands associated with all buttons on the controller
        // driver
        configureButtonBindingsDriver(isRedAlliance, driverController == "XBOX");
        // operator
        configureButtonBindingsOperator(isRedAlliance, operatorController == "XBOX");

        // Set the alliance to either red or blue (to invert controls if necessary)
        m_robotDrive.setAlliance(isRedAlliance);
    }

    /**
     * [UNUSED]
     * Method for configuring named commands 
     * (used during autos)
     */
    public void configureNamedCommands() {
        //NamedCommands.registerCommand("Elevator L1", new MoveElevatorCommand(elevator, ElevatorPosition.MID));
        //NamedCommands.registerCommand("Harpoon L1", new ScoreCommand(harpoon, HarpoonPosition.TOP.getSetpoint()));
    }

    private void configureButtonBindingsDriver(boolean isRedAlliance, boolean isXbox) {
        if (isXbox) {
            // Reset Gyro
            driverController_XBOX.y().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading();}));

            // Slow mode command (Left Bumper)
            driverController_XBOX.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            driverController_XBOX.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            driverController_XBOX.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            driverController_XBOX.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            // toggling the elevator up and down
            driverController_PS5.square().onTrue(new ToggleElevatorCommand(elevator, harpoon));
        }
        else {
            // Reset Gyro
            driverController_PS5.triangle().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading();}));

            // Slow mode command (Left Bumper)
            driverController_PS5.L1().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            driverController_PS5.L1().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            driverController_PS5.R1().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            driverController_PS5.R1().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            // toggling the elevator up and down
            driverController_XBOX.a().onTrue(new ToggleElevatorCommand(elevator, harpoon));

            // intake gamepiece
            this.operatorController_XBOX.x()
            .onTrue(new IntakeCommand(harpoon,0.6));

            // spit out gamepiece
             this.driverController_XBOX.b()
             .onTrue(new ShootCommand(harpoon,0.6))
             .onFalse(new StopClawCommand(harpoon));
        }
    }

    /**
     * configure what commands are called by what buttons (on both controllers)
     * @param isRedAlliance is the robot on the RED OR BLUE SIDE
     * @param useBackup whether the active controller is the backup (XBOX)
     */
    private void configureButtonBindingsOperator(boolean isRedAlliance, boolean isXbox) {
        //final double invert = isRedAlliance ? -1 : 1;

        if (isXbox) {
            // NOTE FOR SLOW/FAST MODE COMMANDS
            // These commands don't have requirements else they interrupt the drive command (TeleopDriveCommand)

            this.operatorController_XBOX.leftBumper().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.REEF, elevator.getUpperPosition()).
                andThen(new PrepareHarpoonCommand(harpoon, harpoon.getUpperSetpoint()))
            );

            this.operatorController_XBOX.rightBumper().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.REEF, elevator.getLowerPosition()).
                andThen(new PrepareHarpoonCommand(harpoon, harpoon.getLowerSetpoint()))
            );

            this.operatorController_XBOX.leftTrigger().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.FEEDER, ElevatorPosition.INTAKE).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonPosition.INTAKE.getSetpoint()))
            );
        }
        else {
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     * @throws ParseException 
     * @throws IOException 
     * @throws FileVersionException 
     */ 
    public Command getAutonomousCommand() throws FileVersionException, IOException, ParseException {
        //using the string provided by the user to build and run an auto
        return AutoBuilder.buildAuto("Test Auto");
    }
}
