// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.Constants.HarpoonConstants.HarpoonPosition;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.CalibrateElevator;
import frc.robot.commands.elevator.PrepareElevatorCommand;
import frc.robot.commands.elevator.StowElevatorCommand;
import frc.robot.commands.elevator.ToggleElevatorCommand;
import frc.robot.commands.harpoon.AutoScoreCommand;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.PrepareHarpoonCommand;
import frc.robot.commands.harpoon.ShootCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.utils.AutoUtils;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    public final Elevator elevator = new Elevator();
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
        
        // setting up a dropdown for switching between xbox and playstation
        // FOR DRIVER
        controllerOptions_driver = new SendableChooser<String>();
        controllerOptions_driver.addOption("Xbox", "XBOX");
        controllerOptions_driver.addOption("Playstation", "PS");
        SmartDashboard.putData("Driver Select", controllerOptions_driver);

        // setting up a dropdown for switching between xbox and playstation
        // FOR OPERATOR
        controllerOptions_operator = new SendableChooser<String>();
        controllerOptions_operator.addOption("Xbox", "XBOX");
        controllerOptions_operator.addOption("Playstation", "PS");
        SmartDashboard.putData("Operator Select", controllerOptions_operator);

        new EventTrigger("Elevator Up").onTrue(new ToggleElevatorCommand(elevator, harpoon));
        new EventTrigger("Pose Estimation").onTrue(
            new InstantCommand(() -> {
                m_robotDrive.setOdometryOffset(m_cameras.getPoseEstimatedOffset());
            }, new Subsystem[0])
        );

        new EventTrigger("Score").onTrue(new WaitCommand(2).andThen(new AutoScoreCommand(harpoon, 0.6)));
        new EventTrigger("Stop Intake").onTrue(new StopClawCommand(harpoon));

        harpoon.resetHarpoon();
        elevator.resetElevator();
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

    private void configureButtonBindingsDriver(boolean isRedAlliance, boolean isXbox) {
        if (isXbox) {
            // Reset Gyro
            driverController_XBOX.y().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading();})); //  m_robotDrive.resetOdometry(m_cameras.estimateRobotPoseManual(false));

            // Slow mode command (Left Bumper)
            driverController_XBOX.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            driverController_XBOX.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            driverController_XBOX.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            driverController_XBOX.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            // toggling the elevator up and down
            driverController_XBOX.a().onTrue(
                new ToggleElevatorCommand(elevator, harpoon));

            // intake gamepiece
            this.driverController_XBOX.x()
            .onTrue(new IntakeCommand(elevator, harpoon,0.6, driverController_XBOX.x())
            );

            // spit out gamepiece
             this.driverController_XBOX.b()
             .onTrue(new ShootCommand(harpoon,0.6))
             .onFalse(new StopClawCommand(harpoon));

             this.driverController_XBOX.povDown()
             .onTrue(new CalibrateElevator(elevator, this.driverController_XBOX.povDown()));
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
            driverController_PS5.cross().onTrue(new ToggleElevatorCommand(elevator, harpoon));

            // intake gamepiece
            this.driverController_PS5.square()
            .onTrue(new IntakeCommand(elevator, harpoon,0.6, driverController_XBOX.x())
            );

            // spit out gamepiece
             this.driverController_PS5.circle()
             .onTrue(new ShootCommand(harpoon,0.6))
             .onFalse(new StopClawCommand(harpoon));

            //  this.driverController_XBOX.povDown()
            //  .onTrue(new CalibrateElevator(elevator, this.driverController_XBOX.povDown()));
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
                new PrepareElevatorCommand(elevator, ElevatorMode.REEF, () -> elevator.getLowerPosition()).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonMode.REEF, () -> harpoon.getLowerSetpoint()))
            );

            this.operatorController_XBOX.rightBumper().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.REEF, () -> elevator.getUpperPosition()).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonMode.REEF, () -> harpoon.getUpperSetpoint()))
            );

            this.operatorController_XBOX.leftTrigger().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.FEEDER, () -> ElevatorPosition.INTAKE.getSetpoint()).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonMode.FEEDER, () -> HarpoonPosition.INTAKE.getSetpoint()))
            );
        }
        else {
        }
    }

    public void resetAuto() {
        m_robotDrive.setDriveMode(DriveModes.MANUAL);
        m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        harpoon.resetHarpoon();
        elevator.resetElevator();
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
        //return AutoUtils.actuallyBuildAutoFromCommands("move(BlueReef4Left)/wait(5)", m_robotDrive, m_cameras, 0);

        return Commands.none();
    }
}
