// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MechanismMode;
import frc.robot.Constants.MechanismPosition;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.commands.ForceMechanismCommand;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.drive.FollowPointsCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.CalibrateElevator;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.ShootCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    // drivetrain
    private final Drivetrain m_robotDrive = new Drivetrain();

    // the elevator subsystem, controls intake and scoring positions
    public final Elevator elevator = new Elevator();

    // the claw subsytem, controls intake/outtake and wrist
    private final Harpoon harpoon = new Harpoon();

    // the led subsystems. each strip (left and right) is a separate class
    private final LED ledRight = new LED(0);
    private final LED ledLeft = new LED(1);

    // no vision for now
    //private final Vision visionSystem = new Vision();

    // Driving controller, one for xbox and one for ps5
    CommandPS5Controller driverController_PS5 = new CommandPS5Controller(OIConstants.kMainControllerPort);
    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);

    // drop down menu for selecting xbox/ps5 for the driver controller
    SendableChooser<Integer> controllerOptions_driver; 
    // drop down menu for selecting the auto
    SendableChooser<Command> autoChooser;

    SendableChooser<Boolean> visionActivator;

    private boolean isRedAlliance;

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
        // setting up a dropdown for switching between xbox and playstation
        // FOR DRIVER
        controllerOptions_driver = new SendableChooser<Integer>();
        controllerOptions_driver.addOption("Xbox", 0);
        controllerOptions_driver.addOption("Playstation", 1);
        controllerOptions_driver.setDefaultOption("default", 1);
        SmartDashboard.putData("Driver Select", controllerOptions_driver);

        visionActivator = new SendableChooser<Boolean>();
        visionActivator.addOption("Yes", true);
        visionActivator.addOption("No", false);
        visionActivator.setDefaultOption("default", true);
        SmartDashboard.putData("Use vision in auto?", visionActivator);

        // we want to set up the named commands before the auto chooser, to avoid weird errors
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Name", autoChooser);
        
        // reseting both mechs to their default state (zeroing the elevator is important)
        // the way this is implemented means that having the elevator up when you deploy code causes problems
        // dont do this ^^^
        harpoon.resetHarpoon();
        elevator.resetElevator();
        
        // defining what alliance we are on
        isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // Setup the commands associated with all buttons on the controller
        // driver
        configureButtonBindingsDriver(controllerOptions_driver.getSelected() == 0);
    }

    // setting up the named commands for AutoBuilder
    void configureNamedCommands() {
        // these six commands are for positioning the mechanism
        // the mechanism will be moved directly via event markers

        // move elevator/claw back to stowed
        NamedCommands.registerCommand("STOW", new ForceMechanismCommand(
            ledRight, ledLeft, 
            elevator, MechanismMode.STOWED, () -> MechanismPosition.STOWED.getElevatorSetpoint(), 
            harpoon, () -> MechanismPosition.STOWED.getClawSetpoint()));

        // move elevator/claw to L1 for scoring
        NamedCommands.registerCommand("L1", new ForceMechanismCommand(
            ledRight, ledLeft, 
            elevator, MechanismMode.REEF, () -> MechanismPosition.L1.getElevatorSetpoint(), 
            harpoon, () -> MechanismPosition.L1.getClawSetpoint()));

        // move elevator/claw to L2 for scoring
        NamedCommands.registerCommand("L2", new ForceMechanismCommand(
            ledRight, ledLeft, 
            elevator, MechanismMode.REEF, () -> MechanismPosition.L2.getElevatorSetpoint(), 
            harpoon, () -> MechanismPosition.L2.getClawSetpoint()));

        // move elevator/claw to L3 for scoring
        NamedCommands.registerCommand("L3", new ForceMechanismCommand(
            ledRight, ledLeft, 
            elevator, MechanismMode.REEF, () -> MechanismPosition.L3.getElevatorSetpoint(), 
            harpoon, () -> MechanismPosition.L3.getClawSetpoint()));

        // move elevator/claw to L4 for scoring
        NamedCommands.registerCommand("L4", new ForceMechanismCommand(
            ledRight, ledLeft, 
            elevator, MechanismMode.REEF, () -> MechanismPosition.L4.getElevatorSetpoint(), 
            harpoon, () -> MechanismPosition.L4.getClawSetpoint()));

        // move elevator/claw to the intake position, for, well, intaking
        NamedCommands.registerCommand("INTAKE", new ForceMechanismCommand(
            ledRight, ledLeft, 
            elevator, MechanismMode.FEEDER, () -> MechanismPosition.INTAKE.getElevatorSetpoint(), 
            harpoon, () -> MechanismPosition.INTAKE.getClawSetpoint()));

        // these two are for the claw

        // move elevator/claw to the intake position, for, well, intaking
        NamedCommands.registerCommand("SHOOT", new ShootCommand(harpoon, 0.6));

        // move elevator/claw to the intake position, for, well, intaking
        NamedCommands.registerCommand("STOP CLAW", new StopClawCommand(harpoon));
    }

    /**
     * Set up the joystick controls for the main and backup controller, called on teleopInit()
     * @param isRedAlliance is the robot on the RED SIDE OR BLUE SIDE, used for inverting controls
     */
    public void configureDriveMode() {
        isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // set LEDs to blue, the idle color
        new SetLEDCommand(ledLeft, ledRight, 0.87, 0).schedule();

        // this double is used as a multiplier to invert the joysticks for red alliance
        final double invert = isRedAlliance ? -1 : 1;

        // making sure the controller variables are set properly
        Integer driverController = controllerOptions_driver.getSelected();
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        if (driverController == 0) {
            // for if we're using the xbox controller
            m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        }
        else {
            // for if we're using the ps5 controller
            m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(driverController_PS5.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_PS5.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_PS5.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));
        }

        // Set the alliance to either red or blue (to invert controls if necessary)
        m_robotDrive.setAlliance(isRedAlliance);
    }

    // configure the button bindings on the driver controller
    private void configureButtonBindingsDriver(boolean isXbox) {
        if (isXbox) {
            // Reset Gyro
            driverController_XBOX.y().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading(); }));

            // Slow mode command (Left Bumper)
            driverController_XBOX.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            driverController_XBOX.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            driverController_XBOX.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            driverController_XBOX.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            // toggling the elevator up and down
            driverController_XBOX.a().onTrue(new InstantCommand());

            // intake gamepiece
            this.driverController_XBOX.x()
            .onTrue(new IntakeCommand(elevator, harpoon,0.6, this.driverController_XBOX.x(), true)
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
            driverController_PS5.triangle().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading(); }));

            // Fast mode
            driverController_PS5.L1().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));

            // Fast mode
            driverController_PS5.L1().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode
            driverController_PS5.R1().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));

            // Fast mode
            driverController_PS5.R1().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            // // intake gamepiece
            // this.driverController_PS5.cross()
            // .onTrue(new ToggleMechanismCommand(ledRight, ledLeft, elevator, harpoon));

            // // intake gamepiece
            // this.driverController_PS5.square()
            // .onTrue(new IntakeCommand(elevator, harpoon,0.6, this.driverController_PS5.square(), true)
            // );

            // // spit out gamepiece
            // this.driverController_PS5.circle()
            // .onTrue(new ShootCommand(harpoon, 0.7))
            // .onFalse(new StopClawCommand(harpoon));

            this.driverController_PS5.povDown().onTrue(new FollowPointsCommand(() -> OIConstants.kRateLimited,m_robotDrive));
        }
    }

    public boolean areJoysticksPressed() {
        return Math.abs(driverController_PS5.getLeftX()) > 0.25 ||
            Math.abs(driverController_PS5.getLeftY()) > 0.25 || 
            Math.abs(driverController_PS5.getRightX()) > 0.25;
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
    public Command getAutonomousCommand() {
        if (autoChooser.getSelected() != null) {
            // TODO: modify the auto to start at wherever the pose has been estimated
            
            // this is hardcoded for now to bypass NT/shuffleboard trouble
            return autoChooser.getSelected();
        }
        else {
            return Commands.none();
        }
    }
}
