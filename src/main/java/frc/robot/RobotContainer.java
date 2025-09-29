// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.MechanismMode;
import frc.robot.Constants.MechanismPosition;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.commands.ForceMechanismCommand;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.CalibrateElevator;
import frc.robot.commands.harpoon.ShootCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
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

    // drivetrain
    private final Drivetrain m_robotDrive = new Drivetrain();

    // the elevator subsystem, controls intake and scoring positions
    public final Elevator elevator = new Elevator();

    // the claw subsytem, controls intake/outtake and wrist
    private final Harpoon harpoon = new Harpoon();

    // the led subsystems. each strip (left and right) is a separate class
    private final LED ledRight = new LED(0);
    private final LED ledLeft = new LED(1);

    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);
    //CommandXboxController operatorController_XBOX = new CommandXboxController(OIConstants.kBackupControllerPort);

    // drop down menu for selecting the auto
    SendableChooser<Command> autoChooser;

    SendableChooser<Boolean> visionActivator;

    private boolean isRedAlliance;

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
        // reseting both mechs to their default state (zeroing the elevator is important)
        // the way this is implemented means that having the elevator up when you deploy code causes problems
        // dont do this ^^^
        harpoon.resetHarpoon();
        elevator.resetElevator();
        
        // defining what alliance we are on
        isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // Setup the commands associated with all buttons on the controller
        // driver
        configureButtonBindingsDriver();
        // operator
        configureButtonBindingsOperator();
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
 
        // for if we're using the xbox controller
        m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftY() * invert * 0.5, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftX() * invert * 0.5, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getRightX() * 0.75, 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));

        // Set the alliance to either red or blue (to invert controls if necessary)
        m_robotDrive.setAlliance(isRedAlliance);
    }

    // configure the button bindings on the driver controller
    private void configureButtonBindingsDriver() {
        // Reset Gyro
        driverController_XBOX.x().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading(); }));

        // // Slow mode command (Left Bumper)
        // driverController_XBOX.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
        // driverController_XBOX.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

        // // Fast mode command (Right Bumper)
        // driverController_XBOX.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
        // driverController_XBOX.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

        this.driverController_XBOX.leftBumper().onTrue(
            new ForceMechanismCommand(ledLeft, ledRight, elevator, MechanismMode.REEF, () -> elevator.getLowerPosition(), 
                harpoon, () -> harpoon.getLowerSetpoint())
        );

        this.driverController_XBOX.rightBumper().onTrue(
            new ForceMechanismCommand(ledLeft, ledRight, elevator, MechanismMode.REEF, () -> elevator.getUpperPosition(), 
                harpoon, () -> harpoon.getLowerSetpoint())
        );

        this.driverController_XBOX.a().onTrue(
            new ForceMechanismCommand(ledLeft, ledRight, elevator, MechanismMode.REEF, () -> MechanismPosition.STOWED.getElevatorSetpoint(), 
                harpoon, () -> MechanismPosition.STOWED.getClawSetpoint())
        );

        this.driverController_XBOX.y().onTrue(
            new ForceMechanismCommand(ledLeft, ledRight, elevator, MechanismMode.FEEDER, () -> MechanismPosition.INTAKE.getElevatorSetpoint(),
            harpoon, () -> MechanismPosition.INTAKE.getClawSetpoint())
        );

        this.driverController_XBOX.povLeft().onTrue(new ForceMechanismCommand(
            ledRight, ledLeft, 
            elevator, MechanismMode.REEF, () -> MechanismPosition.EMERGENCY.getElevatorSetpoint(), 
            harpoon, () -> MechanismPosition.EMERGENCY.getClawSetpoint()));

        // spit out gamepiece
         this.driverController_XBOX.b()
         .onTrue(new ShootCommand(harpoon,0.6))
         .onFalse(new StopClawCommand(harpoon));

         this.driverController_XBOX.povDown()
         .onTrue(new CalibrateElevator(elevator, this.driverController_XBOX.povDown()));
    }

    /**
     * configure what commands are called by what buttons for the OPERATOR CONTROLLER
     * @param isRedAlliance is the robot on the RED OR BLUE SIDE
     * @param isXbox whether the active controller is the backup (XBOX)
     */
    private void configureButtonBindingsOperator() {  
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
