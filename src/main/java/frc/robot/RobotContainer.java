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
import frc.robot.commands.PrepareMechanismCommand;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.climber.RunClimberCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.CalibrateElevator;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.ShootCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PhotonVisionPoseV2;


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

    //pose Estimator 
    @SuppressWarnings("unused")
    private final PhotonVisionPoseV2 poseEstimator = new PhotonVisionPoseV2(m_robotDrive);

    // the elevator subsystem, controls intake and scoring positions
    public final Elevator elevator = new Elevator();

    // the claw subsytem, controls intake/outtake and wrist
    private final Harpoon harpoon = new Harpoon();

    // the led subsystems. each strip (left and right) is a separate class
    private final LED ledRight = new LED(0);
    private final LED ledLeft = new LED(1);

    private final Climber climber = new Climber();

    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);
    CommandXboxController operatorController_XBOX = new CommandXboxController(OIConstants.kBackupControllerPort);

    // drop down menu for selecting the auto
    SendableChooser<Command> autoChooser;

    SendableChooser<Boolean> visionActivator;

    private boolean isRedAlliance;

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
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
        configureButtonBindingsDriver();
        // operator
        configureButtonBindingsOperator();
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
 
        // for if we're using the xbox controller
        m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(driverController_XBOX.getRightX(), 0.05),
            () -> OIConstants.kFieldRelative, () -> OIConstants.kRateLimited,
            m_robotDrive));

        // Set the alliance to either red or blue (to invert controls if necessary)
        m_robotDrive.setAlliance(isRedAlliance);
    }

    // configure the button bindings on the driver controller
    private void configureButtonBindingsDriver() {
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

    /**
     * configure what commands are called by what buttons for the OPERATOR CONTROLLER
     * @param isRedAlliance is the robot on the RED OR BLUE SIDE
     * @param isXbox whether the active controller is the backup (XBOX)
     */
    private void configureButtonBindingsOperator() {
        //final double invert = isRedAlliance ? -1 : 1;

        // NOTE FOR SLOW/FAST MODE COMMANDS
            // These commands don't have requirements else they interrupt the drive command (TeleopDriveCommand)

            this.operatorController_XBOX.leftBumper().onTrue(
                new PrepareMechanismCommand(elevator, MechanismMode.REEF, () -> elevator.getLowerPosition(),
                harpoon, () -> harpoon.getLowerSetpoint())
            );

            this.operatorController_XBOX.rightBumper().onTrue(
                new PrepareMechanismCommand(elevator, MechanismMode.REEF, () -> elevator.getUpperPosition(),
                harpoon, () -> harpoon.getUpperSetpoint())
            );

            this.operatorController_XBOX.y().onTrue(
                new PrepareMechanismCommand(elevator, MechanismMode.FEEDER, () -> MechanismPosition.INTAKE.getElevatorSetpoint(),
                harpoon, () -> MechanismPosition.INTAKE.getClawSetpoint())
            );

            this.operatorController_XBOX.rightTrigger().onTrue(
                new InstantCommand(() -> poseEstimator.targetSide(false))
            );

            this.operatorController_XBOX.leftTrigger().onTrue(
                new InstantCommand(() -> poseEstimator.targetSide(true))
            );

            this.operatorController_XBOX.povUp().onTrue(new RunClimberCommand(climber, 0.5));
            this.operatorController_XBOX.povUp().onFalse(new RunClimberCommand(climber, 0));

            this.operatorController_XBOX.povDown().onTrue(new RunClimberCommand(climber, -0.5));
            this.operatorController_XBOX.povDown().onFalse(new RunClimberCommand(climber, 0));

            this.operatorController_XBOX.povLeft().onTrue(new ForceMechanismCommand(
                ledRight, ledLeft, 
                elevator, MechanismMode.REEF, () -> MechanismPosition.EMERGENCY.getElevatorSetpoint(), 
                harpoon, () -> MechanismPosition.EMERGENCY.getClawSetpoint()));
    }

    // this function exists to turn pose estimation back on
    public void enterTeleop() {
        poseEstimator.setVisionPoseEnabled(true);
    }

    // serves to disable vision if needed
    public void enterAuto() {
        poseEstimator.setVisionPoseEnabled(visionActivator.getSelected());
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
