// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.elevator.ToggleMechanismCommand;
import frc.robot.commands.harpoon.AutoScoreCommand;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.PrepareHarpoonCommand;
import frc.robot.commands.harpoon.ShootCommand;
import frc.robot.commands.harpoon.StopClawCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PoseEstimatorSubsystem;
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

    // the photon camera
    private final PhotonCamera m_camera = new PhotonCamera("photonvision"); // will change name

    // the pose estimation subsystem, used for vision processing
    private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_camera, m_robotDrive);


    // Driving controller, one for xbox and one for ps5
    CommandPS5Controller driverController_PS5 = new CommandPS5Controller(OIConstants.kMainControllerPort);
    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);

    // operator controller, one for xbox and one for ps5
    CommandPS5Controller operatorController_PS5 = new CommandPS5Controller(OIConstants.kBackupControllerPort);
    CommandXboxController operatorController_XBOX = new CommandXboxController(OIConstants.kBackupControllerPort);

    // drop down menu for selecting xbox/ps5 for the driver controller
    SendableChooser<String> controllerOptions_driver;
    // drop down menu for selecting xbox/ps5 for the operator controller
    SendableChooser<String> controllerOptions_operator;

    // // dropdown menu for selecting autos
    // SendableChooser<String> autoSelect;

    // // dropdown menu for selecting autos
    // SendableChooser<Integer> autoFallback;

    SendableChooser<Command> autoChooser;

    // Command autoCommand;
    // String autoCommandString;

    private boolean isRedAlliance;

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
        m_robotDrive.initializeAuto(m_poseEstimator.getCurrentPoseSupplier());

        
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

        // // defining the auto selection dropdown
        // autoSelect = new SendableChooser<String>();
        // autoSelect.addOption("None", "none");
        // autoSelect.addOption("Move (blue)", "shift(-1.5,0)/wait(1)/score(0)");
        // autoSelect.addOption("Move (red)", "shift(1.5,0)");
        // autoSelect.addOption("1 Coral (10)", "move(Reef3Right)/wait(1)/score(0)");
        // autoSelect.addOption("1 Coral (11)", "move(Reef3Left)/wait(1)/score(0)");
        // autoSelect.addOption("1 Coral (12)", "move(Reef4Right)/wait(1)/score(0)");
        // autoSelect.addOption("1 Coral (1)", "move(Reef4Left)/wait(1)/score(0)");
        // autoSelect.addOption("1 Coral (2)", "move(Reef5Right)/wait(1)/score(0)");
        // autoSelect.addOption("1 Coral (3)", "move(Reef5Left)/wait(1)/score(0)");
        // // TODO: get this one ready
        // // autoSelect.addOption("1 Coral, Feeder", "move(Reef4Left)/wait(1)/score(0)/path(Coral2Left)/intake(0)");

        NamedCommands.registerCommand("SCORE", new AutoScoreCommand(harpoon, 0.6));
        NamedCommands.registerCommand("RAISE", new ToggleMechanismCommand(ledRight, ledLeft, elevator, harpoon));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("uuu", autoChooser);

        //SmartDashboard.putData("Select Auto", autoSelect);

        // // defining the auto FALLBACK POSITION selection dropdown
        // autoFallback = new SendableChooser<Integer>();
        // autoFallback.addOption("1", 0);
        // autoFallback.addOption("2", 1);
        // autoFallback.addOption("3", 2);
        // autoFallback.addOption("4", 3);
        // autoFallback.addOption("5", 4);
        // autoFallback.addOption("6", 5);

        // SmartDashboard.putData("Select Fallback", autoFallback);

        // // defining event markers for auto
        // new EventTrigger("Elevator Up").onTrue(new ToggleMechanismCommand(ledLeft, ledRight, elevator, harpoon));
        // new EventTrigger("Pose Estimation").onTrue(
        //     new InstantCommand(() -> {
        //         m_robotDrive.setOdometryOffset(m_cameras.getPoseEstimatedOffset());
        //         System.out.println("OFFSET HAS BEEN ESTIMATED!");
        //     }, new Subsystem[0])
        // );
        // new EventTrigger("Score").onTrue(new WaitCommand(2).andThen(new AutoScoreCommand(harpoon, 0.6)));
        // new EventTrigger("Stop Intake").onTrue(new StopClawCommand(harpoon));
        
        // reseting both mechs to their default state (zeroing the elevator is important)
        harpoon.resetHarpoon();
        elevator.resetElevator();
        
        // defining what alliance we are on
        isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // Setup the commands associated with all buttons on the controller
        // driver
        configureButtonBindingsDriver(controllerOptions_driver.getSelected() == "XBOX");
        // operator
        configureButtonBindingsOperator(controllerOptions_operator.getSelected() == "XBOX");
    }

    /**
     * Set up the joystick controls for the main and backup controller, called on teleopInit()
     * @param isRedAlliance is the robot on the RED SIDE OR BLUE SIDE, used for inverting controls
     */
    public void configureDriveMode() {
        isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        // 0.87 is blue, 0.67 is gold
        new SetLEDCommand(ledLeft, ledRight, 0.67, 0).schedule();

        // this double is used as a multiplier to invert the joysticks for red alliance
        final double invert = isRedAlliance ? -1 : 1;

        // making sure the controller variables are set properly
        String driverController = controllerOptions_driver.getSelected();
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        if (driverController == "XBOX") {
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
            driverController_XBOX.y().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading(); })); //  m_robotDrive.resetOdometry(m_cameras.estimateRobotPoseManual(false));

            // Slow mode command (Left Bumper)
            driverController_XBOX.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            driverController_XBOX.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            driverController_XBOX.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            driverController_XBOX.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            // toggling the elevator up and down
            driverController_XBOX.a().onTrue(
                new ToggleMechanismCommand(ledLeft, ledRight, elevator, harpoon));

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
            
             // automatic vision lineup (taken out for now)
            //  this.driverController_XBOX.rightTrigger().onTrue(
            //     new InstantCommand(() -> {
            //         if(m_cameras.isVisionActive) {
            //             new AlignTeleopCommand(
            //                 m_cameras.getClosestTagId(),
            //                 true, 
            //                 true, 
            //                 m_robotDrive, 
            //                 m_cameras, 
            //                 () -> m_cameras.getOffsetX(),
            //                 () -> m_cameras.getOffsetY(),
            //                 () -> areJoysticksPressed()
            //                 )
            //                 .alongWith( new ToggleMechanismCommand(ledLeft, ledRight, elevator, harpoon))
            //                 .schedule();
            //         }
            //     })
            // );
        }
        else {
            // Reset Gyro
            driverController_PS5.triangle().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading(); }));

            // Slow mode command (Left Bumper)
            driverController_PS5.L1().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            driverController_PS5.L1().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            driverController_PS5.R1().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            driverController_PS5.R1().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            // toggling the elevator up and down
            driverController_PS5.cross().onTrue(new ToggleMechanismCommand(ledLeft, ledRight, elevator, harpoon));

            // intake gamepiece
            this.driverController_PS5.square()
            .onTrue(new IntakeCommand(elevator, harpoon,0.6, driverController_PS5.square())
            );

            // spit out gamepiece
             this.driverController_PS5.circle()
             .onTrue(new ShootCommand(harpoon,0.7))
             .onFalse(new StopClawCommand(harpoon));

             this.driverController_PS5.povDown()
             .onTrue(new CalibrateElevator(elevator, this.driverController_PS5.povDown()));

             // automatic vision lineup (taken out for now)
             

            
        }
    }

    /**
     * configure what commands are called by what buttons for the OPERATOR CONTROLLER
     * @param isRedAlliance is the robot on the RED OR BLUE SIDE
     * @param isXbox whether the active controller is the backup (XBOX)
     */
    private void configureButtonBindingsOperator(boolean isXbox) {
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

            this.operatorController_XBOX.y().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.FEEDER, () -> ElevatorPosition.INTAKE.getSetpoint()).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonMode.FEEDER, () -> HarpoonPosition.INTAKE.getSetpoint()))
            );

        }
        else {
            this.operatorController_PS5.L1().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.REEF, () -> elevator.getLowerPosition()).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonMode.REEF, () -> harpoon.getLowerSetpoint()))
            );

            this.operatorController_PS5.R1().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.REEF, () -> elevator.getUpperPosition()).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonMode.REEF, () -> harpoon.getUpperSetpoint()))
            );

            this.operatorController_PS5.triangle().onTrue(
                new PrepareElevatorCommand(elevator, ElevatorMode.FEEDER, () -> ElevatorPosition.INTAKE.getSetpoint()).
                andThen(new PrepareHarpoonCommand(harpoon, HarpoonMode.FEEDER, () -> HarpoonPosition.INTAKE.getSetpoint()))
            );

        }
    }

    public boolean areJoysticksPressed() {
        if (controllerOptions_driver.getSelected() == "XBOX") {
            return Math.abs(driverController_XBOX.getLeftX()) > 0.05 ||
            Math.abs(driverController_XBOX.getLeftY()) > 0.05 || 
            Math.abs(driverController_XBOX.getRightX()) > 0.05;
        }
        else {
            return Math.abs(driverController_PS5.getLeftX()) > 0.05 ||
            Math.abs(driverController_PS5.getLeftY()) > 0.05 || 
            Math.abs(driverController_PS5.getRightX()) > 0.05;
        }
    }

    public void resetAuto() {
        m_robotDrive.setDriveMode(DriveModes.MANUAL);
        m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        harpoon.resetHarpoon();
        elevator.resetElevator();
    }

    public void checkAuto() {
        // // defining what alliance we are on
        // isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // if(autoSelect.getSelected() == null || autoFallback.getSelected() == null) {return;}

        // if (autoCommandString != autoSelect.getSelected()) {
        //     autoCommand = AutoUtils.actuallyBuildAutoFromCommands(autoSelect.getSelected(), ledLeft, ledRight, elevator, harpoon, m_robotDrive, m_cameras, autoFallback.getSelected(), isRedAlliance);
        //     autoCommandString = autoSelect.getSelected();

        //     SmartDashboard.putString("LOADED AUTO", autoCommandString);
        // }

        // SmartDashboard.putBoolean("i", isRedAlliance);
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
            return autoChooser.getSelected();
        }
        else {
            return Commands.none();
        }
    }
}
