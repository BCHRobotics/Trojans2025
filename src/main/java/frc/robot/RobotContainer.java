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
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.DriveModes;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.vision.AlignTeleopCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
    private final PhotonVisionPoseV2 poseEstimator = new PhotonVisionPoseV2(m_robotDrive);

    // Driving controller, one for xbox and one for ps5
    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);

    // operator controller, one for xbox and one for ps5

    // drop down menu for selecting xbox/ps5 for the driver controller
    SendableChooser<String> controllerOptions_driver;

    SendableChooser<Command> autoChooser;

    private boolean isRedAlliance;

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
        // setting up a dropdown for switching between xbox and playstation
        // FOR DRIVER
        controllerOptions_driver = new SendableChooser<String>();
        controllerOptions_driver.addOption("Xbox", "XBOX");
        controllerOptions_driver.addOption("Playstation", "PS");
        SmartDashboard.putData("Driver Select", controllerOptions_driver);

        // setting up a dropdown for switching between xbox and playstation
        // FOR OPERATOR

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

//STOW ISSUE MAY BE DUE TO INSTANTCOMMAND. Examine the console output during autonomous to see if your debug messages are appearing.
//If still not working, there might be an issue with how PathPlanner is handling event markers in your setup. In that case, you might want to consider using the command in the auto sequence directly rather than as an event marker.

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Name", autoChooser);
        
        // defining what alliance we are on
        isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // Setup the commands associated with all buttons on the controller
        // driver
        configureButtonBindingsDriver(controllerOptions_driver.getSelected() == "XBOX");
    }

    /**
     * Set up the joystick controls for the main and backup controller, called on teleopInit()
     * @param isRedAlliance is the robot on the RED SIDE OR BLUE SIDE, used for inverting controls
     */
    public void configureDriveMode() {
        isRedAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

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
            //                 // .alongWith( new ToggleMechanismCommand(ledLeft, ledRight, elevator, harpoon))
            //                 .schedule();
            //         }
            //     })
            // );
        }
        else {
            // Reset Gyro
            driverController_XBOX.y().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading(); }));

            this.driverController_XBOX.rightTrigger()
            .onTrue(new AlignTeleopCommand(m_robotDrive, poseEstimator, this.driverController_XBOX.povUp(), "Right"));
        }
    }

    /**
     * configure what commands are called by what buttons for the OPERATOR CONTROLLER
     * @param isRedAlliance is the robot on the RED OR BLUE SIDE
     * @param isXbox whether the active controller is the backup (XBOX)
     */
    private void configureButtonBindingsOperator(boolean isXbox) {
        //final double invert = isRedAlliance ? -1 : 1;

    }

    public boolean areJoysticksPressed() {
        if (controllerOptions_driver.getSelected() == "XBOX") {
            return Math.abs(driverController_XBOX.getLeftX()) > 0.05 ||
            Math.abs(driverController_XBOX.getLeftY()) > 0.05 || 
            Math.abs(driverController_XBOX.getRightX()) > 0.05;
        }
        else {
            return false;
        }
    }

    public void resetAuto() {
        m_robotDrive.setDriveMode(DriveModes.MANUAL);
        m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
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
