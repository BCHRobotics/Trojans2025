// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.PivotHarpoon;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.vision.AlignTeleopCommand;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
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
    private final Elevator elevator = new Elevator();
    private final Harpoon harpoon = new Harpoon();

    // Driving controller
    CommandPS5Controller m_mainController = new CommandPS5Controller(OIConstants.kMainControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kBackupControllerPort);

    SendableChooser<String> controllerOptions;

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
        m_cameras.setDriveSubsystem(m_robotDrive);
        
        configureNamedCommands();
        
        // setting up a dropdown for switching between xbox and playstation
        controllerOptions = new SendableChooser<String>();
        controllerOptions.addOption("Xbox Controller", "XBOX");
        controllerOptions.addOption("Playstation Controller", "PS");
        SmartDashboard.putData("Controller Select", controllerOptions);
    }

    /**
     * Set up the joystick controls for the main and backup controller, called on teleopInit()
     * @param isRedAlliance is the robot on the RED SIDE OR BLUE SIDE, used for inverting controls
     */
    public void configureDriveMode(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;

        String controller = controllerOptions.getSelected();
        
        // If no other command is running on the drivetrain, then this manual driving command (driving via controller) is used
        if (controller == "XBOX") {
            m_robotDrive.setDefaultCommand(new TeleopDriveCommand(
            () -> -MathUtil.applyDeadband(m_operatorController.getLeftY() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_operatorController.getLeftX() * invert, 0.05),
            () -> -MathUtil.applyDeadband(m_operatorController.getRightX(), 0.05),
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

    public void initializeElevator() {
        //elevator.resetElevator();
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

    /**
     * configure what commands are called by what buttons (on both controllers)
     * @param isRedAlliance is the robot on the RED OR BLUE SIDE
     * @param useBackup whether the active controller is the backup (XBOX)
     */
    private void configureButtonBindingsDriver(boolean isRedAlliance, boolean useBackup) {
        //final double invert = isRedAlliance ? -1 : 1;

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
        }
        else {
            // Reset Gyro
            m_operatorController.y().onTrue(new InstantCommand(() -> { m_robotDrive.zeroHeading(); m_robotDrive.resetOdometry(m_cameras.estimateRobotPoseManual()); }));

            // Slow mode command (Left Bumper)
            m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)));
            m_operatorController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

            // Fast mode command (Right Bumper)
            m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true)));
            m_operatorController.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false)));

            m_operatorController.leftTrigger().whileFalse(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

            m_operatorController.x().onTrue(
                new InstantCommand(() -> {
                    if(m_cameras.isVisionActive) {
                        new AlignTeleopCommand(
                            18,  // m_cameras.getClosestTagId()
                            true, 
                            true, 
                            m_robotDrive, 
                            m_cameras, 
                            new Translation2d(1, 0)
                            ).schedule();
                    }
                })
            );
            /*
             this.m_operatorController.a() 
             .onTrue(new MoveElevatorCommand(elevator, ElevatorPosition.L1));
             this.m_operatorController.x()
             .onTrue(new MoveElevatorCommand(elevator, ElevatorPosition.L2));
             this.m_operatorController.b()
             .onTrue(new MoveElevatorCommand(elevator, ElevatorPosition.L3));
            this.m_operatorController.y().onTrue(this.elevator.emergencyStop()); // E-stop elevator
                 */
             //this.m_operatorController.povUp()
             //.onTrue(new ScoreCommand(harpoon, 0));

            this.m_operatorController.a() // rotates claw to 100 degrees 
            .onTrue(new PivotHarpoon(this.harpoon,Constants.HarpoonConstants.HarpoonPosition.HOME.getSetpoint()));

             this.m_operatorController.b() // rotates claw to 120 degrees
             .onTrue(new PivotHarpoon(this.harpoon,Constants.HarpoonConstants.HarpoonPosition.L2.getSetpoint()));

             this.m_operatorController.x() // rotates claw to 120 degrees
             .onTrue(new PivotHarpoon(this.harpoon,Constants.HarpoonConstants.HarpoonPosition.L3.getSetpoint()));

             // emergency brake for harpoon for testing
             this.m_operatorController.y()
             .onTrue(this.harpoon.emergencyStop()); 
            
             // intaking the claw
             //this.m_operatorController.a() 
             //.onTrue(new IntakeCommand(this.harpoon));
             // shooting the claw
             //this.m_operatorController.x()
             //.onTrue(new ShootCommand(this.harpoon));
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

    /**
     * [UNUSED]
     * Initializes the LEDs
     */
    public void initLEDs() {
    }
}
