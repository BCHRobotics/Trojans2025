// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static boolean isRed;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.initLEDs();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_elevator.cancelElevatorCommands();
  }

  /** This function is called periodically during Disabled mode. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    try {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("command start");
      m_autonomousCommand.andThen(() -> System.out.println("command finished")).schedule();
    }

    final boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;



    // Schedules the teleop drive command when entering teleop
    m_robotContainer.configureDriveMode(isRed);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    final boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    // Schedules the teleop drive command when entering teleop
    m_robotContainer.configureDriveMode(isRed);
    //m_robotContainer.m_elevator.calibrate(); // calibrates the elevator
    //new InstantCommand(()-> m_robotContainer.m_elevator.calibrate());
//  Command calibrationCommand = m_robotContainer.m_elevator.calibrate(); // spinning robot up
//  CommandScheduler.getInstance().schedule(calibrationCommand);
    
    //Command setZeroCommand = m_robotContainer.m_elevator.setEncoderPos(0);
    //CommandScheduler.getInstance().schedule(setZeroCommand);
    Command zeroElevatorCommand = this.m_robotContainer.m_elevator.zeroElevator();
    CommandScheduler.getInstance().cancelAll(); // cancelling all commands first 
    CommandScheduler.getInstance().schedule(zeroElevatorCommand);
 // CANCELLING ALL COMMANDS AT THE START
    //CommandScheduler.getInstance().cancelAll();
    
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
