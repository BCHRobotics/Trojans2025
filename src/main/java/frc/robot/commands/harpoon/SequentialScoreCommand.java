package frc.robot.commands.harpoon;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

/**
 * A sequential command that:
 * 1. Sets the elevator to a specific setpoint
 * 2. Waits until the elevator reaches within 1 inch of the target
 * 3. Then sets the harpoon to a specific setpoint
 * 4. Finally scores the game piece
 */
public class SequentialScoreCommand extends Command {
    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;
    
    private double elevatorSetpoint;
    private double harpoonSetpoint;
    private double scoreSpeed;
    
    private enum CommandState {
        MOVING_ELEVATOR,
        MOVING_HARPOON,
        SCORING,
        FINISHED
    }
    
    private CommandState currentState = CommandState.MOVING_ELEVATOR;
    
    /**
     * Creates a new SequentialScoreCommand.
     * 
     * @param elevatorSubsystem The elevator subsystem
     * @param harpoonSubsystem The harpoon subsystem
     * @param elevatorSetpoint The setpoint to move the elevator to
     * @param harpoonSetpoint The setpoint to move the harpoon to
     * @param scoreSpeed The speed at which to score
     */
    public SequentialScoreCommand(Elevator elevatorSubsystem, Harpoon harpoonSubsystem, 
                                 double elevatorSetpoint, double harpoonSetpoint, double scoreSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.harpoonSubsystem = harpoonSubsystem;
        this.elevatorSetpoint = elevatorSetpoint;
        this.harpoonSetpoint = harpoonSetpoint;
        this.scoreSpeed = scoreSpeed;
        
        // Add subsystems as requirements
        addRequirements(elevatorSubsystem, harpoonSubsystem);
    }
    
    @Override
    public void initialize() {
        // Start by moving the elevator
        System.out.println("SEQUENTIAL SCORE: Moving elevator to " + elevatorSetpoint);
        elevatorSubsystem.setSetpoint(elevatorSetpoint);
        currentState = CommandState.MOVING_ELEVATOR;
    }
    
    @Override
    public void execute() {
        switch (currentState) {
            case MOVING_ELEVATOR:
                // Check if the elevator is within 1 inch of the target
                // The getEncoderPosition method already converts to the correct units
                double elevatorPosition = elevatorSubsystem.getEncoderPosition();
                if (Math.abs(elevatorPosition - elevatorSetpoint) < 1.0) {
                    // Elevator is within tolerance, now move the harpoon
                    System.out.println("SEQUENTIAL SCORE: Elevator in position, moving harpoon to " + harpoonSetpoint);
                    harpoonSubsystem.setRotationMotorPosition(harpoonSetpoint);
                    currentState = CommandState.MOVING_HARPOON;
                }
                break;
                
            case MOVING_HARPOON:
                // Check if the harpoon is in position
                double harpoonPosition = harpoonSubsystem.kRotationMotor.getAbsoluteEncoder().getPosition();
                if (Math.abs(harpoonPosition - harpoonSetpoint) < 0.1) {
                    // Harpoon is in position, now score
                    System.out.println("SEQUENTIAL SCORE: Harpoon in position, scoring at speed " + scoreSpeed);
                    harpoonSubsystem.setIntakeMotorVelocity(scoreSpeed);
                    currentState = CommandState.SCORING;
                }
                break;
                
            case SCORING:
                // Check if scoring is complete (no coral detected)
                if (!harpoonSubsystem.isCoralDetected()) {
                    System.out.println("SEQUENTIAL SCORE: Scoring complete");
                    currentState = CommandState.FINISHED;
                }
                break;
                
            case FINISHED:
                // Do nothing, waiting for isFinished to return true
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the harpoon intake motor
        harpoonSubsystem.setIntakeMotorVelocity(0);
        System.out.println("SEQUENTIAL SCORE: Command ended, interrupted: " + interrupted);
    }
    
    @Override
    public boolean isFinished() {
        return currentState == CommandState.FINISHED;
    }
} 