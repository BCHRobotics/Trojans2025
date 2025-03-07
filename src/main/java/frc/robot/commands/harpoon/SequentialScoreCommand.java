package frc.robot.commands.harpoon;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.Constants.HarpoonConstants.HarpoonPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

/**
 * A sequential command that:
 * 1. Sets the elevator to a specific setpoint
 * 2. Waits until the elevator reaches within 1 inch of the target
 * 3. Then sets the harpoon to a specific setpoint
 * 4. Finally scores the game piece
 * 5. Waits 0.25 seconds after scoring
 * 6. Stows the harpoon
 * 7. Stows the elevator
 */
public class SequentialScoreCommand extends Command {
    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;
    
    private double elevatorSetpoint;
    private double harpoonSetpoint;
    private double scoreSpeed;
    
    // Timer for waiting after scoring
    private long waitStartTime;
    private static final long WAIT_DURATION_MS = 250; // 0.25 seconds in milliseconds
    
    private enum CommandState {
        MOVING_ELEVATOR,
        MOVING_HARPOON,
        SCORING,
        WAITING,          // Wait 0.25 seconds after scoring
        STOWING_HARPOON,  // Stow the harpoon first
        STOWING_ELEVATOR, // Then stow the elevator
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
                if (Math.abs(elevatorPosition - elevatorSetpoint) < 2) {
                    // Elevator is within tolerance, now move the harpoon
                    System.out.println("SEQUENTIAL SCORE: Elevator in position, moving harpoon to " + harpoonSetpoint);
                    harpoonSubsystem.setRotationMotorPosition(harpoonSetpoint);
                    currentState = CommandState.MOVING_HARPOON;
                }
                break;
                
            case MOVING_HARPOON:
                // Check if the harpoon is in position
                double harpoonPosition = harpoonSubsystem.kRotationMotor.getAbsoluteEncoder().getPosition();
                if (Math.abs(harpoonPosition - harpoonSetpoint) < 0.02) {
                    // Harpoon is in position, now score
                    System.out.println("SEQUENTIAL SCORE: Harpoon in position, scoring at speed " + scoreSpeed);
                    harpoonSubsystem.setIntakeMotorVelocity(scoreSpeed);
                    currentState = CommandState.SCORING;
                }
                break;
                
            case SCORING:
                // Check if scoring is complete (no coral detected)
                if (!harpoonSubsystem.isCoralDetected()) {
                    System.out.println("SEQUENTIAL SCORE: Coral no longer detected, stopping intake and waiting");
                    // Stop the intake motor
                    harpoonSubsystem.setIntakeMotorVelocity(0);
                    // Start the wait timer
                    waitStartTime = System.currentTimeMillis();
                    currentState = CommandState.WAITING;
                }
                break;
                
            case WAITING:
                // Check if we've waited long enough
                if (System.currentTimeMillis() - waitStartTime >= WAIT_DURATION_MS) {
                    System.out.println("SEQUENTIAL SCORE: Wait complete, now stowing harpoon");
                    // Start stowing the harpoon
                    harpoonSubsystem.setMode(HarpoonMode.STOWED);
                    harpoonSubsystem.setRotationMotorPosition(HarpoonPosition.STOWED.getSetpoint());
                    currentState = CommandState.STOWING_HARPOON;
                }
                break;
                
            case STOWING_HARPOON:
                // Check if the harpoon is in stowed position
                double currentHarpoonPosition = harpoonSubsystem.kRotationMotor.getAbsoluteEncoder().getPosition();
                if (Math.abs(currentHarpoonPosition - HarpoonPosition.STOWED.getSetpoint()) < 0.1) {
                    System.out.println("SEQUENTIAL SCORE: Harpoon stowed, now stowing elevator");
                    // Now stow the elevator
                    elevatorSubsystem.setMode(ElevatorMode.STOWED);
                    elevatorSubsystem.setSetpoint(ElevatorPosition.STOWED.getSetpoint());
                    currentState = CommandState.STOWING_ELEVATOR;
                }
                break;
                
            case STOWING_ELEVATOR:
                // Check if the elevator is in stowed position
                double currentElevatorPosition = elevatorSubsystem.getEncoderPosition();
                if (Math.abs(currentElevatorPosition - ElevatorPosition.STOWED.getSetpoint()) < 2.0) {
                    System.out.println("SEQUENTIAL SCORE: Elevator stowed, command complete");
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
        
        // If interrupted, ensure mechanisms are in a safe position
        if (interrupted) {
            System.out.println("SEQUENTIAL SCORE: Command interrupted, ensuring mechanisms are in safe position");
            elevatorSubsystem.setMode(ElevatorMode.STOWED);
            harpoonSubsystem.setMode(HarpoonMode.STOWED);
            elevatorSubsystem.setSetpoint(ElevatorPosition.STOWED.getSetpoint());
            harpoonSubsystem.setRotationMotorPosition(HarpoonPosition.STOWED.getSetpoint());
        }
        
        System.out.println("SEQUENTIAL SCORE: Command ended, interrupted: " + interrupted);
    }
    
    @Override
    public boolean isFinished() {
        return currentState == CommandState.FINISHED;
    }
} 