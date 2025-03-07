package frc.robot.commands.harpoon;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.Constants.HarpoonConstants.HarpoonPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

/**
 * An intake command that uses enumerated constants to:
 * 1. Set the elevator to the INTAKE position
 * 2. Set the harpoon to the INTAKE position
 * 3. Run the intake motor once both are in position
 */
public class EnumeratedIntakeCommand extends Command {
    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;
    private double intakeSpeed;

    private enum IntakeState {
        MOVING_MECHANISMS,
        INTAKING,
        STOWING,
        DONE
    }

    private IntakeState currentState = IntakeState.MOVING_MECHANISMS;

    /**
     * Creates a new EnumeratedIntakeCommand.
     * 
     * @param elevatorSubsystem The elevator subsystem
     * @param harpoonSubsystem The harpoon subsystem
     * @param intakeSpeed The speed at which to run the intake motor
     */
    public EnumeratedIntakeCommand(Elevator elevatorSubsystem, Harpoon harpoonSubsystem, double intakeSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.harpoonSubsystem = harpoonSubsystem;
        this.intakeSpeed = intakeSpeed;

        // Add subsystems as requirements
        addRequirements(elevatorSubsystem, harpoonSubsystem);
    }

    @Override
    public void initialize() {
        // Set the elevator and harpoon modes to FEEDER mode
        elevatorSubsystem.setMode(ElevatorMode.FEEDER);
        harpoonSubsystem.setMode(HarpoonMode.FEEDER);

        // Set the elevator to the INTAKE position
        elevatorSubsystem.setSetpoint(ElevatorPosition.INTAKE.getSetpoint());

        // Set the harpoon to the INTAKE position
        harpoonSubsystem.setRotationMotorPosition(HarpoonPosition.INTAKE.getSetpoint());

        System.out.println("ENUMERATED INTAKE: Moving mechanisms to intake positions");
        
        currentState = IntakeState.MOVING_MECHANISMS;
    }

    @Override
    public void execute() {
        switch (currentState) {
            case MOVING_MECHANISMS:
                // Check if both mechanisms are in position
                boolean elevatorInPosition = Math.abs(
                    elevatorSubsystem.getEncoderPosition() - ElevatorPosition.INTAKE.getSetpoint()
                ) < 1.0; // Within 1 inch tolerance

                boolean harpoonInPosition = Math.abs(
                    harpoonSubsystem.kRotationMotor.getAbsoluteEncoder().getPosition() - 
                    HarpoonPosition.INTAKE.getSetpoint()
                ) < 0.1; // Within 0.1 rotation tolerance

                if (elevatorInPosition && harpoonInPosition) {
                    // Both mechanisms are in position, start intaking
                    System.out.println("ENUMERATED INTAKE: Mechanisms in position, starting intake");
                    harpoonSubsystem.setIntakeMotorVelocity(-intakeSpeed);
                    currentState = IntakeState.INTAKING;
                }
                break;

            case INTAKING:
                // Check if a coral is detected
                if (harpoonSubsystem.isCoralDetected()) {
                    System.out.println("ENUMERATED INTAKE: Coral detected, stopping intake motor");
                    // Immediately stop the intake motor
                    harpoonSubsystem.setIntakeMotorVelocity(0);
                    currentState = IntakeState.STOWING;
                }
                break;
                
            case STOWING:
                // Stop the elevator and move it to stowed position
                System.out.println("ENUMERATED INTAKE: Stowing elevator and harpoon");
                
                // Set the modes to STOWED
                elevatorSubsystem.setMode(ElevatorMode.STOWED);
                harpoonSubsystem.setMode(HarpoonMode.STOWED);
                
                // Set the positions to STOWED
                elevatorSubsystem.setSetpoint(ElevatorPosition.STOWED.getSetpoint());
                harpoonSubsystem.setRotationMotorPosition(HarpoonPosition.STOWED.getSetpoint());
                
                // Move to DONE state
                currentState = IntakeState.DONE;
                break;

            case DONE:
                // Do nothing, waiting for isFinished to return true
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake motor in case it's still running (e.g., if interrupted)
        harpoonSubsystem.setIntakeMotorVelocity(0);

        // If interrupted, we may need to stow the mechanisms explicitly
        if (interrupted) {
            System.out.println("ENUMERATED INTAKE: Command interrupted, ensuring mechanisms are in safe position");
            // Only stow if we were interrupted during intaking
            if (currentState == IntakeState.INTAKING || currentState == IntakeState.MOVING_MECHANISMS) {
                elevatorSubsystem.setMode(ElevatorMode.STOWED);
                harpoonSubsystem.setMode(HarpoonMode.STOWED);
                elevatorSubsystem.setSetpoint(ElevatorPosition.STOWED.getSetpoint());
                harpoonSubsystem.setRotationMotorPosition(HarpoonPosition.STOWED.getSetpoint());
            }
        }

        System.out.println("ENUMERATED INTAKE: Command ended, interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return currentState == IntakeState.DONE;
    }
} 