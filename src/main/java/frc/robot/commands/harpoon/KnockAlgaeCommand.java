package frc.robot.commands.harpoon;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.HarpoonConstants.HarpoonPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

public class KnockAlgaeCommand extends Command{
    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;
    private Timer timer = new Timer();
    public boolean timeElapsed;
    public BooleanSupplier interrupted;

    public KnockAlgaeCommand(Harpoon harpoonSubsystem, Elevator elevatorSubsystem, BooleanSupplier buttonInterrupt) {
        this.harpoonSubsystem = harpoonSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.interrupted = buttonInterrupt;
        // Add subsystems as requirements
        addRequirements(harpoonSubsystem);
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Set the elevator to the INTAKE position
        elevatorSubsystem.setSetpoint(ElevatorPosition.INTAKE.getSetpoint());
        // Set the harpoon to the INTAKE position
        harpoonSubsystem.setRotationMotorPosition(HarpoonPosition.INTAKE.getSetpoint());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Run the intake motor
        harpoonSubsystem.setIntakeMotorVelocity(-0.5);
        if (timer.get() > 5) {
            timeElapsed = true;
        }
    }

    @Override
    public void end(boolean interrupt) {
        // Stop the intake motor
        harpoonSubsystem.setIntakeMotorVelocity(0);
        harpoonSubsystem.setRotationMotorPosition(HarpoonPosition.STOWED.getSetpoint());
        if (harpoonSubsystem.kRotationMotor.getAbsoluteEncoder().getPosition()-HarpoonPosition.STOWED.getSetpoint() < 0.1) {
            elevatorSubsystem.setSetpoint(ElevatorPosition.INTAKE.getSetpoint());
        }
    }

    @Override
    public boolean isFinished() {
        return timeElapsed || interrupted.getAsBoolean();
    }
}
