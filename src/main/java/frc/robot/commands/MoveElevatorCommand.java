package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Elevator;

public class MoveElevatorCommand extends Command {
    private Elevator elevatorSubsystem;
    private ElevatorPosition position;

    public MoveElevatorCommand(Elevator elevatorSubsystem, ElevatorPosition position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
    }

    @Override
    public void initialize() {
        // we only need to set the setpoint once, that's it
        elevatorSubsystem.setSetpoint(position.getSetpoint());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command
        return true;
    }
}
