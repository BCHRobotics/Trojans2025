package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Elevator;


public class PrepareElevatorCommand extends Command {
    private Elevator elevatorSubsystem;
    private ElevatorMode mode;
    private ElevatorPosition position;
    
    public PrepareElevatorCommand(Elevator elevatorSubsystem, ElevatorMode mode, ElevatorPosition position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.mode = mode;
        this.position = position;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setSelectedPosition(position);
        elevatorSubsystem.setNextMode(mode);

        System.out.println("SELECTED ELEVATOR POSITION: " + position);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}
