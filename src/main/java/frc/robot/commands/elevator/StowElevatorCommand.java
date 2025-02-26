package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.subsystems.Elevator;

public class StowElevatorCommand extends Command {

    private Elevator elevatorSubsystem;
    
    public StowElevatorCommand(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        // if we are currently stowed, run the pre-selected mode and the position
        if (elevatorSubsystem.getMode() == ElevatorMode.FEEDER) {
            elevatorSubsystem.setMode(ElevatorMode.STOWED);
        }
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}