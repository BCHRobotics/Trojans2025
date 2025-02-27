package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

public class ToggleElevatorCommand extends Command {

    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;
    
    public ToggleElevatorCommand(Elevator elevatorSubsystem, Harpoon harpoonSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.harpoonSubsystem = harpoonSubsystem;
    }

    @Override
    public void initialize() {
        // if we are currently stowed, run the pre-selected mode and the position
        if (elevatorSubsystem.getMode() == ElevatorMode.STOWED) {
            elevatorSubsystem.setMode(elevatorSubsystem.getNextMode());
            harpoonSubsystem.setMode(harpoonSubsystem.getNextMode());

            if (harpoonSubsystem.getNextMode() == HarpoonMode.FEEDER) {
                new IntakeCommand(elevatorSubsystem, harpoonSubsystem, 0.6, () -> true).schedule();
            }
        }
        else {
            elevatorSubsystem.setMode(ElevatorMode.STOWED);
            harpoonSubsystem.setMode(HarpoonMode.STOWED);
        }
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}
