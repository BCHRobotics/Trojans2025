package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

public class StowElevatorCommand extends Command {

    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;
    
    public StowElevatorCommand(Elevator elevatorSubsystem, Harpoon harpoonSubsytem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.harpoonSubsystem = harpoonSubsytem;
    }

    @Override
    public void initialize() {
        // if we are currently stowed, run the pre-selected mode and the position
        if (elevatorSubsystem.getMode() == ElevatorMode.FEEDER && harpoonSubsystem.isCoralDetected()) {
            elevatorSubsystem.setMode(ElevatorMode.STOWED);
        }
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}