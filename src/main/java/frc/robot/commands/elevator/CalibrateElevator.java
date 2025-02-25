package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.subsystems.Elevator;


public class CalibrateElevator extends Command {
    private Elevator elevatorSubsystem;
    private ElevatorMode mode;
    
    public CalibrateElevator(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
       
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        if (elevatorSubsystem.isBottomPressed());
        return true;
    }
}
