package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.subsystems.Elevator;

/*
 * pre-selects an elevator and harpoon position
 * the operator runs this command, then the driver commands the actual position later on
 */
public class PrepareElevatorCommand extends Command {
    // the subsystems required
    private Elevator elevatorSubsystem;
    private ElevatorMode mode;
    private DoubleSupplier position;
    
    public PrepareElevatorCommand(Elevator elevatorSubsystem, ElevatorMode mode, DoubleSupplier position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.mode = mode;
        this.position = position;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setSelectedPosition(position.getAsDouble());
        elevatorSubsystem.setNextMode(mode);

        System.out.println("SELECTED ELEVATOR POSITION: " + position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}
