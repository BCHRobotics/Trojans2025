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

        double newPos = position.getAsDouble();
        elevatorSubsystem.setSelectedPosition(newPos);
        elevatorSubsystem.setNextMode(mode);

        // if (position.getAsDouble() == ElevatorPosition.L1.getSetpoint()) {
        //     new SetLEDCommand(led2, led1, 0.61, 0).schedule();
        // } else if (position.getAsDouble() == ElevatorPosition.L2.getSetpoint()) {
        //     new SetLEDCommand(led2, led1, 0.65, 0).schedule();
        // } else if (position.getAsDouble() == ElevatorPosition.L3.getSetpoint()) {
        //     new SetLEDCommand(led2, led1, 0.69, 0).schedule();
        // } else if (position.getAsDouble() == ElevatorPosition.L4.getSetpoint()) {
        //     new SetLEDCommand(led2, led1, 0.77, 0).schedule();
        // } else if (position.getAsDouble() == ElevatorPosition.INTAKE.getSetpoint()) {
        //     new SetLEDCommand(led2, led1, 0.91, 0).schedule();
        // }

        System.out.println("SELECTED ELEVATOR POSITION: " + newPos);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
       
        return true;
    }
}
