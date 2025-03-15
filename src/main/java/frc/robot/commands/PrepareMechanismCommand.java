package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

/*
 * pre-selects an elevator and harpoon position
 * the operator runs this command, then the driver commands the actual position later on
 */
public class PrepareMechanismCommand extends Command {
    // elevator stuff
    private Elevator elevatorSubsystem;
    private ElevatorMode elevatorMode;
    private DoubleSupplier position;

    // harpoon stuff
    private Harpoon harpoonSubsystem;
    private HarpoonMode harpoonMode;
    private DoubleSupplier pivotSetpoint;
    
    public PrepareMechanismCommand(Elevator elevatorSubsystem, ElevatorMode elevatorMode, DoubleSupplier position,
    Harpoon harpoonSubsystem, HarpoonMode harpoonMode, DoubleSupplier pivotSetpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorMode = elevatorMode;
        this.position = position;

        this.harpoonMode = harpoonMode;
        this.harpoonSubsystem = harpoonSubsystem;
        this.pivotSetpoint = pivotSetpoint;
    }

    @Override
    public void initialize() {
        double newPos = position.getAsDouble();
        elevatorSubsystem.setSelectedPosition(newPos);
        elevatorSubsystem.setNextMode(elevatorMode);

        harpoonSubsystem.setSelectedPosition(pivotSetpoint.getAsDouble());
        harpoonSubsystem.setNextMode(harpoonMode);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
       
        return true;
    }
}