package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

/*
 * pre-selects an elevator and harpoon position
 * the operator runs this command, then the driver commands the actual position later on
 */
public class PrepareMechanismCommand extends Command {
    // elevator stuff
    private Elevator elevatorSubsystem;
    private MechanismMode newMode;
    private DoubleSupplier position;

    // harpoon stuff
    private Harpoon harpoonSubsystem;
    private DoubleSupplier pivotSetpoint;
    
    public PrepareMechanismCommand(Elevator elevatorSubsystem, MechanismMode newMode, DoubleSupplier position,
    Harpoon harpoonSubsystem, DoubleSupplier pivotSetpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.newMode = newMode;
        this.position = position;

        this.harpoonSubsystem = harpoonSubsystem;
        this.pivotSetpoint = pivotSetpoint;
    }

    @Override
    public void initialize() {
        double newPos = position.getAsDouble();
        elevatorSubsystem.setSelectedPosition(newPos);
        elevatorSubsystem.setNextMode(newMode);

        harpoonSubsystem.setSelectedPosition(pivotSetpoint.getAsDouble());
        harpoonSubsystem.setNextMode(newMode);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
       
        return true;
    }
}