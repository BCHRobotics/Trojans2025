package frc.robot.commands.harpoon;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Harpoon;


public class PrepareHarpoonCommand extends Command {
    private Harpoon harpoonSubsystem;
    private DoubleSupplier pivotSetpoint;
    
    public PrepareHarpoonCommand(Harpoon harpoonSubsystem, DoubleSupplier pivotSetpoint) {
        this.harpoonSubsystem = harpoonSubsystem;
        this.pivotSetpoint = pivotSetpoint;
    }

    @Override
    public void initialize() {
        harpoonSubsystem.setNextSetpoint(pivotSetpoint.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}