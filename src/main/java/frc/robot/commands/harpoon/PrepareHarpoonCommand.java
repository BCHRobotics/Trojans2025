package frc.robot.commands.harpoon;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.subsystems.Harpoon;


public class PrepareHarpoonCommand extends Command {
    private Harpoon harpoonSubsystem;
    private HarpoonMode mode;
    private DoubleSupplier pivotSetpoint;
    
    public PrepareHarpoonCommand(Harpoon harpoonSubsystem, HarpoonMode mode, DoubleSupplier pivotSetpoint) {
        this.harpoonSubsystem = harpoonSubsystem;
        this.pivotSetpoint = pivotSetpoint;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        harpoonSubsystem.setSelectedPosition(pivotSetpoint.getAsDouble());
        harpoonSubsystem.setNextMode(mode);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}