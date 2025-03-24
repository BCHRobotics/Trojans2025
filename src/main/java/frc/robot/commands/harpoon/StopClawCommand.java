package frc.robot.commands.harpoon;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Harpoon;

public class StopClawCommand extends Command {
    private Harpoon harpoonSubsystem;

    public StopClawCommand(Harpoon harpoonSubsystem) {
        this.harpoonSubsystem = harpoonSubsystem;
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setIntakeMotorVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
