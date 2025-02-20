package frc.robot.commands;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;

public class SetLEDCommand extends Command{

    private LED LEDSubsystem;

    public SetLEDCommand(LED LEDSubsystem) {
        this.LEDSubsystem = LEDSubsystem;
        this.addRequirements(LEDSubsystem);
    }

    @Override
    public void initialize() {
        // we only need to set the setpoint once, that's it
        LEDSubsystem.setLEDColor(0.5);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command
        return true;
    }
}
