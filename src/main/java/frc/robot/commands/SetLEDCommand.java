package frc.robot.commands;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;

public class SetLEDCommand extends Command{

    private LED LEDSubsystem;
    private double colour;

    public SetLEDCommand(LED LEDSubsystem, double colour) {
        this.LEDSubsystem = LEDSubsystem;
        this.addRequirements(LEDSubsystem);
        this.colour = colour;
    }

    @Override
    public void initialize() {
        // we only need to set the setpoint once, that's it
        LEDSubsystem.setLEDColor(colour);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
