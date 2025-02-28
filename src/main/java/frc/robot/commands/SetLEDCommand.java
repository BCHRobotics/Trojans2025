package frc.robot.commands;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;

public class SetLEDCommand extends Command{

    private LED LEDSubsystem1;
    private LED LEDSubsystem2;
    private double colour;

    public SetLEDCommand(LED LEDSubsystem1, LED LEDSubsystem2, double colour) {
        this.LEDSubsystem1 = LEDSubsystem1;
        this.LEDSubsystem2 = LEDSubsystem2;

        this.colour = colour;
    }

    @Override
    public void initialize() {
        // we only need to set the setpoint once, that's it
        LEDSubsystem1.setColor(colour);
        LEDSubsystem2.setColor(colour);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
