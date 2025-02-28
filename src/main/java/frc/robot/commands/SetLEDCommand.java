package frc.robot.commands;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/*
 * command for changing the colors of the LEDs
 */
public class SetLEDCommand extends Command{

    private LED LEDSubsystem1;
    private LED LEDSubsystem2;
    private double colour;

    private double timer;

    public SetLEDCommand(LED LEDSubsystem1, LED LEDSubsystem2, double colour, double timer) {
        this.LEDSubsystem1 = LEDSubsystem1;
        this.LEDSubsystem2 = LEDSubsystem2;

        this.colour = colour;

        this.timer = timer;
    }

    @Override
    public void initialize() {
        // we only need to set the setpoint once, that's it
        LEDSubsystem1.setColor(colour);
        LEDSubsystem2.setColor(colour);
    }

    @Override
    public void end(boolean interrupt) {
        if (timer != 0) {
            new WaitCommand(timer).andThen(new SetLEDCommand(LEDSubsystem1, LEDSubsystem2, 0.87, 0)).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
