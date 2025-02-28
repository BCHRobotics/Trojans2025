package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import frc.robot.subsystems.LED;

public class ToggleMechanismCommand extends Command {

    // elevator and claw subsystems to call the commands on
    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;

    // led subsystems, used for intaking so that we can confirm a coral has been aquired
    private LED led1;
    private LED led2;
    
    public ToggleMechanismCommand(LED led1, LED led2, Elevator elevatorSubsystem, Harpoon harpoonSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.harpoonSubsystem = harpoonSubsystem;

        this.led1 = led1;
        this.led2 = led2;
    }

    @Override
    public void initialize() {
        // if we are currently stowed, run the pre-selected mode and the position
        if (elevatorSubsystem.getMode() == ElevatorMode.STOWED) {
            elevatorSubsystem.setMode(elevatorSubsystem.getNextMode());
            harpoonSubsystem.setMode(harpoonSubsystem.getNextMode());

            if (harpoonSubsystem.getNextMode() == HarpoonMode.FEEDER) {
                new IntakeCommand(elevatorSubsystem, harpoonSubsystem, 0.6, () -> true).
                alongWith(new SetLEDCommand(led1, led2, -0.11, 0)).
                andThen(new SetLEDCommand(led1, led2, -0.05, 1.5))
                .schedule();
            }
        }
        else {
            elevatorSubsystem.setMode(ElevatorMode.STOWED);
            harpoonSubsystem.setMode(HarpoonMode.STOWED);

            new StopClawCommand(harpoonSubsystem).schedule();
            new SetLEDCommand(led2, led1, 0.67, 0).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}
