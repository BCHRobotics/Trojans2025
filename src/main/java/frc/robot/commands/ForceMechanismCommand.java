package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismMode;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import frc.robot.subsystems.LED;

/*
 * pre-selects and then activates an elevator position, all at once
 * this is useful for autos, when you don't want to be running multiple commands
 * THIS IS BETTER (i think) than directly setting setpoints,
 *  because we don't need to arbitrarily switch between an auto system and a tele system for the mech
 */
public class ForceMechanismCommand extends Command {
    // elevator stuff
    private Elevator elevatorSubsystem;
    private MechanismMode newMode;
    private DoubleSupplier position;

    // harpoon stuff
    private Harpoon harpoonSubsystem;
    private DoubleSupplier pivotSetpoint;
    
    // led subsystems, used for intaking so that we can confirm a coral has been aquired
    private LED led1;
    private LED led2;
    
    public ForceMechanismCommand(LED led1, LED led2, Elevator elevatorSubsystem, MechanismMode newMode, DoubleSupplier position,
    Harpoon harpoonSubsystem, DoubleSupplier pivotSetpoint) {
        // first, define elevator-relevant stuff
        this.elevatorSubsystem = elevatorSubsystem;
        this.newMode = newMode;
        this.position = position;
        
        // then, define harpoon-related stuff
        this.harpoonSubsystem = harpoonSubsystem;
        this.pivotSetpoint = pivotSetpoint;

        this.led1 = led1;
        this.led2 = led2;
    }

    @Override
    public void initialize() {
        // set the positions to the position we want, and the modes
        elevatorSubsystem.setSelectedPosition(position.getAsDouble());
        elevatorSubsystem.setNextMode(newMode);
        harpoonSubsystem.setSelectedPosition(pivotSetpoint.getAsDouble());
        harpoonSubsystem.setNextMode(newMode);

        // send those through
        elevatorSubsystem.setMode(newMode);
        harpoonSubsystem.setMode(newMode);

        if (newMode == MechanismMode.FEEDER) {
            new IntakeCommand(elevatorSubsystem, harpoonSubsystem, 0.6, () -> true, false).
            alongWith(new SetLEDCommand(led1, led2, -0.11, 0)).
            andThen(new SetLEDCommand(led1, led2, -0.05, 1.5))
            .schedule();
        }
        else {
            new StopClawCommand(harpoonSubsystem).schedule();
            new SetLEDCommand(led2, led1, 0.87, 0).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
       
        return true;
    }
}