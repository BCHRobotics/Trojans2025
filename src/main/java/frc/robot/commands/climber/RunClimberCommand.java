package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/*
 * pre-selects and then activates an elevator position, all at once
 * this is useful for autos, when you don't want to be running multiple commands
 * THIS IS BETTER (i think) than directly setting setpoints,
 *  because we don't need to arbitrarily switch between an auto system and a tele system for the mech
 */
public class RunClimberCommand extends Command {
    private Climber climberSubsystem;
    private double speed;
    
    public RunClimberCommand(Climber climberSubsystem, double speed) {
        this.climberSubsystem = climberSubsystem;

        this.speed = speed;
    }

    @Override
    public void initialize() {
        climberSubsystem.setMotorSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
       
        return true;
    }
}