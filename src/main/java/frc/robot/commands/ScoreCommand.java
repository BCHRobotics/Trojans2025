package frc.robot.commands;

import frc.robot.subsystems.Harpoon;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCommand extends Command {
     private Harpoon harpoonSubsystem;
    private double scoreAngle;

    public ScoreCommand(Harpoon harpoonSubsystem, double scoreAngle) {
        this.harpoonSubsystem = harpoonSubsystem;
        this.addRequirements(harpoonSubsystem);
        this.scoreAngle = scoreAngle;
    }

    @Override
    public void initialize() {
        // we only need to set the reference once
        harpoonSubsystem.setRotationMotorPosition(scoreAngle);
        harpoonSubsystem.setIntakeMotorVelocity(1);
        System.out.println("HARPOON TIME");
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        harpoonSubsystem.setIntakeMotorVelocity(0);
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command once the sensor sees the coral
        return true;
    }
    
}
