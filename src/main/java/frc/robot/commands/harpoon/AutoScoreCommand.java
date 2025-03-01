package frc.robot.commands.harpoon;

import frc.robot.subsystems.Harpoon;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoScoreCommand extends Command {
    private Harpoon harpoonSubsystem;
    private double speed;

    public AutoScoreCommand(Harpoon harpoonSubsystem, double speed) {
        // creating subsystem, adding the subsystem as a requirement so it is not used elsewhere which could cause problems. 
        this.harpoonSubsystem = harpoonSubsystem;
        
        this.speed = speed;
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setIntakeMotorVelocity(speed);
        
        System.out.println("SCORING...");
    }
    
    @Override
    public void end(boolean interrupt) {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setIntakeMotorVelocity(0);
        System.out.println("DONE SCORING!");
    }

    @Override
    public boolean isFinished() {
        return !harpoonSubsystem.isCoralDetected();
    }
}