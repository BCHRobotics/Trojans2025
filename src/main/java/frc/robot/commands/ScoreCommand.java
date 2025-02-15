package frc.robot.commands;

import frc.robot.subsystems.Harpoon;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCommand extends Command {
     private Harpoon harpoonSubsystem;
    private double scoreAngle;

    public ScoreCommand(Harpoon harpoonSubsystem, double scoreAngle) {
        // creating subsystem, adding the subsystem as a requirement so it is not used elsewhere which could cause problems. 
        this.harpoonSubsystem = harpoonSubsystem;
        this.addRequirements(harpoonSubsystem);
        this.scoreAngle = scoreAngle;
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setRotationMotorPosition(scoreAngle);
        harpoonSubsystem.setIntakeMotorVelocity(1);
        System.out.println("HARPOON TIME");
    }

    @Override
    public void execute() {
        // if the state of the sensor needs to be checked, do it here
    }

    @Override
    public void end(boolean interrupted) {
        //we want to stop the intake motor velocity after we've scored. We also want to stow the elevator
        harpoonSubsystem.setIntakeMotorVelocity(0);
        harpoonSubsystem.setRotationMotorPosition(0); // could change the angle. We don't know what the home angle is. We could also just reset it with the hardware client.
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command once the sensor sees the coral
        return true;
    }
    
}
