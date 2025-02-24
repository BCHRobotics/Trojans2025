package frc.robot.commands.harpoon;
import frc.robot.subsystems.Harpoon;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private Harpoon harpoonSubsystem;
    
    private double speed;
   

    public IntakeCommand(Harpoon harpoonSubsystem, double speed) {
        // creating subsystem, adding the subsystem as a requirement so it is not used elsewhere which could cause problems. 
        this.harpoonSubsystem = harpoonSubsystem;
        this.addRequirements(harpoonSubsystem);
        
        this.speed = speed;
        
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setIntakeMotorVelocity(-speed);
        
        System.out.println("HARPOON TIME");
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        //we want to stop the intake motor velocity after we've scored. We also want to stow the elevator
        harpoonSubsystem.setIntakeMotorVelocity(0);
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command once the sensor sees the coral
        if(harpoonSubsystem.isCoralDetected()) {
            return true;
        }
        else{
            return false;
        }
    }
    
}
