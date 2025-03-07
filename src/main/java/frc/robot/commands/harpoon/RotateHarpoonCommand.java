package frc.robot.commands.harpoon;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Harpoon;

public class RotateHarpoonCommand extends Command{

    private Harpoon harpoonSubsystem;
    private double rotation;

    public RotateHarpoonCommand(Harpoon harpoonSubsystem, double desiredRotation) {
        // creating subsystem, adding the subsystem as a requirement so it is not used elsewhere which could cause problems. 
        this.harpoonSubsystem = harpoonSubsystem;
        
        this.rotation = desiredRotation;
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setRotationMotorPosition(rotation);
        
        System.out.println("Rotating...");
    }
    
    @Override
    public void end(boolean interrupt) {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        System.out.println("DONE SCORING!");
    }

    @Override
    public boolean isFinished() {

        if (harpoonSubsystem.kRotationMotor.getAbsoluteEncoder().getPosition() - rotation < 0.1){
            return true;
        }
        return false;
        
    }
}
