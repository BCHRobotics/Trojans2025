package frc.robot.commands;

import frc.robot.subsystems.Harpoon;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotHarpoon extends Command {
    private Harpoon harpoonSubsystem;
    private double setpoint;
   

    public PivotHarpoon(Harpoon harpoonSubsystem, double setpoint) {
        System.out.println("Called Harpoon");
        // creating subsystem, adding the subsystem as a requirement so it is not used elsewhere which could cause problems. 
        this.harpoonSubsystem = harpoonSubsystem;
        System.out.println("Added Harpoon");
        this.addRequirements(harpoonSubsystem);
        System.out.println("Added Harpoon Subsystem");
        //this.addRequirements(LEDSubsystem);
        this.setpoint = setpoint;
        System.out.println("setpoint"+setpoint);
        
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setRotationMotorPosition(setpoint);
        //harpoonSubsystem.setIntakeMotorVelocity(-1);
        //LEDSubsystem.setLEDColor(-0.3); // I don't know what this colour is. Will read the documentation later. Maybe a good idea to create an LED constants class?
        //System.out.println("PIVOT TIME");
    }

    @Override
    public void execute() {
        // if the state of the sensor needs to be checked, do it here
        //LEDSubsystem.setLEDColor(-0.8);

    }

    @Override
    public void end(boolean interrupted) {
        //we want to stop the intake motor velocity after we've scored. We also want to stow the elevator
        //harpoonSubsystem.setIntakeMotorVelocity(0);
        //harpoonSubsystem.setRotationMotorPosition(0); // could change the angle. We don't know what the home angle is. We could also just reset it with the hardware client.
        //LEDSubsystem.setLEDColor(0);
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command once the sensor sees the coral
        return true;
    }
    
}