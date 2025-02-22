package frc.robot.commands;

import frc.robot.Constants.HarpoonConstants;
import frc.robot.subsystems.Harpoon;
import frc.robot.subsystems.PhotoElectricSensor;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private Harpoon harpoonSubsystem;
    private PhotoElectricSensor sensorSubsystem;
    private LED LEDSubsystem;
   

    public IntakeCommand(Harpoon harpoonSubsystem) {
        // creating subsystem, adding the subsystem as a requirement so it is not used elsewhere which could cause problems. 
        this.harpoonSubsystem = harpoonSubsystem;
        this.addRequirements(harpoonSubsystem);
        this.addRequirements(LEDSubsystem);
        
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setRotationMotorPosition(HarpoonConstants.HarpoonPosition.INTAKE.getSetpoint());
        harpoonSubsystem.setIntakeMotorVelocity(-1);
        LEDSubsystem.setLEDColor(-0.3); // I don't know what this colour is. Will read the documentation later. Maybe a good idea to create an LED constants class?
        System.out.println("HARPOON TIME");
    }

    @Override
    public void execute() {
        // if the state of the sensor needs to be checked, do it here
        LEDSubsystem.setLEDColor(-0.8);
    }

    @Override
    public void end(boolean interrupted) {
        //we want to stop the intake motor velocity after we've scored. We also want to stow the elevator
        harpoonSubsystem.setIntakeMotorVelocity(0);
        harpoonSubsystem.setRotationMotorPosition(0); // could change the angle. We don't know what the home angle is. We could also just reset it with the hardware client.
        LEDSubsystem.setLEDColor(0);
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command once the sensor sees the coral
        if(!sensorSubsystem.isCoralDetected()) {
            return true;
        }
        else{
            return false;
        }
    }
    
}
