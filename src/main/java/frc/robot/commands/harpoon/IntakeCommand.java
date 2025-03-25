package frc.robot.commands.harpoon;
import frc.robot.Constants.MechanismMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private Harpoon harpoonSubsystem;
    private Elevator elevatorSubsystem;
    private BooleanSupplier buttonPressed;
    private double speed;
   

    public IntakeCommand(Elevator elevatorSubsystem, Harpoon harpoonSubsystem, double speed, BooleanSupplier buttonPressed) {
        // creating subsystem, adding the subsystem as a requirement so it is not used elsewhere which could cause problems. 
        this.harpoonSubsystem = harpoonSubsystem;
        this.addRequirements(harpoonSubsystem);
        
        this.speed = speed;
        this.buttonPressed = buttonPressed;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        // we only need to set the reference once. This is true for probably all closed loop control systems
        harpoonSubsystem.setIntakeMotorVelocity(-speed);
        
        System.out.println("HARPOON TIME");
    }

    @Override
    public void end(boolean interrupted) {
        //we want to stop the intake motor velocity after we've scored. We also want to stow the elevator
        harpoonSubsystem.setIntakeMotorVelocity(0);
    }

    @Override
    public boolean isFinished() {
        // this is called after we have set the setpoint, so we can just end the command once the sensor sees the coral
        if(harpoonSubsystem.isCoralDetected() || !buttonPressed.getAsBoolean()) {
            System.out.println("INTAKE DONE!");
            if (harpoonSubsystem.isCoralDetected() && elevatorSubsystem.getMode() == MechanismMode.FEEDER) {
                System.out.println("STOWING...");
                elevatorSubsystem.setMode(MechanismMode.STOWED);
                harpoonSubsystem.setMode(MechanismMode.STOWED);
            }
            return true;
        }
        else{
            return false;
        }
    }
    
}