package frc.robot.commands.elevator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class CalibrateElevator extends Command {
    private Elevator elevatorSubsystem;
    private BooleanSupplier input;
    
    public CalibrateElevator(Elevator elevatorSubsystem, BooleanSupplier input) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.input = input;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.activateCalibration();
    }

    @Override
    public void execute(){
        elevatorSubsystem.driveMotorSlow();
    }

    @Override
    public void end(boolean interrputed){
        elevatorSubsystem.stopMotor();
        elevatorSubsystem.stopCalibration();
    }

    @Override
    public boolean isFinished() {
        return !input.getAsBoolean();
    }
}
