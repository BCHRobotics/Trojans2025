package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.subsystems.Elevator;


public class MoveElevatorCommand extends Command {
    private Elevator elevatorSubsystem;
    private ElevatorMode mode;
    private double setpoint;
    
    public MoveElevatorCommand(Elevator elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.setpoint = setpoint;
       
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setSetpoint(setpoint);
        elevatorSubsystem.setNextMode(mode);

        System.out.println("SELECTED ELEVATOR SETPOINT: " + setpoint);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        return true;
    }
}
