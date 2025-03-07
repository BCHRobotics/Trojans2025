package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.harpoon.IntakeCommand;
import frc.robot.commands.harpoon.StopClawCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harpoon;
import frc.robot.subsystems.LED;

public class MoveElevatorCommand extends Command {
        // elevator and claw subsystems to call the commands on
    private Elevator elevatorSubsystem;
    private Harpoon harpoonSubsystem;

    // led subsystems, used for intaking so that we can confirm a coral has been aquired
    private LED led1;
    private LED led2;

    private double setpoint;
    
    public MoveElevatorCommand(Elevator elevatorSubsystem, double setpoint) {

        this.elevatorSubsystem = elevatorSubsystem;

        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        // if we are currently stowed, run the pre-selected mode and the position
        elevatorSubsystem.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        // finish immediately, bc the command only has to select the position once
        if (Math.abs(elevatorSubsystem.getEncoderPosition() - setpoint) < 0.5) {
            return true;
        }
        return false;
    }
}
