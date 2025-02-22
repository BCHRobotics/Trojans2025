package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class PhotoElectricSensor extends SubsystemBase {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    DigitalInput sensor;
    boolean state;

    public PhotoElectricSensor(int port) {
        // Initialize the sensor here
        sensor = new DigitalInput(port);
    }

    public boolean isCoralDetected() {
        // Return the state of the sensor
        state = !sensor.get();
        return !sensor.get();
    }
    
    @Override
    public void periodic() {
        isCoralDetected();
    }
}
