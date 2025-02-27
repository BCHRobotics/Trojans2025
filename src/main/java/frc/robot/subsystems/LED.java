package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    Spark blinkin; 

    int PWMPort;

    public LED(int PWMPort) {
        this.PWMPort = PWMPort;
        blinkin = new Spark(PWMPort);
    }

    public void setLEDColor(double PWMValue) { // the PWM value is from -1 to 1 so it's not really a PWM value, it's normalized
        blinkin.set(PWMValue);
    } 
}
