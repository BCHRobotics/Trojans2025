package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    Spark blinkin; 

    int PWMPort;
    private double colorValue;

    public LED(int PWMPort) {
        this.PWMPort = PWMPort;
        blinkin = new Spark(PWMPort);
    }

    public void setColor(double color) {
        blinkin.set(color);
    }
}
