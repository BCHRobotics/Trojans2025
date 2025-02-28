package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    // the REV Blinkin LED driver is run through a Spark object
    Spark blinkin; 
    // the Spark is connected to a PWM port on the rio
    int PWMPort;

    // instantiate the spark object with the PWM id
    public LED(int PWMPort) {
        this.PWMPort = PWMPort;
        blinkin = new Spark(PWMPort);
    }

    // set the spark to a color (SEE BLIKIN DOCS FOR WHAT NUMBERS LEAD TO WHAT COLORS)
    public void setColor(double color) {
        blinkin.set(color);
    }
}
