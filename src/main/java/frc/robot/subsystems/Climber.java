package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // the NEO motor used to climb
    private final SparkMax climbMotor;

    private final SparkMaxConfig kMotorConfig = new SparkMaxConfig();

    public Climber() {
        this.climbMotor = new SparkMax(ClimberConstants.kClimbMotorCANID, SparkMax.MotorType.kBrushless);

        this.kMotorConfig.inverted(false);

        this.kMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        climbMotor.configure(kMotorConfig, null, null);
    } 

    public void setMotorSpeed(double speed) {
        climbMotor.set(speed);
    }
}