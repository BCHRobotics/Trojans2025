package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;


public class Harpoon extends SubsystemBase{

    private final SparkMax kIntakeMotor; // intake motor is the same as the "shooter" motor
    private final RelativeEncoder kIntakeEncoder;
    private final SparkMaxConfig kIntakeConfig = new SparkMaxConfig();
    private final double maxVelocity = 1000; // This is in rpm
    private final double maxAcceleration = 1000; // This is in rpm/second
    private final DigitalInput harpoonBeamBreaker = new DigitalInput(0);

    // private final RelativeEncoder kLeftEncoder;
    private final SparkClosedLoopController kIntakeController;
    private double rotationMotorPosition;

    private Elevator m_elevator = Elevator.getInstance();

    public Harpoon(){
        this.kIntakeMotor = new SparkMax(Constants.HarpoonConstants.kIntakeMotorCANID, MotorType.kBrushless);

        this.kIntakeEncoder = kIntakeMotor.getEncoder();
        this.kIntakeConfig.inverted(false);
        this.kIntakeConfig.idleMode(IdleMode.kBrake);

        this.kIntakeController = kIntakeMotor.getClosedLoopController();
    }

    public Command intake(){
        return new InstantCommand(()->{
            kIntakeMotor.set(1);
        });

    }
    /* 
    public Command scoreL1(){
        return this.run(
            m_elevator.moveToPosition(rotationMotorPosition))
            .until(() -> harpoonBeamBreaker.get());
    }

    */

}
