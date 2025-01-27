package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax kLeftMotor;
    private final SparkMax kRightMotor;

    private final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig kRightConfig = new SparkMaxConfig();

    private final SparkClosedLoopController kLeftController;


    public Climber() {
        this.kLeftMotor = new SparkMax(Constants.ClimberConstants.kLeftCanID, SparkMax.MotorType.kBrushless);
        this.kRightMotor = new SparkMax(Constants.ClimberConstants.kRightCanID, SparkMax.MotorType.kBrushless);

        this.kLeftConfig.inverted(false);

        this.kLeftConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        this.kRightConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        this.kRightConfig.follow(kLeftMotor, true);

        this.kLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            Constants.ClimberConstants.climberP,
            Constants.ClimberConstants.climberP,
            Constants.ClimberConstants.climberP);

        this.kLeftController = kLeftMotor.getClosedLoopController();

        kLeftMotor.configure(kLeftConfig, null, null);
        kRightMotor.configure(kRightConfig, null, null);
            
    } 
    
    public Command climb() {
        return new InstantCommand(() -> {
            this.kLeftController.setReference(Constants.ClimberConstants.gearConversionFactor * 
            Constants.ClimberConstants.fullyRotatedPositionDegrees/360,
            SparkBase.ControlType.kMAXMotionPositionControl);
        });
    }

    public Command reset() {
        return new InstantCommand(() -> {
            this.kLeftController.setReference(0,
            SparkBase.ControlType.kMAXMotionPositionControl);
        });
    }
    
}
