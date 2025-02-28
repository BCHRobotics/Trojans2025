package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax climbMotor;


    private final SparkMaxConfig kMotorConfig = new SparkMaxConfig();

    private final SparkClosedLoopController kMotorController;

    public Climber() {
        this.climbMotor = new SparkMax(ClimberConstants.kClimbMotorCANID, SparkMax.MotorType.kBrushless);

        this.kMotorConfig.inverted(false);

        this.kMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        this.kMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            ClimberConstants.climberP,
            ClimberConstants.climberP,
            ClimberConstants.climberP);

        this.kMotorController = climbMotor.getClosedLoopController();

        climbMotor.configure(kMotorConfig, null, null);

            
    } 

    public Command prepareClimb(){
        return new InstantCommand(() -> {
            this.kMotorController.setReference(
                ClimberConstants.gearConversionFactor * 
                (ClimberConstants.preparedPositionDegrees/360),
                SparkBase.ControlType.kMAXMotionPositionControl);
        });
    }
    
    public Command climb() {
        return new InstantCommand(() -> {
            this.kMotorController.setReference(
                ClimberConstants.gearConversionFactor * 
                (ClimberConstants.fullyRotatedPositionDegrees/360),
                SparkBase.ControlType.kMAXMotionPositionControl);
        });
    }

    public Command reset() {
        return new InstantCommand(() -> {
            this.kMotorController.setReference(0,
            SparkBase.ControlType.kMAXMotionPositionControl);
        });
    }
    
}