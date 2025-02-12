package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase{

    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;

    private double setpoint = 0; // rotations
    double currentPos;

    public Elevator() {
        
        //Creating new motors, encoders, and PID controllers
        //motors
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        
        //encoder
        encoder = primaryMotor.getEncoder();
        
        //PID controller
        pidController = new PIDController(
            ElevatorConstants.ElevatorkP,
            ElevatorConstants.ElevatorkI,
            ElevatorConstants.ElevatorkD
        );
        
        pidController.setTolerance(0.1); // orignally 0.5, rotations
        
        //setting limits for safety
        SparkMaxConfig resetConfig = new SparkMaxConfig();
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);

        LimitSwitchConfig switchConfig = new LimitSwitchConfig();
        switchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
        switchConfig.forwardLimitSwitchType(Type.kNormallyClosed);

        switchConfig.forwardLimitSwitchEnabled(false);
        switchConfig.reverseLimitSwitchEnabled(false);

        resetConfig.apply(switchConfig);
        
        //reseting factory defaults
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        
        //configuring follower motor (follower follow main)
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);
        followerMotor.configure(followerConfig, null, null); 

        encoder.setPosition(0);
    } 
    
    public void setTargetPosition(double positionInches) {
        //set limits on the target position
        setpoint = positionInches;
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    @Override
    public void periodic() {
        run();
    }

    public void run() {
        SmartDashboard.putBoolean("Top Limit", primaryMotor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("Bottom Limit", primaryMotor.getReverseLimitSwitch().isPressed());

        double currentPosition = encoder.getPosition(); // inches
        //uses kP, kI, and kD constants to calculate pidOutput
        double pidOutput = pidController.calculate(currentPosition, setpoint);

        double output = pidOutput;
        
        //set limits on the output of the motor
        output = MathUtil.clamp(output, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
        
        primaryMotor.set(applyLimits(output));

        //handleBottomLimit();

        //printing data onto FRC Driver Station
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putNumber("velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Elevator PID Output", applyLimits(output));
        SmartDashboard.putNumber("Elevator Error", (setpoint-currentPosition));
    }

    double applyLimits(double input) {
        boolean isBottomPressed = primaryMotor.getReverseLimitSwitch().isPressed();
        boolean isTopPressed = primaryMotor.getForwardLimitSwitch().isPressed();
        
        if (isBottomPressed) {
            return MathUtil.clamp(input, 0, ElevatorConstants.maxOutput);
        }
        else if (isTopPressed) {
            return MathUtil.clamp(input, -ElevatorConstants.maxOutput, 0);
        }

        if (input < 0) {
            input *= 0.6 * MathUtil.clamp(1 / (Math.abs(encoder.getVelocity()) / 1200), 0, 1);
        }

        return MathUtil.clamp(input, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
    }

    //setting target position based on configured button binding
    public Command moveLevel(String level) {
        return runOnce(()-> {
            switch (level) {
                case "L1":
                    this.setTargetPosition(ElevatorConstants.L1);
                    break;   
                case "L2":
                    this.setTargetPosition(ElevatorConstants.L2);
                    break;
                case "L0":
                this.setTargetPosition(1);
                break;
            }
        });
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

}