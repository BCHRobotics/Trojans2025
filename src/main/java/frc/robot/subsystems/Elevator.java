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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

/*
 * Subsystem for managing elevator movement
 */
public class Elevator extends SubsystemBase{

    // the motors are set up so that one is commanded,
    // and the other just follows that command

    // these are the SparkMaxs
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;

    // the encoder of the primary motor
    private final RelativeEncoder encoder;
    // the PID controller used to move the elevator
    private final PIDController pidController;

    // the current setpoint of the elevator
    private double setpoint = 0;

    public Elevator() {
        
        //Creating new motors, encoders, and PID controllers
        // -----------------------

        // create the sparkmax objects with the proper can IDs
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        
        // relative encoder of the primary motor
        encoder = primaryMotor.getEncoder();
        
        //PID controller
        pidController = new PIDController(
            ElevatorConstants.ElevatorkP,
            ElevatorConstants.ElevatorkI,
            ElevatorConstants.ElevatorkD
        );
        
        // the error tolerance, or allowed error, of the elevator PID controller
        pidController.setTolerance(0.1);

        // motor config
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(40);
        motorConfig.voltageCompensation(12.0);

        // limit switch config,
        // the limit switches ARE NOT ACTIVE HERE because the logic is handled in code,
        // not in the SparkMaxs themselves
        LimitSwitchConfig switchConfig = new LimitSwitchConfig();
        switchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
        switchConfig.forwardLimitSwitchType(Type.kNormallyClosed);

        switchConfig.forwardLimitSwitchEnabled(false);
        switchConfig.reverseLimitSwitchEnabled(false);

        // apply the limit switch data to the motorConfig
        motorConfig.apply(switchConfig);
        
        // apply the motor config to BOTH motors
        primaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, null);
        
        // configuring one motor to follow the other
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);
        followerMotor.configure(followerConfig, null, null); 
    } 

    public void resetElevator() {
        // set the position of the encoder to 0, since the elevator should be resting at the bottom
        // THIS CAUSES ISSUES IF YOU DEPLOY WHILE THE ELEVATOR IS UP, SAME AS LAST YEAR
        encoder.setPosition(0);
        setpoint = 0;
    }

    @Override
    public void periodic() {
        // moving the elevator to the desired setpoint
        drive();

        // debug values
        printToDashboard();;
    }

    /**
     * Driving the elevator motors based on the setpoint and current position
     */
    public void drive() {
        double currentPosition = encoder.getPosition(); // inches
        //uses kP, kI, and kD constants to calculate pidOutput
        double pidOutput = pidController.calculate(currentPosition, setpoint);

        double output = pidOutput;
        
        //set limits on the output of the motor
        output = MathUtil.clamp(output, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
        
        primaryMotor.set(applyLimits(output));

        //printing data onto FRC Driver Station
    }

    /**
     * Set the setpoint of the elevator, 
     * in other words the target position
     * @param newSetpoint the desired position
     */
    public void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }

    /**
     * Applies limit switch logic and velocity limits to the output to be fed to the motors
     * @param input The raw output, only affected by PID (no limit switches or anything yet)
     * @return The transformed output, ready to be passed to the motors
     */
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

    /**
     * Grab the current position from the encoder, 
     * it's a separate function so that the value can be transformed
     * @return The position, in rotations (I think)
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Grab the current velocity from the encoder, 
     * it's a separate function so that the value can be transformed
     * @return The velocity, in rad/s (I think)
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * A space to put periodically updated debug values that need to be put to dashboard
     * This is called in periodic()
     */
    public void printToDashboard() {
        // SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        // SmartDashboard.putNumber("velocity", getVelocity());
        // SmartDashboard.putNumber("Elevator Error", (setpoint-getPosition()));

        // SmartDashboard.putBoolean("Top Limit", primaryMotor.getForwardLimitSwitch().isPressed());
        // SmartDashboard.putBoolean("Bottom Limit", primaryMotor.getReverseLimitSwitch().isPressed());
    }
}