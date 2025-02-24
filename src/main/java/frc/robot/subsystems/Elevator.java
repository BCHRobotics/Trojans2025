package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

// we may want to switch to using the maxmotion controller

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
    private double setpoint = 0; // rotations

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
        motorConfig.smartCurrentLimit(120);
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

        SmartDashboard.putBoolean("invert 1", primaryMotor.getInverted()); // Deprecated !!
        SmartDashboard.putBoolean("invert 2", followerMotor.getInverted());
        
        // configuring one motor to follow the other
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);
        followerMotor.configure(followerConfig, null, PersistMode.kPersistParameters); 

        SmartDashboard.putNumber("Elevator Power", 0);
    } 

    public void resetElevator() {
        // set the position of the encoder to 0, since the elevator should be resting at the bottom
        // THIS CAUSES ISSUES IF YOU DEPLOY WHILE THE ELEVATOR IS UP, SAME AS LAST YEAR
        encoder.setPosition(0);
        setpoint = 0;
    }
    // EMERGENCY STOP THE ELEVATOR
    public Command emergencyStop() {
             return this.runOnce(() -> {
                this.primaryMotor.stopMotor();
                this.followerMotor.stopMotor();
        });
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
        double currentPosition = encoder.getPosition(); // rotations
        //uses kP, kI, and kD constants to calculate pidOutput
        double pidOutput = pidController.calculate(currentPosition, setpoint);

        double output = pidOutput;
        
        //set limits on the output of the motor
        output = MathUtil.clamp(output, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
        
        SmartDashboard.putNumber("Running Power", applyLimits(output));

        SmartDashboard.putNumber("Motor 1", primaryMotor.getAppliedOutput());
        SmartDashboard.putNumber("Motor 2", followerMotor.getAppliedOutput());
        primaryMotor.set(applyLimits(output));
    }

    /**
     * Set the szetpoint of the elevator, 
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
        boolean isTopPressed = primaryMotor.getReverseLimitSwitch().isPressed();
        boolean isBottomPressed = primaryMotor.getForwardLimitSwitch().isPressed();
        
        if (isBottomPressed) {
            return MathUtil.clamp(input, 0, ElevatorConstants.maxOutput);
        }
        else if (isTopPressed) {
            return MathUtil.clamp(input, -ElevatorConstants.maxOutput, 0);
        }

        if (input < 0) {
            input *= 0.5 * MathUtil.clamp(1 / (Math.abs(encoder.getVelocity()) / 400), 0, 1);
        }

        //return MathUtil.clamp(input, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);

        return 1;
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
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putNumber("velocity", getVelocity());
        SmartDashboard.putNumber("Elevator Error", (setpoint-getPosition()));

        SmartDashboard.putBoolean("Top Limit", primaryMotor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("Bottom Limit", primaryMotor.getReverseLimitSwitch().isPressed());
    }
}