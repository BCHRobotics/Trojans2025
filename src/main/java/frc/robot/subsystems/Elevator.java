package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechanismMode;
import frc.robot.Constants.MechanismPosition;

/*
 * Subsystem for managing elevator movement
 */
public class Elevator extends SubsystemBase{

    // there is no follower config, the motors are both driven
    
    // these are the SparkMaxs
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;

    // the encoder of the primary motor
    public final RelativeEncoder encoder;
    // the PID controller used to move the elevator
    private final PIDController pidController;

    // the current setpoint of the elevator
    private double setpoint = 0; // rotations

    // pre-selected position
    private double selectedPosition;
    // pre-selected mode
    private MechanismMode nextMode;

    // currently running mode
    private MechanismMode currentMode;

    private boolean runCalibration;

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
        motorConfig.smartCurrentLimit(60);
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
        primaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.setpoint = MechanismPosition.STOWED.getElevatorSetpoint();  
    } 

    public void driveMotorSlow(){
        primaryMotor.set(-0.1);
        followerMotor.set(-0.1);
    }

    public void stopMotor(){
        primaryMotor.set(0);
        followerMotor.set(0);
    }

    public double getSetpoint(){
        return setpoint;
    }

    // returns the position 1 HIGHER than the current one
    public double getUpperPosition() {
        if (selectedPosition == MechanismPosition.L1.getElevatorSetpoint()) {
            return MechanismPosition.L2.getElevatorSetpoint();
        }
        else if (selectedPosition == MechanismPosition.L2.getElevatorSetpoint()) {
            return MechanismPosition.L3.getElevatorSetpoint();
        } else if (selectedPosition == MechanismPosition.L3.getElevatorSetpoint()) {
            return MechanismPosition.L4.getElevatorSetpoint();
        } else if (selectedPosition == MechanismPosition.L4.getElevatorSetpoint()) {
            return MechanismPosition.L1.getElevatorSetpoint();
        }   

        // this should never happen, in theory
        return MechanismPosition.L1.getElevatorSetpoint();
    }

    // returns the position 1 LOWER than the current one
    public double getLowerPosition() {
        if (selectedPosition == MechanismPosition.L1.getElevatorSetpoint()) {
            return MechanismPosition.L4.getElevatorSetpoint();
        }
        else if (selectedPosition == MechanismPosition.L2.getElevatorSetpoint()) {
            return MechanismPosition.L1.getElevatorSetpoint();
        } else if (selectedPosition == MechanismPosition.L3.getElevatorSetpoint()) {
            return MechanismPosition.L2.getElevatorSetpoint();
        } else if (selectedPosition == MechanismPosition.L4.getElevatorSetpoint()) {
            return MechanismPosition.L3.getElevatorSetpoint();
        }   

        // this should never happen, in theory
        return MechanismPosition.L1.getElevatorSetpoint();
    }

    public void activateCalibration() {
        runCalibration = true;
    }

    public void stopCalibration() {
        runCalibration = false;
        resetElevator();
    }

    public void setNextMode(MechanismMode mode) {
        nextMode = mode;
    }

    public MechanismMode getNextMode() {
        return nextMode;
    }

    public void setMode(MechanismMode mode) {
        currentMode = mode;
    }

    public MechanismMode getMode() {
        return currentMode;
    }

    public void setSelectedPosition(double position) {
        selectedPosition = position;
    }

    public double getSelectedPosition() {
        return selectedPosition;
    }

    public void resetElevator() {
        // set the position of the encoder to 0, since the elevator should be resting at the bottom
        // THIS CAUSES ISSUES IF YOU DEPLOY WHILE THE ELEVATOR IS UP, SAME AS LAST YEAR
        encoder.setPosition(0);
        setpoint = 0;

        currentMode = MechanismMode.STOWED;
        nextMode = MechanismMode.REEF;
        selectedPosition = MechanismPosition.L4.getElevatorSetpoint();
    }

    public void setSetpoint(double setpoint){
        System.out.println("Setting Elevator Setpoint");
        this.setpoint = setpoint; 
    }

    public boolean isBottomPressed(){
        return primaryMotor.getForwardLimitSwitch().isPressed();
    }



    @Override 
    public void periodic() {
        if (currentMode == MechanismMode.REEF) {
            // if we're scoring, use the selected position
            // this allows the operator to switch scoring positions immediately
            setpoint = selectedPosition;
        }
        else if (currentMode == MechanismMode.FEEDER) {
            // for intaking, use the constant
            // this allows the operator to pre-select a mode without the elevator moving
            setpoint = MechanismPosition.INTAKE.getElevatorSetpoint();
        }
        else if (currentMode == MechanismMode.STOWED) {
            // ditto with stowed, use the constant for the same reason
            setpoint = MechanismPosition.STOWED.getElevatorSetpoint();
        }

        // moving the elevator to the desired setpoint
        
        if (!runCalibration){
            drive();
        }

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
    
        primaryMotor.set(applyLimits(output));
        followerMotor.set(applyLimits(output));
    }

    /**
     * Applies limit switch logic and velocity limits to the output to be fed to the motors
     * @param input The raw output, only affected by PID (no limit switches or anything yet)
     * @return The transformed output, ready to be passed to the motors
     */
    double applyLimits(double input) {
        boolean isTopPressed = primaryMotor.getReverseLimitSwitch().isPressed();

        // no bottom switch rn
        //boolean isBottomPressed = primaryMotor.getForwardLimitSwitch().isPressed();
        // if (isBottomPressed) {
        //     return MathUtil.clamp(input, 0, ElevatorConstants.maxOutput);
        // }


        if (isTopPressed) {
            return MathUtil.clamp(input, -ElevatorConstants.maxOutput, 0);
        }

        // double maxVelocity = 300;

        // input *= 1 / (1 + MathUtil.clamp((Math.abs(encoder.getVelocity()) - 300) / 100, 0, 1));

        return MathUtil.clamp(input, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
    }

    /**
     * Grab the current position from the encoder, 
     * it's a separate function so that the value can be transformed
     * @return The position, in rotations (I think)
     */
    public double getEncoderPosition() {
        return encoder.getPosition()*1.0*ElevatorConstants.countsPerInch;
    }

    /**
     * Grab the current velocity from the encoder, 
     * it's a separate function so that the value can be transformed
     * @return The velocity, in rad/s (I think)
     */
    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    /**
     * A space to put periodically updated debug values that need to be put to dashboard
     * This is called in periodic()
     */
    public void printToDashboard() {
        SmartDashboard.putNumber("nPos", setpoint == 0 ? selectedPosition : 0);
        SmartDashboard.putNumber("cPos", setpoint);
    }
}