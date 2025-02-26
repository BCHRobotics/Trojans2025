package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HarpoonConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.HarpoonConstants.HarpoonMode;
import frc.robot.Constants.HarpoonConstants.HarpoonPosition;
public class Harpoon extends SubsystemBase{

    private final SparkMax kIntakeMotor; // intake motor is the same as the "shooter" motor
    private final SparkMax kRotationMotor; // rotation motor turns the wrist

    private final SparkMaxConfig kIntakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig kRotationConfig = new SparkMaxConfig();
    private final double maxVelocity = 600; // This is in rpm
    private final double maxAcceleration = 600; // This is in rpm/second 

    private final SparkLimitSwitch sensorLimitSwitch;

    private final PIDController pidController;

    private double setpoint;

    // pre-selected setpoint
    private double selectedSetpoint;

    // currently active mode
    private HarpoonMode currentMode;
    // mode to activate next
    private HarpoonMode nextMode;

    public Harpoon(){

        //PID controller
        pidController = new PIDController(
            HarpoonConstants.harpoonP,
            HarpoonConstants.harpoonI,
            HarpoonConstants.harpoonD
        );

        // set up the motors
        this.kIntakeMotor = new SparkMax(Constants.HarpoonConstants.kIntakeMotorCANID, MotorType.kBrushless);
        this.kRotationMotor = new SparkMax(Constants.HarpoonConstants.kRotationMotorCANID, MotorType.kBrushless);

        this.sensorLimitSwitch = kRotationMotor.getForwardLimitSwitch();
        LimitSwitchConfig sensorConfig = new LimitSwitchConfig();
        sensorConfig.forwardLimitSwitchType(Type.kNormallyOpen);

        // important configurations. Idlemode is just the mode the sensor is in when it is not being commanded. 
        this.kRotationConfig.inverted(false); // 
        this.kRotationConfig.idleMode(IdleMode.kBrake);

        // more important configs
        this.kIntakeConfig.inverted(false); // inverting intake motor
        this.kIntakeConfig.idleMode(IdleMode.kCoast);
        
        this.kRotationConfig.apply(sensorConfig);

        // finally, configure the motors
        this.kRotationMotor.configure(this.kRotationConfig, null, PersistMode.kPersistParameters);
        this.kIntakeMotor.configure(this.kIntakeConfig, null, PersistMode.kPersistParameters);
    }

    public void resetHarpoon() {
        setSelectedPosition(HarpoonPosition.L4.getSetpoint());
        setNextMode(HarpoonMode.REEF);

        setMode(HarpoonMode.STOWED);
    }
    
    public double getUpperSetpoint() {
        if (selectedSetpoint == HarpoonPosition.L1.getSetpoint()) {
            return HarpoonPosition.L2.getSetpoint();
        }
        else if (selectedSetpoint == HarpoonPosition.L2.getSetpoint()) {
            return HarpoonPosition.L3.getSetpoint();
        } 
        else if (selectedSetpoint == HarpoonPosition.L3.getSetpoint()) {
            return HarpoonPosition.L4.getSetpoint();
        } 
        else if (selectedSetpoint == HarpoonPosition.L4.getSetpoint()) {
            return HarpoonPosition.L1.getSetpoint();
        }

        return HarpoonPosition.L1.getSetpoint();
    }

    public double getLowerSetpoint() {
        if (selectedSetpoint == HarpoonPosition.L1.getSetpoint()) {
            return HarpoonPosition.L4.getSetpoint();
        }
        else if (selectedSetpoint == HarpoonPosition.L2.getSetpoint()) {
            return HarpoonPosition.L1.getSetpoint();
        } 
        else if (selectedSetpoint == HarpoonPosition.L3.getSetpoint()) {
            return HarpoonPosition.L2.getSetpoint();
        } 
        else if (selectedSetpoint == HarpoonPosition.L4.getSetpoint()) {
            return HarpoonPosition.L3.getSetpoint();
        }

        return HarpoonPosition.L1.getSetpoint();
    }

    public void setNextMode(HarpoonMode mode) {
        nextMode = mode;
    }

    public HarpoonMode getNextMode() {
        return nextMode;
    }

    public void setMode(HarpoonMode mode) {
        currentMode = mode;
    }

    public HarpoonMode getMode() {
        return currentMode;
    }

    public void setSelectedPosition(double position) {
        selectedSetpoint = position;
    }

    public double getSelectedPosition() {
        return selectedSetpoint;
    }
 
    public void setRotationMotorPosition(double HarpoonPosition){ // HarpoonPosition is 0.6-1, (1 is stowed, 0.6 is reaching bumpers)
        setpoint = HarpoonPosition;
    }

    public void setIntakeMotorVelocity(double velocity){
        kIntakeMotor.set(velocity);
    }

    public boolean isCoralDetected() {
        return this.sensorLimitSwitch.isPressed();  // Returns true if sensor is triggered
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", kRotationMotor.getAbsoluteEncoder().getPosition());

        SmartDashboard.putString("CURRENT MODE", currentMode.toString());
        SmartDashboard.putString("NEXT MODE", nextMode.toString());
        SmartDashboard.putNumber("NEXT SETPOINT", selectedSetpoint);

        if (currentMode == HarpoonMode.REEF) {
            // if we're scoring, use the selected position
            // this allows the operator to switch scoring positions immediately
            setRotationMotorPosition(selectedSetpoint);
        }
        else if (currentMode == HarpoonMode.FEEDER) {
            // for intaking, use the constant
            // this allows the operator to pre-select a mode without the elevator moving
            setRotationMotorPosition(HarpoonPosition.INTAKE.getSetpoint());
        }
        else if (currentMode == HarpoonMode.STOWED) {
            // ditto with stowed, use the constant for the same reason
            setRotationMotorPosition(HarpoonPosition.STOWED.getSetpoint());
        }

        // if (kRotationMotor.getAbsoluteEncoder().getPosition() < 0.5) {
        //     setRotationMotorPosition(-0.001);
        // }

        kRotationMotor.set(pidController.calculate(kRotationMotor.getAbsoluteEncoder().getPosition(), setpoint));
    }
}