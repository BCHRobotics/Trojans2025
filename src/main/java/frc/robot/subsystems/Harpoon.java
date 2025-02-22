package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;



public class Harpoon extends SubsystemBase{

    private final SparkMax kIntakeMotor; // intake motor is the same as the "shooter" motor
    private final SparkMax kRotationMotor; // rotation motor turns the wrist

    private final SparkMaxConfig kIntakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig kRotationConfig = new SparkMaxConfig();
    private final double maxVelocity = 100; // This is in rpm
    private final double maxAcceleration = 100; // This is in rpm/second 


    // private final RelativeEncoder kLeftEncoder;
    private final SparkClosedLoopController kRotationController;
    private final SparkLimitSwitch sensorLimitSwitch;

    // private Elevator m_elevator = Elevator.getInstance(); This method is not defined in this code base

    public Harpoon(){
        // set up the motors
        this.kIntakeMotor = new SparkMax(Constants.HarpoonConstants.kIntakeMotorCANID, MotorType.kBrushless);
        this.kRotationMotor = new SparkMax(Constants.HarpoonConstants.kRotationMotorCANID, MotorType.kBrushless);

        this.sensorLimitSwitch = kRotationMotor.getForwardLimitSwitch();
        LimitSwitchConfig sensorConfig = new LimitSwitchConfig();
        sensorConfig.forwardLimitSwitchType(Type.kNormallyClosed);

        // important configurations. Idlemode is just the mode the sensor is in when it is not being commanded. 
        this.kRotationConfig.inverted(false); // 
        this.kRotationConfig.idleMode(IdleMode.kBrake); // IdleMode.kBrake

        // super helpful position conversion factor. If using REV's maxmotion, this is important because it eliminates the need to adjust the setpoints based on the converion, making the code less elusive
        this.kRotationConfig.encoder.positionConversionFactor(
            Constants.HarpoonConstants.gearConversionFactor*(Math.PI/180));

        // more important configs
        this.kIntakeConfig.inverted(false); // inverting intake motor
        this.kIntakeConfig.idleMode(IdleMode.kBrake);

        // closed loop controller for the rotation motor - we're using a pid feedforward controller
        this.kRotationConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pidf(
            Constants.HarpoonConstants.harpoonP,
            Constants.HarpoonConstants.harpoonI,
            Constants.HarpoonConstants.harpoonD,
            0);
        
        // maxmotion! This is a really cool feature that REV has. It allows you to set the max velocity and acceleration of the motor. This is super helpful for tuning the motor.
        this.kRotationConfig.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration)
            .allowedClosedLoopError(0.5);

        
        // get the controller for the rotation motor
        this.kRotationController = kRotationMotor.getClosedLoopController();
        this.kRotationConfig.apply(sensorConfig);

        // finally, configure the motors
        this.kRotationMotor.configure(this.kRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double degreesToRotations(double degrees){
        return degrees/360;
    }

    public void setRotationMotorPosition(double positionInDegrees){
        kRotationController.setReference(
            degreesToRotations(positionInDegrees),
            SparkBase.ControlType.kMAXMotionPositionControl);
        SmartDashboard.putNumber("Desired Setpoint", positionInDegrees);
        
    }

    public void setIntakeMotorVelocity(double velocity){
        kIntakeMotor.set(velocity);
    }

    public Command emergencyStop() {
        return this.runOnce(() -> {
            kRotationController.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl);
            kRotationMotor.stopMotor();
            kIntakeMotor.stopMotor();
        });
    }

    public boolean isCoralDetected() {
        return this.sensorLimitSwitch.isPressed();  // Returns true if sensor is triggered
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", kRotationMotor.getAbsoluteEncoder().getPosition());
        
        // we gotta get SmartDashboard sorted since there's already a lot of stuff being sent to it
    }
}