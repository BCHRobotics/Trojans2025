package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;


// import com.revrobotics.spark.config.SparkMaxConfig.follow;

// testing pushing to github
public class Elevator extends SubsystemBase{
    private static Elevator instance = null;

    //DigitalInput toplimitSwitch = new DigitalInput(0);
    //DigitalInput bottomlimitSwitch = new DigitalInput(1);
    //bottomlimitSwitch = kLeftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // LIMIT SWITCH
    private SparkLimitSwitch toplimitSwitch;
    private SparkLimitSwitch bottomlimitSwitch;
    
    // Motors
    private final SparkMax kLeftMotor;
    private final SparkMax kRightMotor;
    
    // SparkMax Configs
    private final SparkMaxConfig kLeftConfig; 
    private final SparkMaxConfig kRightConfig;

    private final double maxVelocity = 500; //1000; // This is in rpm
    private final double maxAcceleration = 50; // 100 This is in rpm/second

    // private final RelativeEncoder kLeftEncoder;

    private final SparkClosedLoopController kLeftController;
    //private double position;
    private double current_encoder_pos;  // the current pos of the encoder, in rotations
    private double offset =0;
    private double motorRotations; // motor rotations to reach setpoint

    public Elevator(){
        this.kLeftMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        this.kRightMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

        this.kLeftConfig = new SparkMaxConfig();
        this.kRightConfig = new SparkMaxConfig();

        this.kLeftConfig.openLoopRampRate(0.05);
        this.kRightConfig.openLoopRampRate(0.05);

        this.kLeftConfig.smartCurrentLimit(60, 20);
        this.kRightConfig.smartCurrentLimit(60, 20);
        // LIMIT SWITCH
        toplimitSwitch = kLeftMotor.getForwardLimitSwitch();
        bottomlimitSwitch = kRightMotor.getReverseLimitSwitch();
        // Enable limit switches to stop the motor when they are closed
        
        kLeftConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen) // setting limit switchs to be normally open, 
        .forwardLimitSwitchEnabled(true); // setting limit switches to be enabled, so they shutdown motor when clicked
        
        // You can only attach one limit switch to a breakout board
        kRightConfig.limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

        //toplimitSwitch = kLeftMotor.getForwardLimitSwitch();
        //toplimitSwitch.EnableLimitSwitch(true);

        //this.kLeftEncoder = kLeftMotor.getEncoder();
        this.kRightConfig.follow(kLeftMotor, true);
        this.kLeftConfig.inverted(true); 

        this.kLeftConfig.idleMode(IdleMode.kBrake);
        this.kRightConfig.idleMode(IdleMode.kBrake);

        this.kLeftController = kLeftMotor.getClosedLoopController();

        this.kLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            Constants.ElevatorConstants.elevatorP,
            Constants.ElevatorConstants.elevatorI,
            Constants.ElevatorConstants.elevatorD);

        this.kRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            Constants.ElevatorConstants.elevatorP,
            Constants.ElevatorConstants.elevatorI,
            Constants.ElevatorConstants.elevatorD);

        this.kLeftConfig.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration)
            .allowedClosedLoopError(0.5);

        this.kRightConfig.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration)
            .allowedClosedLoopError(0.5);
           
        this.kLeftMotor.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.kRightMotor.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.current_encoder_pos = this.getEncoderPos();
        
    }
    
    private void setLeftMotorPos(double pos) {
        // kLeftController.calculate(kLeftEncoder.getPosition(), pos);
        // the amount of rotations the SPROCKET needs to reach the setpoint (pos) 
        double sprocketRotations = pos / ElevatorConstants.kElevatorPulleyCircumference; 
        // the amount of rotations the MOTOR needs to reach the setpoint (pos)
        motorRotations = sprocketRotations * ElevatorConstants.sprocketConversionFactor; 

        this.kLeftController.setReference(
            motorRotations - offset, 
            SparkBase.ControlType.kMAXMotionPositionControl);
    }

    //public Command moveToPosition(double pos) {
    public void moveToPosition(double pos) {
        //return this.runOnce(() -> setLeftMotorPos(pos));
        // subtracting current encoder pos to get the distance needed to travel 
        // Ex. Level 2 --> Level 3 is different from bottom -- > 3 level 3
        setLeftMotorPos(pos - ElevatorConstants.elevatorStowedHeightInches); 
    }

    public void cancelElevatorCommands() {
        this.kLeftMotor.stopMotor();
    }

    public void moveToHomePosition() {
        this.moveToPosition(0);
        this.kLeftMotor.getEncoder().setPosition(0); // setting the encoder positino to zero
    }

    public void calibrate() {
        this.setLeftMotorPos(10);
        while (!bottomlimitSwitch.isPressed()){
            this.setLeftMotorPos(kLeftMotor.getEncoder().getPosition()-1);
        }
        offset = kLeftMotor.getEncoder().getPosition();
        this.setLeftMotorPos(10);
    }
/*  
    private void limitChecking() {
        if ((toplimitSwitch.isPressed() || bottomlimitSwitch.isPressed()) && !) { // if top or bottom limit swtiches are pressed

        }
    }
 */ 
    // get's the encoder's current position in rotations
    public double getEncoderPos() {
        return this.kLeftMotor.getEncoder().getPosition();
    }
    // set the encoder pos
    public Command setEncoderPos(double position) {
        return runOnce(() -> this.kLeftMotor.getEncoder().setPosition(position));
    }

    @Override


    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("Encoder Position", this.position);
        //setLeftMotorPos(10);
        this.current_encoder_pos = this.getEncoderPos(); // getting the encoder pos

        // Display data from SPARK onto the dashboard
        SmartDashboard.putBoolean("Forward Limit Reached", toplimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached", bottomlimitSwitch.isPressed());
        SmartDashboard.putNumber("Applied Output", kLeftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Encoder Position", this.current_encoder_pos);
        
        SmartDashboard.putNumber("Setpoint Rotations", motorRotations); // printing out the refrence
       /* OLD LIMIT SWITCH CODE
        if (toplimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
            kLeftMotor.stopMotor();
        }

        if (bottomlimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
            kLeftMotor.stopMotor(0);
        }
        */

    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
    
}

