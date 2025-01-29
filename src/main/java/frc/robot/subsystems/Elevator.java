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

    private final double maxVelocity = 1000; // This is in rpm
    private final double maxAcceleration = 1000; // This is in rpm/second

    // private final RelativeEncoder kLeftEncoder;

    private final SparkClosedLoopController kLeftController;
    private double position;
    private double offset;


    public Elevator(){
        this.kLeftMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        this.kRightMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

        this.kLeftConfig = new SparkMaxConfig();
        this.kRightConfig = new SparkMaxConfig();
        // LIMIT SWITCH
        toplimitSwitch = kLeftMotor.getForwardLimitSwitch();
        bottomlimitSwitch = kLeftMotor.getReverseLimitSwitch();
        // Enable limit switches to stop the motor when they are closed
        kLeftConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen) // setting limit switchs to be normally open, 
        .forwardLimitSwitchEnabled(true) // setting limit switches to be enabled, so they shutdown motor when clicked
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

        //toplimitSwitch = kLeftMotor.getForwardLimitSwitch();
        //toplimitSwitch.EnableLimitSwitch(true);

        //this.kLeftEncoder = kLeftMotor.getEncoder();
        this.kRightConfig.follow(kLeftMotor, true);

        this.kLeftConfig.inverted(false);

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

        this.position = kLeftMotor.getEncoder().getPosition();
    }

    private void setLeftMotorPos(double pos) {
        // kLeftController.calculate(kLeftEncoder.getPosition(), pos);
        this.kLeftController.setReference(
            (pos*ElevatorConstants.gearConversionFactor)/(2*Math.PI*Constants.ElevatorConstants.kElevatorPulleyRadius), 
            SparkBase.ControlType.kMAXMotionPositionControl);
    }

    //public Command moveToPosition(double pos) {
    public void moveToPosition(double pos) {
        //return this.runOnce(() -> setLeftMotorPos(pos));
        setLeftMotorPos(pos-offset);
    }

    public void cancelElevatorCommands() {
        this.kLeftMotor.stopMotor();
    }

    public void moveToHomePosition() {
        this.moveToPosition(0);
    }
/* OLD LIMIT SWITCH CODE
    public void calibrate() {
        while (!bottomlimitSwitch.get()){
            this.setLeftMotorPos(-1);
        }
        offset = kLeftMotor.getEncoder().getPosition();
    }*/
/*  
    private void limitChecking() {
        if ((toplimitSwitch.isPressed() || bottomlimitSwitch.isPressed()) && !) { // if top or bottom limit swtiches are pressed

        }
    }
 */  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Encoder Position", this.position);
        //setLeftMotorPos(10);

        // Display data from SPARK onto the dashboard
        SmartDashboard.putBoolean("Forward Limit Reached", toplimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached", bottomlimitSwitch.isPressed());
        SmartDashboard.putNumber("Applied Output", kLeftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Position", kLeftMotor.getEncoder().getPosition());
       
       /* OLD LIMIT SWITCH CODE
        if (toplimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
            kLeftMotor.set(0);
        }

        if (bottomlimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
            kLeftMotor.set(0);
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

