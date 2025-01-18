package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;


// import com.revrobotics.spark.config.SparkMaxConfig.follow;


public class Elevator extends SubsystemBase{

    private final SparkMax kLeftMotor;
    private final SparkMax kRightMotor;

    private final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig kRightConfig = new SparkMaxConfig();

    private final double maxVelocity = 100; // This is in rpm
    private final double maxAcceleration = 100; // This is in rpm/second

    // private final RelativeEncoder kLeftEncoder;

    private final SparkClosedLoopController kLeftController;

    private double position;


    public Elevator(){
        this.kLeftMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        this.kRightMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);
        

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

        this.kRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(Constants.ElevatorConstants.elevatorP,
            Constants.ElevatorConstants.elevatorI,
            Constants.ElevatorConstants.elevatorD);

        this.kLeftConfig.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration);

        this.kRightConfig.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration);
           
        this.kLeftMotor.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.kRightMotor.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.position = kLeftMotor.getEncoder().getPosition();
    }

    private void setLeftMotorPos(double pos) {
        // kLeftController.calculate(kLeftEncoder.getPosition(), pos);
        this.kLeftController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public Command moveToPosition(double pos) {
        return this.runOnce(() -> setLeftMotorPos(pos));
    }

    public void cancelElevatorCommands() {
        this.kLeftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Encoder Position", this.position);
    }
    
}

