package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

// import com.revrobotics.spark.config.SparkMaxConfig.follow;


public class Elevator extends SubsystemBase{

    private final SparkMax kLeftMotor;
    private final SparkMax kRightMotor;

    private final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig kRightConfig = new SparkMaxConfig();

    private final RelativeEncoder kLeftEncoder;

    private PIDController controller = new PIDController(
        Constants.ElevatorConstants.elevatorP,
        Constants.ElevatorConstants.elevatorI,
        Constants.ElevatorConstants.elevatorD
        );


    public Elevator(){
        this.kLeftMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        this.kRightMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);
        

        this.kLeftEncoder = kLeftMotor.getEncoder();

        kLeftConfig.inverted(true);
        kRightConfig.inverted(false);

        kLeftConfig.idleMode(IdleMode.kBrake);
        kLeftConfig.idleMode(IdleMode.kBrake);

        kLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);
        kRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);

        this.kLeftMotor.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.kRightMotor.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


       
        

        
    }
 
}

