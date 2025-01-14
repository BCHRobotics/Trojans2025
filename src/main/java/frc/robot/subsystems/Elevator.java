package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

// import com.revrobotics.spark.config.SparkMaxConfig.follow;


public class Elevator extends SubsystemBase{
    

    private final SparkMax kLeftMotor;
    private final SparkMax kRightMotor;

    private final RelativeEncoder kLeftEncoder;

    private PIDController controller = new PIDController(
        Constants.ElevatorConstants.elevatorP,
        Constants.ElevatorConstants.elevatorI,
        Constants.ElevatorConstants.elevatorD);


    public Elevator(){
        kLeftMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        kRightMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

        kLeftEncoder = kLeftMotor.getEncoder();

        this.kLeftMotor.restoreFactoryDefaults();
        this.kRightMotor.restoreFactoryDefaults();

        this.kRightMotor.follow(kLeftMotor, true);

    }
 
}

