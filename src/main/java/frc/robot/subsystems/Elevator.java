package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
//import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;

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
        Constants.ElevatorConstants.elevatorD
        );


    public Elevator(){
        this.kLeftMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        this.kRightMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);
        

        this.kLeftEncoder = kLeftMotor.getEncoder();
        
        //SparkBaseConfigAccessor kRightAccessor = new SparkBaseConfigAccessor();
        //kRightConfig = new SparkBaseConfig();
       
        

        
    }
 
}

