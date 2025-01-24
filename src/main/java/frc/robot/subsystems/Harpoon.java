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


public class Harpoon extends SubsystemBase{

    private final SparkMax kLeftMotor;
    private final SparkMax kRightMotor;

    private final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig kRightConfig = new SparkMaxConfig();

    private final double maxVelocity = 1000; // This is in rpm
    private final double maxAcceleration = 1000; // This is in rpm/second

    // private final RelativeEncoder kLeftEncoder;

    private final SparkClosedLoopController kLeftController;

    private double position;

    public Harpoon(){
        this.kLeftMotor = new SparkMax(Constants.HarpoonConstants.kLeftHarpoonMotorCanId, MotorType.kBrushless);
        this.kRightMotor = new SparkMax(Constants.HarpoonConstants.kRightHarpoonMotorCanId, MotorType.kBrushless);
        

        //this.kLeftEncoder = kLeftMotor.getEncoder();
        this.kRightConfig.follow(kLeftMotor, true);

        this.kLeftConfig.inverted(false);

        this.kLeftConfig.idleMode(IdleMode.kBrake);
        this.kRightConfig.idleMode(IdleMode.kBrake);

        this.kLeftController = kLeftMotor.getClosedLoopController();
    }

}
