package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase{

    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;

    private double setpoint = 0; // rotations
    double currentPos;

    public Elevator() {
        
        //Creating new motors, encoders, and PID controllers
        //motors
        primaryMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        
        //encoder
        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);
        
        //PID controller
        pidController = new PIDController(
            ElevatorConstants.ElevatorkP,
            ElevatorConstants.ElevatorkI,
            ElevatorConstants.ElevatorkD
        );
        
        pidController.setTolerance(2.5); // orignally 0.5, rotations
        
        //setting limits for safety
        SparkMaxConfig resetConfig = new SparkMaxConfig();
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        
        //reseting factory defaults
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        
        //configuring follower motor (follower follow main)
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);
        followerMotor.configure(followerConfig, null, null); 
    }
    
    //run this when the elevatory is at the bottom
    private void handleBottomLimit() {
        //requires bottom limit switch
        if(!bottomLimit.get()) {
            stopMotors();
            encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch);
            setpoint = ElevatorConstants.bottomPos;
            
            if (setpoint == ElevatorConstants.bottomPos) {
                pidController.reset();
        }
    }
    } 
    
    public void setTargetPosition(double positionInches) {
        //set limits on the target position
        setpoint = positionInches;
        /* 
        
        MathUtil.clamp(
            positionInches, 
            ElevatorConstants.bottomPos, 
            ElevatorConstants.topPos);
        */
        
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    @Override
    public void periodic() {
        run();
    }

    public void run() {
        double currentPosition = encoder.getPosition() * ElevatorConstants.countsPerInch; // inches
        //uses kP, kI, and kD constants to calculate pidOutput
        double pidOutput = pidController.calculate(currentPosition, setpoint);

        double feedForward = ElevatorConstants.feedForward*Math.signum(setpoint-currentPosition);
        double output = pidOutput + feedForward;
        
        //set limits on the output of the motor
        output = MathUtil.clamp(output, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput);
        
        primaryMotor.set(output);

        handleBottomLimit();

        //printing data onto FRC Driver Station
        System.out.println("Encoder Position" + encoder.getPosition());
        System.out.println("Elevator Position" + currentPosition);
        System.out.println("Elevator Setpoint" +  setpoint);
        System.out.println("Elevator PID Output" + output);
        System.out.println("Elevator Error" + (setpoint-currentPosition));
    }

    //setting target position based on configured button binding
    public Command moveLevel(String level) {
        return runOnce(()-> {
            switch (level) {
                case "L1":
                    this.setTargetPosition(ElevatorConstants.L1);
                    break;   
                case "L2":
                    this.setTargetPosition(ElevatorConstants.L2);
                    break;
                case "L3":
                    this.setTargetPosition(ElevatorConstants.L3);
                    break;
                case "DOWN":
                    this.setTargetPosition(ElevatorConstants.bottomPos);
                    break;
                case "L0":
                    stopMotors();
                    break;
            }
        });
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

}