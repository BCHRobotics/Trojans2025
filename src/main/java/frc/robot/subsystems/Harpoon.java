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


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;



public class Harpoon extends SubsystemBase{

    private final SparkMax kIntakeMotor; // intake motor is the same as the "shooter" motor
    private final SparkMax kRotationMotor; // rotation motor turns the wrist

    private final SparkMaxConfig kIntakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig kRotationConfig = new SparkMaxConfig();
    private final double maxVelocity = 1000; // This is in rpm
    private final double maxAcceleration = 1000; // This is in rpm/second
    private final DigitalInput harpoonBeamBreaker = new DigitalInput(0);


    // private final RelativeEncoder kLeftEncoder;
    private final SparkClosedLoopController kRotationController;


    private Elevator m_elevator = Elevator.getInstance();

    public Harpoon(){
        this.kIntakeMotor = new SparkMax(Constants.HarpoonConstants.kIntakeMotorCANID, MotorType.kBrushless);
        this.kRotationMotor = new SparkMax(Constants.HarpoonConstants.kRotationMotorCANID, MotorType.kBrushless);


        this.kRotationConfig.inverted(false);
        this.kRotationConfig.idleMode(IdleMode.kBrake);

        this.kIntakeConfig.inverted(false);
        this.kIntakeConfig.idleMode(IdleMode.kBrake);
        
        this.kRotationConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
            Constants.HarpoonConstants.harpoonP,
            Constants.HarpoonConstants.harpoonI,
            Constants.HarpoonConstants.harpoonD);
        
        this.kRotationConfig.closedLoop.maxMotion
            .maxVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration)
            .allowedClosedLoopError(0.5);
        
        this.kRotationController = kRotationMotor.getClosedLoopController();

        this.kRotationMotor.configure(kRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    }

    public double degreesToRotations(double degrees){
        return degrees/360;
    }

    public void setRotationMotorPosition(double positionInDegrees){
        this.kRotationController.setReference(
            degreesToRotations(positionInDegrees)*Constants.HarpoonConstants.gearConversionFactor,
            SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public Command intake(){
        return new InstantCommand(()->{
            kIntakeMotor.set(1);
        }).until(() -> harpoonBeamBreaker.get());

    }
     
    public Command scoreL1(){
        return this.runOnce(
            ()->m_elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPositions.get("L1")))
            .andThen(()->
                setRotationMotorPosition(Constants.HarpoonConstants.L1RotationMotorPositionDegrees)
            )
            .andThen(()->
                kIntakeMotor.set(1)
            )
            .beforeStarting(new WaitCommand(0.5))
            .until(() -> harpoonBeamBreaker.get())
            .andThen(()->
                kIntakeMotor.set(0)
            )
            .andThen(()->
                setRotationMotorPosition(Constants.HarpoonConstants.HomeRotationMotorPositionDegrees)
            )
            .andThen(()->
                m_elevator.moveToHomePosition());
            
            
    }

    public Command scoreL2(){
        return this.runOnce(
            ()->m_elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPositions.get("L2")))
            .andThen(()->
                setRotationMotorPosition(Constants.HarpoonConstants.L2RotationMotorPositionDegrees)
            )
            .andThen(()->
                kIntakeMotor.set(1)
            )
            .beforeStarting(new WaitCommand(0.5))
            .until(() -> harpoonBeamBreaker.get())
            .andThen(()->{
                kIntakeMotor.set(0);
            })
            .andThen(()->
                m_elevator.moveToHomePosition());
            
    }
    public Command scoreL3(){
        return this.runOnce(
            ()->m_elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPositions.get("L3")))
            .andThen(()->
                setRotationMotorPosition(Constants.HarpoonConstants.L3RotationMotorPositionDegrees)
            )
            .andThen(()->
                kIntakeMotor.set(1)
            )
            .beforeStarting(new WaitCommand(0.5))
            .until(() -> harpoonBeamBreaker.get())
            .andThen(()->
                kIntakeMotor.set(0)
            )
            .andThen(()->
                m_elevator.moveToHomePosition()); 
    }
    public Command scoreL4(){
        return this.runOnce(
            ()->m_elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPositions.get("L4")))
            .andThen(()->
                setRotationMotorPosition(Constants.HarpoonConstants.L4RotationMotorPositionDegrees)
            )
            .andThen(()->
                kIntakeMotor.set(1))
            .beforeStarting(new WaitCommand(0.5))
            .until(() -> harpoonBeamBreaker.get())
            .andThen(()->
                kIntakeMotor.set(0)
            )
            .andThen(()->
                m_elevator.moveToHomePosition());
    }
}
