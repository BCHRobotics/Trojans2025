package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;


public class Harpoon extends SubsystemBase{

    private final SparkMax kIntakeMotor; // intake motor is the same as the "shooter" motor
    private final SparkMax kRotationMotor; // rotation motor turns the wrist
    private final RelativeEncoder kIntakeEncoder;
    private final SparkMaxConfig kIntakeConfig = new SparkMaxConfig();
    private final double maxVelocity = 1000; // This is in rpm
    private final double maxAcceleration = 1000; // This is in rpm/second
    private final DigitalInput harpoonBeamBreaker = new DigitalInput(0);
    private double position;

    // private final RelativeEncoder kLeftEncoder;
    private final SparkClosedLoopController kRotationController;
    private double rotationMotorPosition;

    private Elevator m_elevator = Elevator.getInstance();

    public Harpoon(){
        this.kIntakeMotor = new SparkMax(Constants.HarpoonConstants.kIntakeMotorCANID, MotorType.kBrushless);
        this.kRotationMotor = new SparkMax(Constants.HarpoonConstants.kRotationMotorCANID, MotorType.kBrushless);

        this.kIntakeEncoder = kIntakeMotor.getEncoder();
        this.kIntakeConfig.inverted(false);
        this.kIntakeConfig.idleMode(IdleMode.kBrake);

        this.kRotationController = kRotationMotor.getClosedLoopController();
        //this.position = this.kRotationMotor.getPosition();
        
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
