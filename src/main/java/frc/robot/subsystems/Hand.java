package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.fasterxml.jackson.core.JsonToken;

import java.util.concurrent.Delayed;
import java.util.concurrent.atomic.AtomicReference;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import java.util.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;



public class Hand extends SubsystemBase{
    public CANSparkMax HandMotor = RobotContainer.handMotor;
    public RelativeEncoder HandMotorEncoder = HandMotor.getEncoder();
    public double handPosition;

    public static final DigitalInput photoSwitch = new DigitalInput(0);

    public static double openHandPosition = 0.15;
    public static double CloseHandPosition = 3.8;
    public static double holdSpeed = 0.1;
    public static double motorSpeed;
    double currentPosition;
    boolean photoSwitchOn = false;
    
    
    //DigitalInput sensorInput = new DigitalInput(1);

    // private static boolean hasHandSwitchDevice = false;
    //Timer timer = new Timer();
  
    
    public static final double lastPosition = 0;
    public static boolean hold = false;
    boolean autoclose = false;
    // public static final double CUBE_CLOSING_FORCE = 0.8;
    // public static final double CONE_CLOSING_FORCE = 1.1



    private final static Hand INSTANCE = new Hand();

    /**
     * Returns the Singleton instance of this Hand. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code Hand.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Hand getInstance() {
        return INSTANCE;
    }

    private Hand(){
        HandMotorEncoder.setPosition(0);
        HandMotor.set(0);
        HandMotor.getPIDController().setP(0.7);
        SmartDashboard.putNumber("Motor speed", 0.5);
        
    }


    // public Command Nothing(){
    //     return new InstantCommand(() -> {
    //         HandMotor.set(0);
    //     });
    // }

    public Command Opening(){
        //Test to see if the hand will reclose after opening before the item is placed
        return runOnce(() -> {
            while(true){
                currentPosition = HandMotorEncoder.getPosition();
                if (currentPosition > openHandPosition) {
                    HandMotor.set(-0.1);
                    holdSpeed = -0.1;
                    hold = false;
                    photoSwitchOn = false;
                } 
                else {
                    HandMotor.set(0);
                    holdSpeed = 0;
                    break;
                 }
            }     
        });
    }

    

    public Command Closing(){
        return runOnce( () -> {
            // while(true){
            // double currentPosition = HandMotorEncoder.getPosition();
                // if (currentPosition < HandPositionForClose) {
            holdSpeed = 0.5;
            
            HandMotor.set(motorSpeed);
            hold = true;
            });
        }
    
    public Command Holding(){
        return runOnce(() -> {
            if(hold == true){
                holdSpeed = 0.05;
                HandMotor.set(holdSpeed);
            } else{
                holdSpeed = 0;
                HandMotor.set(0);
            }
            
        });
    }

    public Command setFalse(){
        return runOnce(() -> {
            hold = false;
        });
    }

    public Command setSensorOn(){
        return runOnce(() -> {
            photoSwitchOn = true;
        });
    }

    public Command setSensorOff(){
        return runOnce(() -> {
            photoSwitchOn = false;
        });
    }

    public Command autoClose(){
        return runOnce(() -> {
            if(!photoSwitch.get() || photoSwitchOn == true){
                photoSwitchOn = true;
                currentPosition = HandMotorEncoder.getPosition();
                if (currentPosition < CloseHandPosition) {
                    HandMotor.set(motorSpeed);
                    hold = false;
                }
                // else if(currentPosition > 6){
                //     while(true){
                //         if (currentPosition > openHandPosition) {
                //             HandMotor.set(-0.1);
                //             hold = false;
                //         } 
                //         else {
                //             HandMotor.set(0);
                //             holdSpeed = 0;
                //             break;
                //          }
                //     }    
                // }
                else{
                    holdSpeed = 0.05;
                    SmartDashboard.putString("HOLDING", "HOLDING");
                    HandMotor.set(0.05);
                }
            } else{
                HandMotor.set(0);
            }
        });
    }



    @Override
    public void periodic() {
        motorSpeed = SmartDashboard.getNumber("Motor speed", 0.5);
        SmartDashboard.putNumber("Hand Temperature", HandMotor.getMotorTemperature());
        SmartDashboard.putBoolean("Sensor", !photoSwitch.get());
        SmartDashboard.putNumber("Hand Position", HandMotorEncoder.getPosition()); 
        SmartDashboard.putNumber("Speed", holdSpeed);


        
    }
}
