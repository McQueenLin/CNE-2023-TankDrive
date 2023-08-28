package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;



public class Hand extends SubsystemBase{
    public CANSparkMax HandMotor = RobotContainer.handMotor;
    public RelativeEncoder HandMotorEncoder = HandMotor.getEncoder();
    public double handPosition;

    public static double openHandPosition = 0;
    public static double HandPositionForClose = 2;
    public static double holdSpeed = 0.1;
    double currentPosition;
    //DigitalInput sensorInput = new DigitalInput(1);

    // private static boolean hasHandSwitchDevice = false;
    //Timer timer = new Timer();
  
    public static final double motorSpeed = 0.15;
    public static final double lastPosition = 0;
    public static boolean hold = false;
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
        
    }


    // public Command Nothing(){
    //     return new InstantCommand(() -> {
    //         HandMotor.set(0);
    //     });
    // }

    public Command Opening(){
        
        return runOnce(() -> {
            while(true){
                currentPosition = HandMotorEncoder.getPosition();
                if (openHandPosition < currentPosition) {
                    HandMotor.set(motorSpeed*-1);
                    holdSpeed = -0.15;
                    hold = false;
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
            HandMotor.set(holdSpeed);
            hold = true;
            });
        }
    
    public Command Holding(){
        return runOnce(() -> {
            if(hold == true){
                holdSpeed = 0.1;
                HandMotor.set(holdSpeed);
            } else{
                holdSpeed = 0;
                HandMotor.set(0);
            }
            
        });
    }

    
        

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hand Position", HandMotorEncoder.getPosition()); 
        SmartDashboard.putNumber("Speed", holdSpeed);
    }
}
