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
    public CANSparkMax HandMotor = new CANSparkMax(Constants.OperatorConstants.handMotor, MotorType.kBrushless);
    public RelativeEncoder HandMotorEncoder = HandMotor.getEncoder();
    public double handPosition;

    public static double openHandPosition = -1.0;
    public static double HandPositionForCone = 2;
    public static double handPositionForCube = 2;
    //DigitalInput sensorInput = new DigitalInput(1);

    // private static boolean hasHandSwitchDevice = false;
    //Timer timer = new Timer();
  
    public static final double motorSpeed = 0.15;
    // public static final double CUBE_CLOSING_FORCE = 0.8;
    // public static final double CONE_CLOSING_FORCE = 1.1

    public Hand(){
        HandMotorEncoder.setPosition(0);
        
    }

    // public Command Nothing(){
    //     return new InstantCommand(() -> {
    //         HandMotor.set(0);
    //     });
    // }

    public Command Opening(){
        double currentPosition = HandMotorEncoder.getPosition();
        return new InstantCommand(() -> {
            while(true){
                if (openHandPosition < currentPosition) {
                    HandMotor.set(motorSpeed * (-1.1));
                } 
                else {
                    HandMotor.set(0);
                    break;
                 }
            }     
        });
    }

    public Command Closing(){
        return runOnce( () -> {
            while(true){
                double currentPosition = HandMotorEncoder.getPosition();
                if (currentPosition < handPositionForCube) {
                    HandMotor.set(0.95 * motorSpeed);
                } else {
                    HandMotor.set(0);
                    break;
                }
    
            }    
        });
    }
        

    @Override
    public void periodic() {
        //double currentPosition = HandMotorEncoder.getPosition();
    }
}
