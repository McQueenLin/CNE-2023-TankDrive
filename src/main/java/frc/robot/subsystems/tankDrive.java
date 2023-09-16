// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class tankDrive extends SubsystemBase {

  public CANSparkMax m_leftFrontMotor = RobotContainer.leftFrontMotor;
  public CANSparkMax m_leftBackMotor = RobotContainer.leftRearMotor;
  public static CANSparkMax m_rightFrontMotor = RobotContainer.rightFrontMotor;
  public CANSparkMax m_rightBackMotor = RobotContainer.rightRearMotor;
  
  MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftBackMotor, m_leftFrontMotor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightBackMotor, m_rightFrontMotor);

  public static RelativeEncoder frEncoder = RobotContainer.rightFrontMotor.getEncoder();
  
  private double driveSpeed = 0.035;
  private double degree = 4;
  private double prevVal;
  private double speedLimit = 0.15;
  public static double autoChargeInches; //Community 54", ramp 14", cStation top 76"
  private double inPerEncoder = 2.289; // 19 inches per 8.3 encoder value, one wheel rotation
  public static double distance = 0;
  public static double additionalSpeed = 0.0002;
  private final double START_SPEED = driveSpeed;
  private int polarity = 1;

  private boolean logic2 = false;  
  private boolean logic3 = false;
  private boolean addSpeed = false;
  public boolean centerPassed = false;

  //This is for proportional code *futuer use for swerve*
  private final double INIT_SPEED = 0.035;
  private double activeSpeed = INIT_SPEED; // The speed that is actually ran
  private double proportionalSpeed = 0;
  private final double DEGREE_RANGE = 4;
  private final double SET_POSITION = 97; //inches
  private final double K_CONSTANT = 0;
  private double difference = 0;
  private final double MAX_SPEED = 1;
  private final double MIN_SPEED = 0.2; //example values
  
  /* Creates a new ExampleSubsystem. */
  public tankDrive() {
    m_rightBackMotor.setInverted(true);
    m_rightFrontMotor.setInverted(true);
    leftMotors.setInverted(false);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    SmartDashboard.putNumber("B- Speed", driveSpeed);
    SmartDashboard.putBoolean("B- Passed Center", centerPassed);
    m_rightFrontMotor.set(rightSpeed);
    m_leftFrontMotor.set(leftSpeed);
    m_leftBackMotor.set(leftSpeed);
    m_rightBackMotor.set(rightSpeed);
  }

  public void straight(double speed) {
    tankDrive(speed, speed);
  }

  public void brake(boolean activate) {
    if (activate) {
      m_rightFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_leftFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_rightBackMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_leftBackMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else {
      m_rightFrontMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_leftFrontMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_rightBackMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_leftBackMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }

  public void pivot(double speed, boolean left) {
      if (left) { //left
        tankDrive(-speed, speed);
      } else { //right
        tankDrive(speed, -speed);
      }
  }

  public void stop() {
      tankDrive(0, 0);
  }

  public void balance(double pitch) {
    brake(true);
    if (centerPassed && additionalSpeed > 0.0001) {
      additionalSpeed -= 0.000015;
    } else {
      //additionalSpeed = 0.0002;
    }
      // For future note, intially start with high power to overcome
      // static friction, then lower power to move at constant slow velocity
      //Currently, steadily increase speed until it start moves
    if(Math.abs(pitch) > degree) {
      polarity = (pitch > degree ? 1 : -1);
      straight(driveSpeed * polarity); 

      if (driveSpeed < speedLimit) { // if wheels have not moved, increase speed
        driveSpeed += additionalSpeed;
        addSpeed = true;
      } else {
        addSpeed = false;
      }

      centerPassed = false;      
      SmartDashboard.putBoolean("B- Moving!", true);
    } else {
      driveSpeed = START_SPEED;
      brake(true);
      stop();
      addSpeed = false;
      centerPassed = true;
      SmartDashboard.putBoolean("B- Moving!", false);      
    }
  }

  public void proportionalBalance(double pitch, double roll, double yaw, double activePosition) {
    brake(true);
      //It is possible to assign yaw = 0 at beginning of the game, and know when if the robot
      //drives onto the charging station tilted if yaw is not 0, or roll is not 0 when pitch is tilted

      //Mode one, realtive-distance dependent code
      //only works on charging staion because it's using gryo and proptionality
    straight(activeSpeed);
    difference = SET_POSITION - activePosition; //If behind SP, difference is (+) and pass SP is (-)
    proportionalSpeed = INIT_SPEED * difference*K_CONSTANT; //This calculation is always running, is applied when necessary
    
    if (pitch > DEGREE_RANGE && pitch < -DEGREE_RANGE) {         
      activeSpeed = proportionalSpeed;
      activeSpeed = (activeSpeed > MAX_SPEED ? MAX_SPEED : proportionalSpeed);
      activeSpeed = (activeSpeed < MIN_SPEED ? MIN_SPEED : proportionalSpeed);
        //activeSpeed must never start increasing from 0 velocity, it won't be strong enough
        //there must be a minimum moving speed, as executed above           
    } else {
      activeSpeed = 0;
    }
  }

  public void autoChargeStation() {
        distance = frEncoder.getPosition() * inPerEncoder;
        
        if (Math.abs(distance) < 175 && !logic2) { //175 pass and over charge station
          if (Math.abs(distance) < 90) { //High speed until near mid of charge
            straight(-0.5);
          } else {
            straight(-0.2); //Don't want bot flying off the station
          }            
      
        } else {
          logic2 = true; //Next logic acivated
          if (!logic3 && Math.abs(distance) > autoChargeInches) { //102, onto charge station from mid field side
            straight(0.5); //high power to get on
          } else {
            logic3 = true;
            balance(Robot.pitch); //auto balance
          }
        }
        
  }
}