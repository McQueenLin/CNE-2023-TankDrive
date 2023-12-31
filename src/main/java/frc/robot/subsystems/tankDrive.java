// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.RelativeEncoder;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotContainer;


public class tankDrive extends SubsystemBase {


  public CANSparkMax m_leftFrontMotor = RobotContainer.leftFrontMotor;
  public CANSparkMax m_leftBackMotor = RobotContainer.leftRearMotor;
  public static CANSparkMax m_rightFrontMotor = RobotContainer.rightFrontMotor;
  public CANSparkMax m_rightBackMotor = RobotContainer.rightRearMotor;
  
  
  MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftBackMotor, m_leftFrontMotor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightBackMotor, m_rightFrontMotor);
  
  
  //m_Drive = new DifferentialDrive(leftMotors, rightMotors);
  public boolean centerPassed = false;
  private double starterSpeed = 0.035;
  private double degree = 4;
  private double prevVal;
  private boolean addSpeed = false;
  private double speedLimit = 0.15;
  public static RelativeEncoder frEncoder = RobotContainer.rightFrontMotor.getEncoder();
  public static double autoChargeInches; //Community 54", ramp 14", cStation top 76"
  private double inPerEncoder = 2.289; // 19 inches per 8.3 encoder value, one wheel rotation
  public static double distance = 0;
  private boolean logic2 = false;
  public static double additionalSpeed = 0.0002;
  private boolean logic3 = false;
  

  /** Creates a new ExampleSubsystem. */
  public tankDrive() {
    m_rightBackMotor.setInverted(true);
    m_rightFrontMotor.setInverted(true);
    leftMotors.setInverted(false);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    SmartDashboard.putNumber("B- Speed", starterSpeed);
    //SmartDashboard.putBoolean("B- Adding Speed", addSpeed);
    SmartDashboard.putBoolean("B- Passed Center", centerPassed);
    //m_Drive.tankDrive(leftSpeed, rightSpeed);
    m_rightFrontMotor.set(rightSpeed);
    m_leftFrontMotor.set(leftSpeed);
    m_leftBackMotor.set(leftSpeed);
    m_rightBackMotor.set(rightSpeed);
    //leftMotors.set(leftSpeed);
  }

  public void straight(double speed) {
    // straight(starterSpeed); 
    //   if (starterSpeed < speedLimit) { // if wheels have not moved, increase speed
    //     starterSpeed += 0.0002;
    //     addSpeed = true;
        
    //   } else {
    //     addSpeed = false;
    //   }
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
      //System.out.println("DPAD UP");
  }

  public void balance(double pitch) {
    brake(true);
    if (centerPassed && additionalSpeed > 0.0001) {
      additionalSpeed -= 0.000015;
    } else {
      //additionalSpeed = 0.0002;
    }
    
    //frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // For future note, intially start with high power to overcome
    // static friction, then lower power to move at constant slow velocity

    //Currently, steadily increase speed until it start moves
    
    if(pitch > degree) {
      straight(starterSpeed); 
      if (starterSpeed < speedLimit) { // if wheels have not moved, increase speed
        starterSpeed += additionalSpeed;
        addSpeed = true;
        
      } else {
        addSpeed = false;
      }
      centerPassed = false;
      
      SmartDashboard.putBoolean("B- Moving!", true);
    } else if (pitch < -degree) {
      straight(-starterSpeed);
      if (starterSpeed < speedLimit) { // if wheels have not moved, increase speed
        starterSpeed += additionalSpeed;
        addSpeed = true;
      } else {
        addSpeed = false;
      }
      SmartDashboard.putBoolean("B- Moving!", true);
      centerPassed = false;
    } else {
      starterSpeed = 0.035;
      brake(true);
      stop();
      SmartDashboard.putBoolean("B- Moving!", false);
      addSpeed = false;
      centerPassed = true;
    }
  }

  public void autoChargeStation() {
    //SmartDashboard.putData(leftMotors);
        // if (!logic2) {
        //    //in inches
        // } else {
        //   if (!logic3) {
        //       //autoChargeInches = 90;
        //   } else {
        //      //to stop distance tracking
        //   }
          
        // }

        distance = frEncoder.getPosition() * inPerEncoder;
        
        if (Math.abs(distance) < 175 && !logic2) {
          if (Math.abs(distance) < 90) {
            straight(-0.5);
          } else {
            straight(-0.2);
          }
          //System.out.println("Running");
            
      
        } else {
          logic2 = true;
          if (!logic3 && Math.abs(distance) > autoChargeInches) {
            straight(0.5);

            System.out.println(distance);
          } else {
            logic3 = true;
            balance(Robot.pitch);
            System.out.println("balancing!");
          }
          
            
            //System.out.println("Balance");
        }
        
  }
}