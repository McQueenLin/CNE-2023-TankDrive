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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotContainer;


public class tankDrive extends SubsystemBase {


  private CANSparkMax m_leftFrontMotor = new CANSparkMax(Constants.tankDriveConstants.leftFrontDeviceID, MotorType.kBrushless);
  private CANSparkMax m_leftBackMotor = new CANSparkMax(Constants.tankDriveConstants.leftBackDeviceID, MotorType.kBrushless);
  private CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.tankDriveConstants.rightFrontDeviceID, MotorType.kBrushless);
  private CANSparkMax m_rightBackMotor = new CANSparkMax(Constants.tankDriveConstants.rightBackDeviceID, MotorType.kBrushless);

  MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftBackMotor, m_leftFrontMotor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightBackMotor, m_rightFrontMotor);
  
  
  DifferentialDrive m_Drive = new DifferentialDrive(leftMotors, rightMotors);
  
  

  /** Creates a new ExampleSubsystem. */
  public tankDrive() {
    rightMotors.setInverted(true);
    leftMotors.setInverted(false);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    m_Drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void straight(double speed) {
    m_Drive.tankDrive(speed, speed);

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
      m_Drive.tankDrive(-speed, speed);
    } else { //right
      m_Drive.tankDrive(speed, -speed);
    }
  }

  public void stop() {
    m_Drive.tankDrive(0, 0);
    System.out.println("DPAD UP");
  }
}