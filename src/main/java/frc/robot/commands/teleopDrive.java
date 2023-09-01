// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.tankDrive;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
//import frc.robot.NavxGyro;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;




/** An example command that uses an example subsystem. */
public class teleopDrive extends CommandBase {
  private final tankDrive m_Drive;
  public XboxController m_Controller;
  
  private double speedReductionConstant = 0.5;
  private double timer = 0;
  private boolean backToNormal = false;
  
  private double adjSpeed = 0.065;
  
  

  //public RobotContainer robotContainer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public teleopDrive(tankDrive subsystem, XboxController controller) {
    this.m_Drive = subsystem;
    this.m_Controller = controller;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //robotContainer = new RobotContainer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RelativeEncoder frEncoder = m_Drive.m_rightFrontMotor.getEncoder();
    SmartDashboard.putNumber("Robot Pitch", Robot.pitch);
    //SmartDashboard.putNumber("Distance Encoder", frEncoder.getPosition());
    SmartDashboard.putNumber("Drive Speed %", speedReductionConstant*100);
    
    //System.out.println(m_Controller.getPOV());
    // tank_Drive.setLeftMotors(m_leftSpeed);
    // tank_Drive.setRightMotors(m_rightSpeed);
    if (timer <= 10) {
      timer++;
    }
     
    if(Arm.resting) {
      if (backToNormal) {
        speedReductionConstant = 0.5;
        backToNormal = false;
      }
      if (m_Controller.getYButton() && timer >= 10) {
        speedReductionConstant = (speedReductionConstant == 0.7 ? 0.5:0.9);
        timer = 0;
        //this.frEncoder.setPosition(0);        
      }
    } else {
      speedReductionConstant = 0.2;
      backToNormal = true;
    }

    if (m_Controller.getPOV() == 0) {
      m_Drive.straight(adjSpeed);
    } else if (m_Controller.getPOV() == 180) {
      m_Drive.straight(-adjSpeed);
    } else if (m_Controller.getPOV() == 90) {
      m_Drive.pivot(adjSpeed, false);
    } else if (m_Controller.getPOV() == 270) {
      m_Drive.pivot(adjSpeed, true);
    } else if (m_Controller.getBButton()) {
      //m_Drive.test();
    } else if (m_Controller.getAButton()) {
      //starterSpeed = 0.01;
      //m_Drive.balance(Robot.pitch);
    } else if (m_Controller.getXButton()) {
      m_Drive.brake(true);
    }else if(Math.abs(m_Controller.getLeftY()) > 0.1 || Math.abs(m_Controller.getRightY()) > 0.1) {
      
      m_Drive.tankDrive(-m_Controller.getLeftY() * speedReductionConstant, -m_Controller.getRightY() * speedReductionConstant);
    } else {
      m_Drive.brake(false);
      m_Drive.centerPassed = false;
      m_Drive.straight(0);
    }
    
    
    //SmartDashboard.putNumber("leftSpeed", m_Controller.getLeftY());
    //SmartDashboard.putNumber("rightSpeed", m_Controller.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

 
}
