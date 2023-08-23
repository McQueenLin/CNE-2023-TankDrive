// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.tankDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


/** An example command that uses an example subsystem. */
public class teleopDrive extends CommandBase {
  private final tankDrive m_Drive;
  public XboxController m_Controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public teleopDrive(tankDrive subsystem, XboxController controller) {
    m_Drive = subsystem;
    this.m_Controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // tank_Drive.setLeftMotors(m_leftSpeed);
    // tank_Drive.setRightMotors(m_rightSpeed);
    m_Drive.tankDrive(m_Controller.getLeftY() * 0.3, m_Controller.getRightY() * 0.3);
    SmartDashboard.putNumber("leftSpeed", m_Controller.getLeftY());
    SmartDashboard.putNumber("rightSpeed", m_Controller.getRightY());
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
