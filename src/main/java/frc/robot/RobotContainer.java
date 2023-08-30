// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.Charge;
import frc.robot.auto.midConeAuto;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.tankDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.tankDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Hand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  public static final XboxController operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);
  
  public static final CANSparkMax armMotor = new CANSparkMax(24,CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax elbowMotor = new CANSparkMax(41,CANSparkMaxLowLevel.MotorType.kBrushless);
  //public static final DigitalInput input = new DigitalInput(0);

  public static final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.tankDriveConstants.leftFrontDeviceID, MotorType.kBrushless);
  public static final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.tankDriveConstants.rightFrontDeviceID, MotorType.kBrushless);
  public static final CANSparkMax leftRearMotor = new CANSparkMax(Constants.tankDriveConstants.leftBackDeviceID, MotorType.kBrushless);
  public static final CANSparkMax rightRearMotor = new CANSparkMax(Constants.tankDriveConstants.rightBackDeviceID, MotorType.kBrushless);
  
  public static final CANSparkMax handMotor = new CANSparkMax(23, CANSparkMaxLowLevel.MotorType.kBrushless);
  

  public static final NavXGyro navX = NavXGyro.getInstance();
 
  
  JoystickButton operatorLeftBumper = new JoystickButton(operatorController, Button.kLeftBumper.value);
  JoystickButton operatorRightModifier = new JoystickButton(operatorController, Button.kStart.value);
  JoystickButton operatorLeftModifier = new JoystickButton(operatorController, Button.kBack.value);
  JoystickButton operatorRightBumper = new JoystickButton(operatorController, Button.kRightBumper.value);
  JoystickButton operatorYButton = new JoystickButton(operatorController, Button.kY.value);
   JoystickButton operatorAButton = new JoystickButton(operatorController, Button.kA.value);
//   JoystickButton operatorBButton = new JoystickButton(operatorController, Button.kB.value);
  JoystickButton operatorXButton = new JoystickButton(operatorController, Button.kX.value);
  // POVButton driverDpadUp = new POVButton(m_driverController, 0);
  // POVButton driverDpadRight = new POVButton(m_driverController, 90);
  // POVButton driverDpadDown = new POVButton(m_driverController, 180);
  // POVButton driverDpadLeft = new POVButton(m_driverController, 270);
 
  public tankDrive tank_Drive = new tankDrive();
  //public Hand hand = new Hand();
  // public RelativeEncoder frEncoder = tank_Drive.m_rightFrontMotor.getEncoder();

  public teleopDrive tDrive = new teleopDrive(tank_Drive, driverController);

  public Arm armSubsystem = new Arm();
  public midConeAuto midConeAuto = new midConeAuto(armSubsystem);
  
  public RobotContainer(){
        // Configure the trigger bindings
        configureBindings();
    }
 
  private void configureBindings()
  {

      // System.out.println("Bind");


      Arm.getInstance().setDefaultCommand(Arm.getInstance().changePos());
      Hand.getInstance().setDefaultCommand(Hand.getInstance().autoClose().repeatedly().unless(() -> operatorController.getLeftBumper()));

      
      // Hand.getInstance().setDefaultCommand(Hand.getInstance().Opening());
    
      // operatorYButton.whileTrue(Arm.getInstance().rest());
      // operatorAButton.whileTrue(Arm.getInstance().cone());
      // // operatorBButton.whileTrue(Arm.getInstance().cube());
      // operatorXButton.whileTrue(Arm.getInstance().floor());

      operatorLeftBumper.whileTrue(Hand.getInstance().Opening().repeatedly());
      
    
      //operatorRightBumper.whileTrue(Hand.getInstance().autoClose().repeatedly());
      operatorRightBumper.whileTrue(Hand.getInstance().Closing().repeatedly());
      operatorRightBumper.whileFalse(Hand.getInstance().Holding().repeatedly());
  
  }
}