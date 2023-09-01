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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import edu.wpi.first.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;

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
  JoystickButton operatorBButton = new JoystickButton(operatorController, Button.kB.value);
  JoystickButton operatorXButton = new JoystickButton(operatorController, Button.kX.value);
  POVButton operatorPOV = new POVButton(operatorController, -1);
  // POVButton driverDpadUp = new POVButton(m_driverController, 0);
  // POVButton driverDpadRight = new POVButton(m_driverController, 90);
  // POVButton driverDpadDown = new POVButton(m_driverController, 180);
  // POVButton driverDpadLeft = new POVButton(m_driverController, 270);
 
  public tankDrive tank_Drive = new tankDrive();

  //public Hand hand = new Hand();
  // public RelativeEncoder frEncoder = tank_Drive.m_rightFrontMotor.getEncoder();

  public teleopDrive tDrive = new teleopDrive(tank_Drive, driverController);

  Arm arm = Arm.getInstance();
  Hand hand = Hand.getInstance();

  Command Charge = new Charge();
  public static final DigitalInput PhotoSwitch = new DigitalInput(0);
  //public RepeatCommand charge = Charge.repeatedly();

  public SequentialCommandGroup midConeAuto = hand.setSensorOn().andThen(new ParallelCommandGroup(arm.cone(), hand.autoClose())).andThen(new WaitCommand(1)).andThen(new ParallelCommandGroup(arm.dunk(),hand.autoClose()))
  .andThen(new WaitCommand(1)).andThen(hand.Opening()).andThen(new WaitCommand(0.5)).andThen(arm.undunk()).andThen(new WaitCommand(1))
  .andThen(arm.rest()).andThen(hand.setFalse()).andThen(hand.setSensorOff());

  public SequentialCommandGroup highCubeAuto = hand.setSensorOn().andThen(new ParallelCommandGroup(arm.cube(), hand.autoClose())).andThen(new WaitCommand(1)).andThen(new ParallelCommandGroup(arm.dunk(),hand.autoClose()))
  .andThen(new WaitCommand(1)).andThen(hand.Opening()).andThen(new WaitCommand(0.5)).andThen(arm.undunk()).andThen(new WaitCommand(1))
  .andThen(arm.rest()).andThen(hand.setFalse()).andThen(hand.setSensorOff());
  

  
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
    
      operatorBButton.whileTrue(Arm.getInstance().cone().andThen(new WaitCommand(0.5)).andThen(Arm.getInstance().coneDunk()));
      operatorYButton.whileTrue(Arm.getInstance().cube().andThen(new WaitCommand(1)).andThen(Arm.getInstance().cubeDunk()));
      operatorPOV.whileTrue(Arm.getInstance().rest());
      operatorAButton.whileTrue(Arm.getInstance().floor());
      operatorXButton.whileTrue(Arm.getInstance().chute());

      operatorLeftBumper.whileTrue(Hand.getInstance().Opening().repeatedly());
      
    
      //operatorRightBumper.whileTrue(Hand.getInstance().autoClose().repeatedly());
      operatorRightBumper.whileTrue(Hand.getInstance().Closing().repeatedly());
      operatorRightBumper.whileFalse(Hand.getInstance().Holding().repeatedly());
  
  }
}