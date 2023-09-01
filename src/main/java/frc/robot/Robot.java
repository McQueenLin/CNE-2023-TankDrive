// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Charge;
import frc.robot.auto.midConeAuto;

import java.nio.file.attribute.AclFileAttributeView;

import com.ctre.phoenixpro.Timestamp;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  // public static final CANSparkMax armMotor = RobotContainer.armMotor;
  // public static final CANSparkMax elbowMotor = RobotContainer.elbowMotor;
  // public XboxController controller = new XboxController(0);
  
  private Command m_autonomousCommand;

  public RobotContainer robotContainer;
  public static double pitch;
  public RelativeEncoder frEncoder = RobotContainer.rightFrontMotor.getEncoder();

  public final String midConeAuto = "midConeAuto";
  public final String midCubeAuto = "midCubeAuto";
  public final String highCubeAuto = "highCubeAuto";
  String autoSelected;
  SendableChooser<String> auto = new SendableChooser<>();
  
  
  public static int first = 0;
  public static int OpenCounter=0;
  public static boolean detect = false;
  public static boolean activated = false;
  public XboxController operator = RobotContainer.operatorController;

  public CANSparkMax HandMotor = RobotContainer.handMotor;
  Thread m_visionThread;

  // Arm arm = new Arm();
  //Command midConeAuo = new midConeAuto(arm);
  Command Charge = new Charge();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    auto.setDefaultOption("High Cube Auto", highCubeAuto);
    auto.addOption("Mid Cube Auto", midCubeAuto);
    auto.addOption("Mid Cone Auto", midConeAuto);
    SmartDashboard.putData("Auto choices", auto);
    robotContainer = new RobotContainer();
    robotContainer.navX.resetGyro();
    m_visionThread =
    new Thread(
      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(160, 240);
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Rectangle", 160, 240);
        
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }
          Imgproc.rectangle(mat, new Point(75, 115), new Point(85, 125), new Scalar(255, 255, 255), 2);
          outputStream.putFrame(mat);
        } 
      });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

   /*
    m_visionThread =
    new Thread(
      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture(1);
        camera.setResolution(160, 240);
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Rectangle", 160, 240);
        /*
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }
          Imgproc.rectangle(mat, new Point(75, 115), new Point(85, 125), new Scalar(255, 255, 255), 2);
          outputStream.putFrame(mat);
        } 
      });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    */

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    pitch = robotContainer.navX.getNavPitch();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Elbow Temperature", Arm.elbowMotor.getMotorTemperature());
    SmartDashboard.putNumber("Arm Position", Arm.armEncoder.getPosition());
    SmartDashboard.putNumber("Elbow Position", Arm.elbowEncoder.getPosition());

    Hand.motorSpeed = SmartDashboard.getNumber("Motor speed", 0.5);
    SmartDashboard.putNumber("Hand Temperature", HandMotor.getMotorTemperature());
    SmartDashboard.putBoolean("Sensor", !robotContainer.PhotoSwitch.get());
    SmartDashboard.putNumber("Hand Position", HandMotor.getEncoder().getPosition()); 

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // public Command getAutonomousCommand() {
  //   return m_chooser.getSelected();
  // }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoSelected = auto.getSelected();
    tankDrive.frEncoder.setPosition(0);    
    
  switch(autoSelected){
    case midConeAuto:
      CommandScheduler.getInstance().schedule(robotContainer.midConeAuto.andThen(robotContainer.Charge.repeatedly()));
      break;
    case midCubeAuto:
      CommandScheduler.getInstance().schedule(robotContainer.midCubeAuto.andThen(robotContainer.Charge.repeatedly()));
      break;
    default:
      CommandScheduler.getInstance().schedule(robotContainer.highCubeAuto.andThen(robotContainer.Charge.repeatedly()));
  }

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    //CommandScheduler.getInstance().schedule(robotContainer.highCubeAuto.andThen(robotContainer.Charge.repeatedly()));
    //CommandScheduler.getInstance().schedule(robotContainer.Charge.repeatedly());
    //CommandScheduler.getInstance().schedule(Charge.repeatedly());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //Hand.getInstance().Holding();
    
    //SendableChooser
    
  }

  @Override
  public void teleopInit() {
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putBoolean("Left Bumper", operator.getLeftBumper());
    // if(!activated)  detect = !robotContainer.photoSwitch.get();
    // SmartDashboard.putBoolean("Detect", detect);
    // SmartDashboard.putNumber("First", first);
    // SmartDashboard.putBoolean("Close", activated);
    CommandScheduler.getInstance().schedule(robotContainer.tDrive);
     // SmartDashboard.putBoolean("Robot.Detect", Robot.detect);
    // if (detect && !activated)// && !driverController.getLeftBumper()) 
    // {
    //   first ++;
      
    //   if (first < 5) 
    //   {
        
    //     HandMotor.set(0.3);
    //   } 
    //   else 
    //   {
    //     HandMotor.set(0.1);
    //     if(first > 200)
    //     {
    //       OpenCounter=0;
    //       first = 0;
    //       detect = false;
    //       activated = true;
    //     }
    //   }
      
    // }
    // else
    // {
    //   OpenCounter ++;
    //   HandMotor.set(-0.05);
    //   if(OpenCounter > 500) 
    //   {
    //     activated = false;
    //     OpenCounter = 0;
    //   }
    // }
    
    //handMotor.set(controller.getLeftY());
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
