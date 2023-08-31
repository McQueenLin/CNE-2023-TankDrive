// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.tankDrive;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.tankDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.tankDrive;
import frc.robot.auto.Charge;

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
  
  
  public static int first = 0;
  public static int OpenCounter=0;
  public static boolean detect = false;
  public static boolean activated = false;
  public XboxController operator = RobotContainer.operatorController;

  public CANSparkMax HandMotor = RobotContainer.handMotor;
  Thread m_visionThread;

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Command Charge = new Charge();

  SequentialCommandGroup midConeAuto;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    robotContainer.navX.resetGyro();
    m_visionThread =
    new Thread(
      () -> {
        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(320, 240);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);

        // Mats are very memory expensive. Lets reuse this Mat.
        Mat mat = new Mat();

        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
          // Tell the CvSink to grab a frame from the camera and put it
          // in the source mat.  If there is an error notify the output.
          if (cvSink.grabFrame(mat) == 0) {
            // Send the output the error.
            outputStream.notifyError(cvSink.getError());
            // skip the rest of the current iteration
            continue;
          }
          // Put a rectangle on the image
          Imgproc.rectangle(
              mat, new Point(250, 200), new Point(350, 300), new Scalar(255, 255, 255), 5);
          // Give the output stream a new image to display
          outputStream.putFrame(mat);
        }
      });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    
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
    Hand.getInstance().hold = true;
    // midConeAuto = Arm.getInstance().cube().andThen(new WaitCommand(1)).andThen(Arm.getInstance().dunk()).andThen(
     //  new WaitCommand(1)).andThen(Arm.getInstance().undunk()).andThen(new WaitCommand(2)).andThen
     //  (Arm.getInstance().rest()).andThen(Hand.getInstance().setFalse().andThen(Charge.repeatedly()));
        
    //midConeAuto.schedule();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
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
    SmartDashboard.putBoolean("Left Bumper", operator.getLeftBumper());
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
