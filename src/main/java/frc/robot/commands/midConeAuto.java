package frc.robot.commands;

import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.tankDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
//import frc.robot.NavxGyro;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import frc.robot.subsystems.Arm;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import static frc.robot.RobotContainer.*;
import edu.wpi.first.wpilibj2.command.*;




/** An example command that uses an example subsystem. */
public class midConeAuto extends CommandBase {



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
