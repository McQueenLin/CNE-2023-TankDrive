package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.tankDrive;


public class Charge extends CommandBase {

    tankDrive tankDrive = new tankDrive();

    public Charge() {
        addRequirements(tankDrive);
    }

    @Override
    public void execute() {
        tankDrive.autoChargeStation(); 
        
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
