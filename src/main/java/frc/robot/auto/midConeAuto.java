package frc.robot.auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Subsystem.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
public class midConeAuto extends CommandBase{
    boolean finished;
    private final Arm arm;

    public midConeAuto(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        finished = false;
    }
    @Override
    public void execute() {
        
        finished = true;

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
