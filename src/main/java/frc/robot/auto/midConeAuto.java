package frc.robot.auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem.*;
import frc.robot.subsystems.*;
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
        Hand.hold = true;
        Arm.getInstance().cone().andThen(Arm.getInstance().dunk()).andThen(Arm.getInstance().rest()).andThen(Arm.getInstance().charge);
        Hand.hold = false;
        finished = true;

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
