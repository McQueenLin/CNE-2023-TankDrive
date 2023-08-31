package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;
import static frc.robot.RobotContainer.*;
import frc.robot.auto.Charge;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Arm extends SubsystemBase {

    public static final CANSparkMax armMotor = RobotContainer.armMotor;
    public static final CANSparkMax elbowMotor = RobotContainer.elbowMotor;
    //public static final DigitalInput input = RobotContainer.input;
    private final SparkMaxPIDController armPID;
    private final SparkMaxPIDController elbowPID;
    private final SparkMaxPIDController armPIDHold;
    private final SparkMaxPIDController ElbowPIDHold;


    private RelativeEncoder elbowEncoder;
    private RelativeEncoder armEncoder;

    Position currentPosition = Position.REST;

    boolean PID = true;

    double currentPositionHoldArm;
    double currentPositionHoldElbow;

    static double armChange;
    static double elbowChange;
    
    double target;

    enum Position {
        FLOOR(0, -90),
        CUBE(-75, 54),
        CONE(-33, -18),
        //DUNK(-90, 70),
        REST(0, 0),
        UNDUNK(armChange, elbowChange);
        public double arm;
        public double elbow;
        Position(double arm, double elbow) {
            this.arm = arm;
            this.elbow = elbow;
        }
    }



    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this Hand. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static Arm INSTANCE = new Arm();

    /**
     * Returns the Singleton instance of this Hand. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code Hand.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Arm getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Hand. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public Arm() {
        this.elbowEncoder = elbowMotor.getEncoder();
//        this.elbowEncoder.setPosition(0);
        this.armEncoder = armMotor.getEncoder();
//        this.armEncoder.setPosition(0);

        this.armPID = armMotor.getPIDController();
        this.elbowPID = elbowMotor.getPIDController();
        this.armPIDHold = armMotor.getPIDController();
        this.ElbowPIDHold = elbowMotor.getPIDController();

        elbowPID.setOutputRange(-1,1);
        armPID.setOutputRange(-1,1);

        elbowPID.setP(1);
        elbowPID.setI(0);
        elbowPID.setD(0);
        elbowPID.setFF(0.1);

        armPID.setP(1);
        armPID.setI(0);
        armPID.setD(0);
        armPID.setFF(0.1);

        elbowPID.setOutputRange(-0.5,0.25);
        armPID.setOutputRange(-1,1);

        armEncoder.setPosition(0);
        elbowEncoder.setPosition(0);

        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    }


    public Command cube() {
        return runOnce( () -> {
            setPosition(Position.CUBE);
        }).andThen(moveArm());
    }

    public Command cone() {
        return runOnce( () -> {
            setPosition(Position.CONE);
        }).andThen(moveArm());
    }

    public Command floor(){
        return runOnce( () -> {
            setPosition(Position.FLOOR);
        });
    }

    public Command rest() {
        return runOnce( () -> {
            setPosition(Position.REST);
        }).andThen(moveArm());
    }

    boolean idleMode = false;

    public void setPosition(Position position){

        currentPosition = position;

    }

    public Command changePos(){
        //currentPositionHoldArm = currentPosition.arm;
        //currentPositionHoldElbow = currentPosition.elbow;

        return runOnce(() ->{
            // armMotor.set(operatorController.getRightY()*0.3);
            // elbowMotor.set(operatorController.getLeftY()*0.3);
            double elbow = operatorController.getRightY();
            double arm = operatorController.getLeftY();

            //System.out.println(Math.abs(arm))}

            if(Math.abs(arm) > 0.05){
                SmartDashboard.putNumber("arm", arm);
                //currentPositionHoldArm = currentPositionHoldArm + arm;
                currentPosition.arm = currentPosition.arm + arm;
            }

            if(Math.abs(elbow) > 0.05){
                SmartDashboard.putNumber("elbow", elbow);
                //currentPositionHoldElbow =  currentPositionHoldElbow - elbow;
                currentPosition.elbow = currentPosition.elbow - elbow;
            }
        }).andThen(moveArm()); //}).andThen(moveArm())


    }



    public Command moveArm() { //Auto positioning
        return runOnce(() -> {
                armPID.setReference(currentPosition.arm, CANSparkMax.ControlType.kPosition);
                elbowPID.setReference(currentPosition.elbow, CANSparkMax.ControlType.kPosition);
        });
    }

    /*
    public Command holdArm() { //Auto positioning
        return runOnce(() -> {
            if(PID == true){
                elbowPID.setP(1);
                elbowPID.setI(0);
                elbowPID.setD(0);
                elbowPID.setFF(0.1);

                armPID.setP(1);
                armPID.setI(0);
                armPID.setD(0);
                armPID.setFF(0.1);
                armPID.setReference(currentPositionHoldArm, CANSparkMax.ControlType.kPosition);
                elbowPID.setReference(currentPositionHoldElbow, CANSparkMax.ControlType.kPosition);
            }else if (PID == false){
                elbowPID.setP(0);
                elbowPID.setFF(0);

                armPID.setP(0);
                armPID.setFF(0);
                armPID.setReference(currentPositionHoldArm, CANSparkMax.ControlType.kPosition);
                elbowPID.setReference(currentPositionHoldElbow, CANSparkMax.ControlType.kPosition);
            }
        });
    }
*/

    public Command dunk(){
        target = currentPosition.elbow - 50;
        this.elbowChange = currentPosition.elbow;
        double elbow = 0.1;
        SmartDashboard.putNumber("Elbow change", elbowChange);
        return runOnce(() -> {
            while(true){
                SmartDashboard.putBoolean("dunk", Math.abs(elbow) > 0.05 && currentPosition.elbow < target);
                if(currentPosition.elbow > target){
                    //SmartDashboard.putNumber("elbow", elbow);
                    //currentPositionHoldElbow =  currentPositionHoldElbow - elbow;
                    currentPosition.elbow = currentPosition.elbow - elbow;
                    elbowPID.setReference(currentPosition.elbow, CANSparkMax.ControlType.kPosition);             
                }
                else if (currentPosition.elbow < target){
                    // currentPosition.elbow = currentPosition.elbow;
                    break;
                }
            }
        });
    }

    public Command undunk(){
        return runOnce(() -> {
            elbowPID.setReference(elbowChange, CANSparkMax.ControlType.kPosition);
        });
    }


    public Charge charge = new Charge();


    @Override
    public void periodic() {
       
        SmartDashboard.putNumber("target", currentPositionHoldArm);
        SmartDashboard.putNumber("Elbow Temperature", elbowMotor.getMotorTemperature());
        SmartDashboard.putNumber("arm", armEncoder.getPosition());
        SmartDashboard.putNumber("P integral", elbowPID.getP());
        SmartDashboard.putNumber("Elbow", elbowEncoder.getPosition());



    }
}