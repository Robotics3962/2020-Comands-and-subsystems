package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    /**
     * Users Guide
     * 
     * The intake consists of a arm that holds
     * a spinning axle with wheels.  When the
     * Robot is in its pre-competition position
     * the arm will be up. 
     * 
     * there will be a double acting pneumatic cylinder
     * that will be used to push the axle to a position
     * where it can pull balls in
     * 
     * there will be one motor to spin the axle.  the
     * motor will be controlled with a talonsrx controller.
     * 
     * To move the intake to the down position, call
     * Intake.lower()
     * 
     * To start the intake spinning, call
     * Intake.start()
     * 
     * To stop the intake spinning, call
     * Intake.stop()
     */

    /**
     * used to keep track of current state
     */
    private enum IntakePositions { DOWN, UP };
    private enum MotorStates {STOPPED, RUNNING};

    /**
     * declare variables tracking current state of intake
     * and initialize them
     */
    private double speed = RobotMap.Intake_Motor_Speed;
    private MotorStates motorState = MotorStates.STOPPED;
    private IntakePositions intakePosition = IntakePositions.UP;

    /**
     * these are the controllers for the motors and the
     * pneumatics
     */
    private WPI_TalonSRX motor;
    private DoubleSolenoid solenoid;

    public Intake(){

        // initialize the motor
		motor = new WPI_TalonSRX(RobotMap.Intake_TalonMotor_Id);
        motor.setInverted(RobotMap.Intake_TalonMotor_Invert); 
        Util.configTalon(motor);

        // initialize the solenoid
        solenoid = new DoubleSolenoid(RobotMap.Pneumatic_Module_ID, RobotMap.Intake_Pneumatic_Forward_Solenoid_ID);
    }

    public void lower(){
        solenoid.set(DoubleSolenoid.Value.kForward);
        intakePosition = IntakePositions.DOWN;
    }

    public void raise(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
        intakePosition = IntakePositions.UP;
    }

    public boolean armUp(){
        return (intakePosition == IntakePositions.UP);
    }

    public boolean armDown(){
        return (intakePosition == IntakePositions.DOWN);
    }

    public void start(){
        motor.set(speed);
        motorState = MotorStates.RUNNING;
    }

    public void stop(){
        motor.set(0);
        motor.stopMotor();
        motorState = MotorStates.STOPPED;
    }

    public boolean spinning(){
        return (motorState == MotorStates.RUNNING);
    }

    public boolean notSpinning(){
        return (motorState == MotorStates.STOPPED);
    }
}