package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Solenoid;
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
     * the axle will be up. 
     * 
     * there will be a double acting pneumatic cylinder
     * that will be used to push the axle to a position
     * where it can pull balls in
     * 
     * there will be one motor to spin the axle.  the
     * motor will be controlled with a spark controller.
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
    private Spark motor;
    private Solenoid solenoid;

    public Intake(){

        // initialize the motor
		motor = new Spark(RobotMap.Intake_SparkMotor_Id);
        motor.enableDeadbandElimination(true);

        // initialize the solenoid
        solenoid = new Solenoid(RobotMap.Pneumatic_Module_ID, RobotMap.Intake_Pneumatic_Forward_Solenoid_ID);
    }

    public void lower(){
        solenoid.set(true);
        intakePosition = IntakePositions.DOWN;
    }

    public void raise(){
        solenoid.set(false);
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
        motorState = MotorStates.STOPPED;
    }

    public boolean spinning(){
        return (motorState == MotorStates.RUNNING);
    }

    public boolean notSpinning(){
        return (motorState == MotorStates.STOPPED);
    }
}