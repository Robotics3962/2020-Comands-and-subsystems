package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Lift extends SubsystemBase {
    /**
     * Users Guide
     * 
     * To start the ball lift, call start().  The speed
     * the motors move is defined in robotmap
     * 
     * To stop the ball lif call stop()
     */

    /**
     * used to keep track of current state
     */
    private enum MotorStates {STOPPED, RUNNING};

    /**
     * declare variables tracking current state of intake
     * and initialize them
     */
    private double speed = RobotMap.Lift_MotorSpeed;
    private MotorStates motorState = MotorStates.STOPPED;

    /**
     * these are the controllers for the motors and the
     * pneumatics
     */
    private Spark motor;

    public Lift(){

        // initialize the motor
		motor = new Spark(RobotMap.Lift_SparkMotor_ID);
        motor.enableDeadbandElimination(true);
    }

    public void start(){
        motor.set(speed);
        motorState = MotorStates.RUNNING;
    }

    public void stop(){
        motor.set(0);
        motorState = MotorStates.STOPPED;
    }

    public boolean isSpinning(){
        return (motorState == MotorStates.RUNNING);
    }

    public boolean isNNotSpinning(){
        return (motorState == MotorStates.STOPPED);
    }

    @Override
    public void periodic(){
        
    }
}