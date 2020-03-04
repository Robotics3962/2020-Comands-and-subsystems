package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;

public class Lift extends SubsystemBase {

    /**
     * Users Guide
     * 
     * To start the ball lift, call start().  The speed
     * the motors move is defined in robotmap as Lift_MotorSpeed
     * 
     * To stop the ball lift call stop()
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
     * these are the controllers for the motors 
     */
    private Spark motor;

    public Lift(){
        if (RobotMap.Lift_Motor1_Invert){
            speed = speed * -1;
        }

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

    public void forward(){
        start();
    }

    public void reverse(){
        motor.set(-speed);
        motorState = MotorStates.RUNNING;
    }

    public boolean isSpinning(){
        return (motorState == MotorStates.RUNNING);
    }

    public boolean isNotSpinning(){
        return (motorState == MotorStates.STOPPED);
    }

    @Override
    public void periodic(){
    }
}