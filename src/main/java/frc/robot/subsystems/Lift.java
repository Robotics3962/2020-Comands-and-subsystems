package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private SpeedControllerGroup motors = null;

    public Lift(){

        // initialize the motor
        motor1 = new WPI_TalonSRX(RobotMap.Lift_TalonMotor1_ID);
        motor2 = new WPI_TalonSRX(RobotMap.Lift_TalonMotor2_ID);
        motors = new SpeedControllerGroup(motor1, motor2);
        motor1.setInverted(RobotMap.Lift_TalonMotor1_Invert); 
        motor2.setInverted(RobotMap.Lift_TalonMotor2_Invert); 

        Util.configTalon(motor1);
        Util.configTalon(motor2);
    }

    public void start(){
        motors.set(speed);
        motorState = MotorStates.RUNNING;
    }

    public void stop(){
        motors.set(0);
        motorState = MotorStates.STOPPED;
    }

    public void forward(){
        start();
    }

    public void reverse(){
        motors.set(-speed);
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