package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Spinner extends Subsystem {
    /**
     * Users Guide
     * 
     * To start the ball lift, call start().  The speed
     * the motors move is defined in robotmap
     * 
     * To stop the ball lif call stop()
     * 
     * To see if the motor is spinning call isSpinning()
     * 
     * To read the color under the color sensor call
     * readColor().  This will return a constant indicating
     * one of the colors that is predefined or other if
     * the color sensor is on a transition between pie slices
     */

    /**
     * used to keep track of if spinner is deployed
     */
    private enum SolenoidStates { NOT_EXTENDED, EXTENDED};

     /**
     * used to keep track of current state
     */
    private enum MotorStates {STOPPED, RUNNING};

    /**
     * this describes which color the sensor is over
     */
    public enum SpinnerColors {OTHER, RED, GREEN, BLUE, YELLOW};

    /**
     * declare variables tracking current state of intake
     * and initialize them
     */
    private double speed = RobotMap.Spinner_MotorSpeed;
    private MotorStates motorState = MotorStates.STOPPED;
    private SolenoidStates solenoidState = SolenoidStates.NOT_EXTENDED;
    /**
     * these are the controllers for the motors 
     */
    private Spark motor;
    private Solenoid solenoid;

    public Spinner(){

        // initialize the motor
		motor = new Spark(RobotMap.Spinner_SparkMotor_ID);
        motor.enableDeadbandElimination(true);

        // initialize the solenoid
        solenoid = new Solenoid(RobotMap.Pneumatic_Module_ID, RobotMap.Spinner_Pneumatic_Forward_Solenoid_ID);
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

    public SpinnerColors readColor(){
        // need to implement this
        return SpinnerColors.RED;
    }

    public void extend(){
        solenoid.set(true);
        solenoidState = SolenoidStates.EXTENDED;
    }

    public boolean isExtended(){
        return (solenoidState == SolenoidStates.EXTENDED);
    }

    @Override
    protected void initDefaultCommand() {
    }
}
