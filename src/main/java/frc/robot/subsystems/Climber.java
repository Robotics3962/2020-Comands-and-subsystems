package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Climber extends Subsystem {
    /**
     * Users Guide
     *
     * The climber has two positions to move to:
     *  elevated: all the way up
     *  retracted: all the way down
     * 
     * we may need manual control if we need finer control
     * of the elevated position.
     * 
     * To move the climber to the elevated position, call elevate().  This will start
     * the motors moving to extend the climber.  It will extend until the upper limit
     * switch is activated.  Because the limit switch is wired to the talon controller
     * it will automatically stop the motor so we don't have to write any code.
     * 
     * To move the climber to the retracted position, call retract(). This will start
     * the motors moving to lower the climber.  It will lower until the lower limit
     * switch is activated.  Because the limit switch is wired to the talon controller
     * it will automaticallystop the motor so we don't have to write any code. 
     * 
     * To check if the climber is fully elevated, call isElevated().  To check if the
     * climber is fully retracted, call isRetracted().
     * 
     * Generally, the flow of calls would be:
     *      call elevate()
     *      continue to call isElevated() until it returns true
     *      move to hook up to giant hanger
     *      call retract()
     *      continue to call isRetracted() until it returns true
     */

    /**
     * This is used to track what the climber is doing
     */
    private enum Commands {NONE, ELEVATE, RETRACT};

    /**
     * used to keep track of current state
     */
    private enum MotorStates {STOPPED, RUNNING};

    /**
     * declare variables tracking current state of intake
     * and initialize them
     */
    private MotorStates motorState = MotorStates.STOPPED;
    private Commands currCommand = Commands.NONE;
    /**
     * this is the controller for the motor
     */
    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;

    public Climber(){

        // initialize the motor
		motor1 = new WPI_TalonSRX (RobotMap.Climber_TalonMotor1_ID);
        motor2 = new WPI_TalonSRX (RobotMap.Climber_TalonMotor2_ID);
        motor2.follow(motor1);

        /**
         * add motor initialization code here
         */
    }

    public void elevate(){
        currCommand = Commands.ELEVATE;
        motor1.set(ControlMode.PercentOutput, RobotMap.Climber_MotorSpeed);
        motorState = MotorStates.RUNNING;
    }

    public void retract(){
        currCommand = Commands.RETRACT;
        motor1.set(ControlMode.PercentOutput, RobotMap.Climber_MotorSpeed);
        motorState = MotorStates.RUNNING;
    }

    public boolean isMoving(){
        boolean elevated = isElevated();
        boolean retracted = isRetracted();
        boolean moving = (motorState == MotorStates.RUNNING);

        // if elevated or retracted, stop the motor

        return (!elevated && !retracted && moving);
    }

    public boolean isElevated(){
        boolean upperLimitSwitchState = false;

        // read state of limit switch
        // if elevated or retracted, stop the motor

        return upperLimitSwitchState;
    }

    public boolean isRetracted(){
        boolean lowerLimitSwitchState = false;

        // read state of limit switch
        // if elevated or retracted, stop the motor

        return lowerLimitSwitchState;
    }

    @Override
    protected void initDefaultCommand() {
    }
}