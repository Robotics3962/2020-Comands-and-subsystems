package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
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
     * 
     * elevate(): 
     *      this function is called to start elevating the climber
     * 
     * isElevated():
     *      this function is called to see if the climber is fully elevated. It returns
     *      true if the climber is completely elevated and false otherwise.
     * 
     * retract():
     *      this function is called to start retracting the climber. 
     * 
     * isRetracted():
     *      this funtion is called to see if the climber is fully retracted.  It returns
     *      true if the climber is completely retracted and false otherwise.
     * 
     * isMoving():
     *      this function is called to see if the climber is moving up or down.  It returns
     *      true if it is not elevated, and it is not retracted, and the motor state
     *      is moving.
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
    private SpeedControllerGroup motors;
    
    public Climber(){

        // initialize the motor
		motor1 = new WPI_TalonSRX (RobotMap.Climber_TalonMotor1_ID);
        motor2 = new WPI_TalonSRX (RobotMap.Climber_TalonMotor2_ID);
        motors = new SpeedControllerGroup(motor1, motor2);

        /**
         * add motor initialization code here
         */

        motor1.setNeutralMode(NeutralMode.Coast);//brake
        motor2.setNeutralMode(NeutralMode.Coast);//brake
        motor1.setInverted(RobotMap.Climber_TalonMotor1_Invert); 
        motor2.setInverted(RobotMap.Climber_TalonMotor2_Invert); 
        Util.configTalon(motor1);
        Util.configTalon(motor2);

        /**
         * configure limit switches
         */
        /* Configured forward and reverse limit switch of Talon to be from a feedback connector and be normally open */
        motor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        motor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    public void elevate(){
        currCommand = Commands.ELEVATE;
        motors.set(RobotMap.Climber_MotorSpeedUp);
        motorState = MotorStates.RUNNING;
    }

    public void retract(){
        currCommand = Commands.RETRACT;
        motors.set(-RobotMap.Climber_MotorSpeedDown);
        motorState = MotorStates.RUNNING;
    }

    public void stop(){
        currCommand = Commands.NONE;
        motors.set(0);
        motors.stopMotor();
        motorState = MotorStates.STOPPED;
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
        upperLimitSwitchState =  motor1.getSensorCollection().isFwdLimitSwitchClosed();
        
        if(upperLimitSwitchState == true) {
            motor1.stopMotor();
            motor2.stopMotor();
        } else {
            //DO NOTHING
        }

        return upperLimitSwitchState;
    }

    public boolean isRetracted(){
        boolean lowerLimitSwitchState = false;

        // read state of limit switch
        // if elevated or retracted, stop the motor
        lowerLimitSwitchState =  motor1.getSensorCollection().isRevLimitSwitchClosed();

        if(lowerLimitSwitchState == true) {
            motor1.stopMotor();
            motor2.stopMotor();
        } else {
            // DO NOTHING
        }

        return lowerLimitSwitchState;
    }
    
    public void up(){
        motors.set(RobotMap.Climber_MotorSpeedUp);
        motorState = MotorStates.RUNNING;
    }

    public void down() {
        motors.set(RobotMap.Climber_MotorSpeedDown);
        motorState = MotorStates.RUNNING;
    }

    @Override
    public void periodic(){
        updateDashboard();

        if (! isMoving()){
            motors.stopMotor();
        }
    }

    public void dumpState(){
        System.out.println("Moving:" + (motorState == MotorStates.RUNNING) + " upperLimitSwitch:" + isElevated() + " lowerLimitSwitch:" + isRetracted());
    }

    public void updateDashboard(){
        SmartDashboard.putBoolean("Climber-moving", (motorState == MotorStates.RUNNING));
        SmartDashboard.putBoolean("Climber-topLimitSw", isElevated());
        SmartDashboard.putBoolean("Climber-bottomLimitSw", isRetracted());

    }
}