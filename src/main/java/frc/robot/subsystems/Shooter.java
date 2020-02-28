package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    /**
     * Users guide
     * 
     * To start the shooter wheel call spin(speed) where
     * speed is a value between -1.0 and 1.0. The greater
     * the absolute value of the speed, the faster the wheel
     * will rotate and the further the ball will shoot (maybe)
     * 
     * To stop the shooter wheel call stop().
     * 
     * To change the position of the adjuster call setAdjusterPosition.
     * This will use the pwm controller built into the talons to move
     * the adjuster to a position.  
     */

    /**
     * used to keep track of current state
     *  of wheelMotors
     */
    private enum MotorStates {STOPPED, RUNNING};
    
    /**
     * These are the controllers for the wheel
     * and the adjuster
     */
    private TalonSRX  adjusterMotor = null;
    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private SpeedControllerGroup wheelMotors = null;

    /**
     * These variables hold the state of the
     * components
     */
    private MotorStates wheelState = MotorStates.STOPPED;
    private MotorStates adjusterState = MotorStates.STOPPED;

    private double wheelSpeed = 0;

    /**
     * This variable holds the initial encoder position
     * we use this to normalize the position so we can
     * move to predetermined offsets
     */
    private int initialEncoderPosition = 0;

    public Shooter(){

        //initialize the spark motors driving the
        // flywheel
        motor1 = new WPI_TalonSRX(RobotMap.Shooter_TalonMotor1_ID);
        motor2 = new WPI_TalonSRX(RobotMap.Shooter_TalonMotor2_ID);
        wheelMotors = new SpeedControllerGroup(motor1, motor2);
        motor1.setNeutralMode(NeutralMode.Brake);
        motor2.setNeutralMode(NeutralMode.Brake);
        motor1.setInverted(RobotMap.Shooter_TalonMotor1_Invert); 
        motor2.setInverted(RobotMap.Shooter_TalonMotor2_Invert); 

        Util.configTalon(motor1);
        Util.configTalon(motor2);

        /**
         * set up the talon with the encoder
         */
        adjusterMotor = new TalonSRX(RobotMap.Shooter_TalonAdjusterMotor_ID);

        Util.configTalonSRX(adjusterMotor);
        adjusterMotor.setInverted(RobotMap.Shooter_TalonAdjusterMotor_Invert);

        /**
         * configure limit switches
         */
        /* Configured forward and reverse limit switch of Talon to be from a feedback connector and be normally open */
        motor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        motor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        /**
         * invert the direction if necessary
         * talon.setInverted(RobotMap.Shooter_TalonAdjusterMotor_Invert);
         * 
         * make sure the encode is going the same direction as the motor
         * can't do this through wpi lib though
         * 
         * some info here:
         * https://www.chiefdelphi.com/t/using-encoder-with-talon-srx/145483/7
         * 
         */
    }

    public void spinShooter(double spinSpeed){
        wheelSpeed = spinSpeed;
        wheelMotors.set(spinSpeed);
        wheelState = MotorStates.RUNNING;
        wheelSpeed = spinSpeed;
    }

    public void spinShooterReverse(double spinSpeed){
        wheelMotors.set(-spinSpeed);
        wheelState = MotorStates.RUNNING;
        wheelSpeed = -spinSpeed;
    }

    public void stopShooter(){
        wheelMotors.set(0);
        wheelMotors.stopMotor();
        wheelState = MotorStates.STOPPED;
        wheelSpeed = 0;
    }

    public void initAdjuster(){

    }

    public void setAdjusterPosition(int position){

    }

    public int getAdjusterPosition(){
        return 0; //read encoder
    }

    
    public boolean atUpperLimit(){
        boolean upperLimitSwitchState = false;

        // read state of limit switch
        // if elevated or retracted, stop the motor
        upperLimitSwitchState =  adjusterMotor.getSensorCollection().isFwdLimitSwitchClosed();
        

        return upperLimitSwitchState;
    }

    public boolean atLowerLimit(){
        boolean lowerLimitSwitchState = false;

        // read state of limit switch
        // if elevated or retracted, stop the motor
        lowerLimitSwitchState =  adjusterMotor.getSensorCollection().isRevLimitSwitchClosed();

        return lowerLimitSwitchState;
    }
    
    @Override
    public void periodic(){
        if (atUpperLimit() || atLowerLimit()){
            adjusterMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }

    }
}