package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;


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
    private Spark feederMotor = null;
    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private SpeedControllerGroup wheelMotors = null;
    
    /**
     * These variables hold the state of the
     * components
     */
    private MotorStates flyWheelState = MotorStates.STOPPED;
    private MotorStates adjusterState = MotorStates.STOPPED;
    private MotorStates feederWheelState = MotorStates.STOPPED;

    private double flyWheelSpeed = 0;

    /**
     * This variable holds the initial encoder position
     * we use this to normalize the position so we can
     * move to predetermined offsets
     */
    private int initialEncoderPosition = 0;

    /**
     * These variables check the DIOs of the Through Bore Encoder(shooter)
     */
    //DigitalInput input1 = new DigitalInput(0);
    //DigitalInput input2 = new DigitalInput(1);
    //DigitalInput input3 = new DigitalInput(2);
    //DigitalInput input4 = new DigitalInput(3);

    Encoder hexShaft_Encoder = null;//(input2,input3,false);

    public Shooter(){

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

        /**
         * initialize the talon motors driving the
         * flywheel
         */ 
        motor1 = new WPI_TalonSRX(RobotMap.Shooter_TalonMotor1_ID);
        motor2 = new WPI_TalonSRX(RobotMap.Shooter_TalonMotor2_ID);
        wheelMotors = new SpeedControllerGroup(motor1, motor2);
        motor1.setNeutralMode(NeutralMode.Coast);
        motor2.setNeutralMode(NeutralMode.Coast);
        motor1.setInverted(RobotMap.Shooter_TalonMotor1_Invert); 
        motor2.setInverted(RobotMap.Shooter_TalonMotor2_Invert);
        Util.configTalon(motor1);
        Util.configTalon(motor2);


        if (RobotMap.Shooter_UseFeederMotor){
            feederMotor = new Spark(RobotMap.Shooter_SparkFeederMotor_ID);
            feederMotor.enableDeadbandElimination(true);
        }

        hexShaft_Encoder = new Encoder(RobotMap.Shooter_EncoderDIO_Port1, RobotMap.Shooter_EncoderDIO_Port2, true, CounterBase.EncodingType.k4X);
        hexShaft_Encoder.setDistancePerPulse(1);
    }

    public void spinShooter(double spinSpeed){
        flyWheelSpeed = spinSpeed;
        wheelMotors.set(spinSpeed);
        flyWheelState = MotorStates.RUNNING;
        flyWheelSpeed = spinSpeed;
    }

    public void spinShooterReverse(double spinSpeed){
        wheelMotors.set(-spinSpeed);
        flyWheelState = MotorStates.RUNNING;
        flyWheelSpeed = -spinSpeed;
    }

    public void stopShooter(){
        wheelMotors.set(0);
        wheelMotors.stopMotor();
        flyWheelState = MotorStates.STOPPED;
        flyWheelSpeed = 0;
    }

    public void spinFeedMotorCW(){
        double speed = RobotMap.Shooter_FeederMotor_Speed;
        if (RobotMap.Shooter_SparkFeederMotor_Invert){
            speed = speed * -1;
        }

        if (RobotMap.Shooter_UseFeederMotor){
            feederMotor.set(speed);
        }
        feederWheelState = MotorStates.RUNNING;
    }

    public void spinFeedMotorCCW(){
        double speed = -RobotMap.Shooter_FeederMotor_Speed;
        if (RobotMap.Shooter_SparkFeederMotor_Invert){
            speed = speed * -1;
        }
        if (RobotMap.Shooter_UseFeederMotor){
            feederMotor.set(speed);
        }
        feederWheelState = MotorStates.RUNNING;
    }

    public void stopFeedMotor(){
        if (RobotMap.Shooter_UseFeederMotor){
            feederMotor.set(0);
            feederMotor.stopMotor();
        }
        feederWheelState = MotorStates.STOPPED;
    }

    public void displayEncoder () {
        SmartDashboard.putNumber("hex encoder value", hexShaft_Encoder.getRate());
        SmartDashboard.putNumber("hex encoder d/p", hexShaft_Encoder.getDistancePerPulse());
    }

    public void updaterEncoderRate () {
        hexShaft_Encoder.getRate();
    }

    @Override
    public void periodic(){
        displayEncoder();
    }
}