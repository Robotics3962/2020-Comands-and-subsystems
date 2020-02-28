package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
    private double wheelSpeed = 0;

    /**
     * This variable holds the initial encoder position
     * we use this to normalize the position so we can
     * move to predetermined offsets
     */
    private int initialEncoderPosition = 0;

    private void DiffConfigTalons(WPI_TalonSRX talon){
        //Tells the talon that the max output that it can give is between 1 and -1 which would mean full forward and full backward.
        //There is no allowance currently for anything in between
        talon.configPeakOutputForward(1,0);
        talon.configPeakOutputReverse(-1,0);
    
        //Tells the talon that it should current limit itself so that we don't blow a 40amp breaker.
        talon.configPeakCurrentLimit(40,0);
        talon.enableCurrentLimit(true);
        talon.configContinuousCurrentLimit(40,0);
        //The max output current is 40Amps for .25 of a second
        talon.configPeakCurrentDuration(250, 0);
    
        //Tells the talon that is should only appy 12 volts or less to the motor.
        talon.configVoltageCompSaturation(12,0);
    }

    public Shooter(){

        //initialize the spark motors driving the
        // flywheel
        motor1 = new WPI_TalonSRX(RobotMap.Shooter_TalonMotor1_ID);
        motor2 = new WPI_TalonSRX(RobotMap.Shooter_TalonMotor2_ID);
        wheelMotors = new SpeedControllerGroup(motor1, motor2);
        motor1.setInverted(RobotMap.Shooter_TalonMotor1_Invert); 
        motor2.setInverted(RobotMap.Shooter_TalonMotor2_Invert); 

        DiffConfigTalons(motor1);
        DiffConfigTalons(motor2);

        /**
         * set up the talon with the encoder
         */
        adjusterMotor = new TalonSRX(RobotMap.Shooter_TalonAdjusterMotor_ID);

        //Tells the talon that the max output that it can give is between 1 and -1 which would mean full forward and full backward.
        //There is no allowance currently for anything in between
        adjusterMotor.configPeakOutputForward(1,0);
        adjusterMotor.configPeakOutputReverse(-1,0);
    
        //Tells the talon that it should current limit itself so that we don't blow a 40amp breaker.
        adjusterMotor.configPeakCurrentLimit(40,0);
        adjusterMotor.enableCurrentLimit(true);
        adjusterMotor.configContinuousCurrentLimit(40,0);
        //The max output current is 40Amps for .25 of a second
        adjusterMotor.configPeakCurrentDuration(250, 0);
    
        //Tells the talon that is should only appy 12 volts or less to the motor.
        adjusterMotor.configVoltageCompSaturation(12,0);

        /**
         * invert the direction if necessary
         * talon.setInverted(RobotMap.Shooter_TalonAdjusterMotor_Invert);
         * 
         * read the encoder and set the initial position
         * can't do this through wpi lib though
         * 
         * some info here:
         * https://www.chiefdelphi.com/t/using-encoder-with-talon-srx/145483/7
         * 
         */
    }

    public void spinShooter(double spinSpeed){
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
}