package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {


    //limelight speed values
   public static double rotation = 0.45;
   public static double speed = 0.45;

    /**
     * This value is to make it clear I don't now the value
     */
    private static final int UNKNOWN_VALUE = -1;

    /**
     * this is the id of the pneumatic controller
     */
    public static final int Pneumatic_Module_ID = 0; 

    /**
     * Configuration for the intake
     * 
     * The intake has a motor to spin it and a pneumatic solenoid
     * to lower it
     * 
     * Intake_Motor_Speed is a value between -1.0 and 1.0 indicating 
     *      forward and reverse, the further the value is from
     *      0, the greater the speed
     * 
     * Intake_TalonkMotor_Id is the id of the talon motor controller
     * 
     * Intake_Pneumatic_Module_ID is the is the the pneumatics cotrol module
     *      (PCM)
     * Intake_Pneumatic_Forward_Solenoid_ID  is the id of the port the solenoid
     *      the pushes the solenoid forward is wired into 
     * 
     * Intake_Pneumatic_Reverse_Solenoid_ID  is the id of the port the solenoid
     *      the pushes the solenoid reverse is wired into 
     */

    public static final double Intake_Motor_Speed = .7;
    public static final int Intake_SparkMotor_ID = 0;
    public static final boolean Intake_SparkMotor_Invert = false;
    public static final int Intake_Pneumatic_Forward_Solenoid_ID = 0;
    public static final int Intake_Pneumatic_Reverse_Solenoid_ID = 1;
    public static final double Intake_Spin_TimeMs = .25;

    /**
     * Configuration for the shooter
     * 
     * the shooter has two motors to spin the shooting wheel and
     * one motor to move the angle adjuster up and down.  the 
     * adjuster will need to be encoded.
     * 
     * Shooter_SparkMotor1_ID is the id of the first motor used to spin
     *      the shooter wheel
     * 
     * Shooter_SparkMotor2_ID is the id of the second motor used to spin
     *      the shooter wheel
     * 
     * Shooter_Speed is the speed to spin the shooter at (we may need multiple speeds
     *      to change the distance/angle the ball travels at)
     * 
     * Shooter_Talon_Motor_ID is the id of the talon controller.  This motor needs to
     *      be encoded
     * 
     * Shooter_Adjuster_MaxPostion is the max enteded position of the adjuster, moving past this
     *      would cause physical damage
     * 
     * Shooter_Adjuster_MinPosition is the minimim position of the adjuster, moving past this point
     *      would cause physical damage
     * 
     * Shooter_Adjuster_Speed is the speed which we will be moving the adjuster at.
     * 
     * NOTE: the motor to move the adjuster needs to have limit switches wired into the
     *      talon so that we do not break the adjuster
     */
    public static final int Shooter_TalonMotor1_ID = 7;//which is top, which is bot?
    public static final int Shooter_TalonMotor2_ID = 8;
    public static final boolean Shooter_TalonMotor1_Invert = false;
    public static final boolean Shooter_TalonMotor2_Invert = true;
    public static final double Shooter_Speed = 1.0;
    public static final double Shooter_TimeForFlyWheelToSpinUp = 2.5;
    public static final int Shooter_SparkFeederMotor_ID = 3;//4
    public static final boolean Shooter_SparkFeederMotor_Invert = false;
    public static final double Shooter_FeederMotor_Speed = 0.5;
    public static final boolean Shooter_UseFeederMotor = true;
    public static final int Shooter_EncoderDIO_Port1 = 0;
    public static final int Shooter_EncoderDIO_Port2 = 1;

    /**
     * Configuration for the ball lift
     * 
     * Lift_SparkMotor_ID is the id of the spark controller used to
     *      spin the motors for the lift
     * 
     * Lift_MotorSpeed is the speed between -1.0 and 1.0 that the
     *      ball lift motor will spin at
     * 
     * Lifs_IndexTimeMilliseconds is the amount of time the lift motor
     *      will run to move a ball up to make room for another ball.
     */
    public static final int Lift_SparkMotor_ID = 2;////mech calls this uptake
    public static final boolean Lift_Motor1_Invert = false;
    public static final double Lift_MotorSpeed = .6;
    public static final double  Lift_IndexTimeMilliSeconds = 0.25;

    /**
     * configuration for the climber.  The climber needs limit switches
     * so need to use a talon controller for each motor.  To climb,
     * the climber will extend until the upper limit switches are
     * activated.  Then it will retract until the lower limit switches
     * are activated.
     * 
     * Climber_TalonMotor1_ID is the id of the first motor
     * 
     * Climber_TalonMotor2_ID is the id of the second motor
     * 
     * Climber_Speed is the value between -1.0 and 1.0 that the
     * the talon motors will spin at 
     */
    public static int Climber_TalonMotor1_ID = 5;
    public static int Climber_TalonMotor2_ID = 6;
    public static boolean Climber_TalonMotor1_Invert = false;
    public static boolean Climber_TalonMotor2_Invert = false;
    public static double Climber_MotorSpeedUp = .4;
    public static double Climber_MotorSpeedDown = -.25;

    /**
     * Configuration for the spinner.  The spinner consists of
     * a color sensor a single action pneumatic solenoid 
     * and a spark controller motor
     * 
     * Spinner_SparkMotor_ID is the is of the spark controller
     *  for the motor
     * 
     * Spinner_MotorSpeed is a value between -1.0 and 1.0 which
     * is the speed the spinner motor runs at
     */
    public static final int Spinner_SparkMotor_ID = 1;//correct assignment, mech calls wheel
    public static final double Spinner_MotorSpeed = 0.15;
    public static final double Spinner_SlowMotorSpeed = 0.1;
    public static final int Spinner_TargetColorTransitions = 7;

    /**
     * 
     * This section contains constants to define the drive motors
     * 
     */
    // use these ids for ghetto bot
    //public static final int Drive_TalonLeftFront_ID = 2;
    //public static final int Drive_TalonLeftRear_ID  = 3;
    //public static final int Drive_TalonRightFront_ID = 4;
    //public static final int Drive_TalonRightRear_ID = 1;
    public static final int Drive_TalonLeftFront_ID = 3;
    public static final int Drive_TalonLeftRear_ID  = 4;
    public static final int Drive_TalonRightFront_ID = 1;
    public static final int Drive_TalonRightRear_ID = 2;
    public static final double Drive_Auto_Distance_Pval = 0.18;//0.1
    public static final double Drive_Auto_Distance_Ival = 0.0;
    public static final double Drive_Auto_Distance_Dval = 0.0;
    public static final double Drive_Auto_CountsPerInch = 218;
    public static final double Drive_Auto_Distance_MinSpeed = 0.15;
    public static final double Drive_Auto_Distance_MaxSpeed = 0.6;
    public static final double Drive_Auto_Distance_DeadZone = Drive_Auto_CountsPerInch * 3; 
    public static final double Drive_Auto_Angle_DeadZone = 1;
    public static final double Drive_Auto_Angle_MinSpeed = 0.3;
    public static final double Drive_Auto_Angle_MaxSpeed = 0.5;
    public static final double Drive_Auto_Angle_Pval = 0.25;
    public static final double Drive_Auto_Angle_Ival = 0.0;
    public static final double Drive_Auto_Angle_Dval = 0.0;
    public static final double Drive_LimeLight_Search_RotateSpeed = 0.45;
    public static final double Drive_Auto_StabilizedTime = 1.0;
    public static final double Drive_Limelight_Search_Tolerance = 0.5;
    public static final int Drive_Limelight_NoProgressCnt = 10;

    /**
     * These scale factors are used to scale the values from the
     * joystick.  The joystick will return a value between -1.0 and 1.0
     * 
     * -1.0 and 1.0 are powering the motors at full speed.  Full speed
     * can be too fast and damange the roobot (make it fall over, etc)
     * 
     * Applying these scale factor will effectively scale the joystick
     * value from 0 to the scale factor in each direction.  t
     * 
     * The scale factor isn't a max limit.  It scales all of the values
     * from 0 to 1.0 (or -1.0) to 0 to scalefactor (or -scalefactor)
     * which allows finer control of the robot
     */
    public static final double Drive_SpeedScaleFactor = 0.5;
    public static final double Drive_RotationScaleFactor = 0.5;
    
    /**
     * 
     * This section defines constants for configuring the joy sticks
     * 
     **/


    // Joystick to use
    public static final int Joystick0Id = 0;
    public static final int Joystick1Id = 1;

    // joystick axis mapping
    public static final int JoystickAxisSpeed = 0;
    public static final int JoystickAxisRotation = 1;
    public static final double JoystickDeadZone = 0.05;
}