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
     */
    public static final double Intake_Motor_Speed = .5;
    public static final int Intake_SparkMotor_ID = 0;
    public static final boolean Intake_TalonMotor_Invert = false;
    public static final int Intake_Pneumatic_Forward_Solenoid_ID = 0;
    public static final int Intake_Pneumatic_Reverse_Solenoid_ID = 1;
    public static final double Intake_Spin_TimeMs = .25;

    /**
     * Configuration for the shooter
     * 
     */
    public static final int Shooter_TalonMotor1_ID = 7;
    public static final int Shooter_TalonMotor2_ID = 8;
    public static final boolean Shooter_TalonMotor1_Invert = false;
    public static final boolean Shooter_TalonMotor2_Invert = false; // could be true, not sure
    public static final double Shooter_Speed = .8;
    public static final int Shooter_SparkFeederMotor_ID = 4;
    public static final double Shooter_FeederMotor_Speed = .5;
    
    /**
     * Configuration for the ball lift
     */
    public static int Lift_SparkMotor_ID = 2;
    public static boolean Lift_TalonMotor1_Invert = false;
    public static boolean Lift_TalonMotor2_Invert = false;
    public static double Lift_MotorSpeed = .6;
    public static int Lift_IndexTimeMilliSeconds = 40;

    /**
     * configuration for the climber.  The climber needs limit switches
     * so need to use a talon controller for each motor.  To climb,
     * the climber will extend until the upper limit switches are
     * activated.  Then it will retract until the lower limit switches
     * are activated.
     */
    public static int Climber_TalonMotor1_ID = 11;
    public static int Climber_TalonMotor2_ID = 12;
    public static boolean Climber_TalonMotor1_Invert = false;
    public static boolean Climber_TalonMotor2_Invert = false;
    public static double Climber_MotorSpeedUp = .5;
    public static double Climber_MotorSpeedDown = .5;

    /**
     * Configuration for the spinner.  The spinner consists of
     * a color sensor a single action pneumatic solenoid 
     */
    public static final int Spinner_SparkMotor_ID = 1;
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
