package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.HashMap;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class Spinner extends SubsystemBase {
   /**
     * Users Guide
     * 
     */

    /**
     * used to keep track of if spinner is deployed
     */
    private enum SolenoidStates { UNKNOWN, EXTENDED, RETRACTED};

     /**
     * used to keep track of current state
     */
    private enum MotorStates {STOPPED, RUNNING};

    /**
     * declare variables tracking current state of intake
     * and initialize them
     */
    private MotorStates motorState = MotorStates.STOPPED;
    private SolenoidStates solenoidState = SolenoidStates.UNKNOWN;

    /**
     * these are the controllers for the motors 
     */
    private Spark motor;
    private DoubleSolenoid solenoid;

    /**
     * used to map color under Control Panel sensor to color under
     * robot sensor
     */
    HashMap<Color, Color> colorDetectorToRobotSensorMap;

    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Match object is used to register and detect known colors. This can 
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private ColorMatch colorMatcher = new ColorMatch();

    /**
     * Calibrated blue, green, red values for the swatches swatch.
     * this was stolen from https://github.com/REVrobotics/Color-Sensor-v3/commit/34469e7c21dc4495b06e6f31fb3df02e2335c8de#diff-3bfa73a3527f6f2d2159702ac78e5165
     */
    private final Color detectorCyanColor = ColorMatch.makeColor(0.12, 0.42, 0.45);
    private final Color detectorGreenColor = ColorMatch.makeColor(0.16, 0.58, 0.25);
    private final Color detectorRedColor = ColorMatch.makeColor(0.53, 0.56, 0.12);
    private final Color detectorYellowColor = ColorMatch.makeColor(0.31, 0.5607, 0.1201);

    // lights out color
    // green .16, .58, .25
    // cyan  .12, .42, .45
    // yellow .31, .56, .12
    // red .53, .34, .12

    // lights on color
    // green .16, .58, .25
    // cyan  .12, .42, .45
    // yellow .32, .56, .11
    // red   .53, .34, .13

    private HashMap<Color, String> colorNameMap;
    private HashMap<String, Color> name2ColorMap;
    /**
     * set to true to display the color under the robot sensor
     */
    private boolean displayColor = true;

    /**
     * keeps track of how many color transitions occurred
     */
    private Color targetColor;

    /** 
     * this tracks the last number of samples from the color detector
     * if we have 1 sample of n that is different, then we transitioned
     */
    //private final int MaxSamples = RobotMap.Spinner_SampleCount;
    //private final int SameColorSampleCount = RobotMap.Spinner_ContinuousColorsForTransition;
    private ArrayList<Color> samples;
    private enum CommandStates { NOT_RUNNING, RUNNING, MOVING_TO_CENTER_OF_WEDGE, COMPLETE };
    private CommandStates commandStatus;
    private boolean collectSamples;
    private int transitionCount;
    private int samplesBetweenTransitions;
    private int periodicCallsBeforeStop;
    private int transitionsNeededToTargetColor;
    private int sampleCount;
    private double speed;

    public Spinner(){

        /**
         * User Guide
         * 
         * external References:
         *      Spinner_MotorSpeed is the speed to spin the motor
         *      Spinner_SparkMotor_ID is the id of the spark controllers
         *          used for the motor
         *      Spinner_Pneumatic_Forward_Solenoid_ID is the id of the forward action
         *          of the solenoid used to deploy the arm with the color sensor and wheel/motor
         *      Spinner_Pneumatic_Reverse_Solenoid_ID is the id of the reverse action
         *          of the solenoid used to deploy the arm with the color sensor and wheel/motor
         *      Spinner_TargetColorTransitions is the number of transitions to the target
         *          color we need to hit in order to complete the charge-up task
         * 
         * init():
         *      this function will put the spinner in its starting state:
         *          *arm will be retracted
         * 
         * interrupt():
         *      this class does runs a state machine in its periodic function.  Because
         *      of this, if a command is interrupted it will not stop this subsystem from
         *      continuing to execute the current task (if any).  To alleviate this situation
         *      any Command which requires this subsystem needs to call this function when
         *      the command is interrupted.
         * 
         * spinCW():
         *      this function will spin the motor clockwise.  Please note this results
         *      in the actual control station wheel spinning counter-clockwise. For this
         *      class direction really doesn't matter much as the spinToTarget() function is
         *      direction agnostic.
         * 
         * spinCCw():
         *      this function will spin the motor counter-clockwise.  Please note this reuslt
         *      in the actual control station wheel spinning clockwise. It spins at the preset
         *      speed.
         * 
         * stop():
         *      this function will stop the motor from spinning
         * 
         * isSpinning():
         *      this function returns true if the motor is spinnand and false if the
         *      motor is stopped
         * 
         * extend():
         *      this function is called to extend the arm holding the sensor and the motor
         * 
         * isExtended():
         *      this function returns true if the arm is extended and false if it is not
         * 
         * retract():
         *      this function is called to retracet the arm holding the sensor and the motor.
         * 
         * isRetracted():
         *      this function returns true of the arm is retracted and false if it is not
         * 
         * getMatchedSensorColor():
         *      this function will return the preset color (red,green,cyan,yellow) that is
         *      closest to the color under the sensor.  If there is no match, black will be
         *      returned.  If display of color data is enabled, it will update the shuffleboard
         *      (via network tables) with the detected color and the matched color.
         *      it return the matched color.
         * 
         * spinToTarget():
         *      this function is called to start  the spinner executing the controlstation
         *      changeup task.  It does the following tasks:
         *          1.) gets the color that needs to be under the control station color sensor
         *              at the completion of the chargeup-task.
         *          2.) maps it to the color that needs to be under the robots color sensor
         *              in order for the right color to be under the control stations sensor
         *          3.) starts the wheel spining.
         *      all of the rest of the work is implemented as a state machine in periodic()
         *      which is called every 20 millseconds (20ms)
         * 
         * spinToTargetComplete():
         *      this function returns true if the spinToTarget has completed, and false
         *      if it is still in progress or has never started. (this should be moved to a command)
         * 
         * updateDashboardWithSensorColor():
         *      this function checks to see if displaying colors is enabled (see enabledDisplayColor)
         *      if it is, it will push out the colors via network tables to the shuffleboard
         *      and driverstation.
         * 
         * enableDisplayColor():
         *      this function will enable the display of color information via network tables
         * 
         * disableDisplayColor():
         *      this function will disable the display of color information via network tables.
         * 
         */

        transitionsNeededToTargetColor = RobotMap.Spinner_TargetColorTransitions;
        transitionCount = 0;
        samplesBetweenTransitions = 0;
        collectSamples = false;
        commandStatus = CommandStates.NOT_RUNNING;
        samples = new ArrayList<Color>();
        targetColor = new Color(0,0,0);
        sampleCount = 0;

        /**
         * indicate the motors have not been started
         */
        motorState = MotorStates.STOPPED;

        /** 
         * populate the hash mapping the color under the Control 
         * panel sensor to the color under the robot sensor
         */
        colorDetectorToRobotSensorMap = new HashMap<Color, Color>();
        colorDetectorToRobotSensorMap.put(detectorGreenColor, detectorRedColor);
        colorDetectorToRobotSensorMap.put(detectorRedColor, detectorYellowColor);
        colorDetectorToRobotSensorMap.put(detectorYellowColor, detectorCyanColor);
        colorDetectorToRobotSensorMap.put(detectorCyanColor, detectorGreenColor);

        colorMatcher.addColorMatch(detectorCyanColor);
        colorMatcher.addColorMatch(detectorGreenColor);
        colorMatcher.addColorMatch(detectorRedColor);
        colorMatcher.addColorMatch(detectorYellowColor);
        colorMatcher.setConfidenceThreshold((0.80));
        // initialize the motor
		motor = new Spark(RobotMap.Spinner_SparkMotor_ID);
        motor.enableDeadbandElimination(true);
        speed = RobotMap.Spinner_MotorSpeed;

        // initialize the solenoid
        solenoid = new DoubleSolenoid(RobotMap.Pneumatic_Module_ID, RobotMap.Spinner_Pneumatic_Forward_Solenoid_ID, RobotMap.Spinner_Pneumatic_Reverse_Solenoid_ID);

        colorNameMap = new HashMap<Color, String>();
        colorNameMap.put(detectorCyanColor, "Blue");
        colorNameMap.put(detectorRedColor, "Red");
        colorNameMap.put(detectorGreenColor, "Green");
        colorNameMap.put(detectorYellowColor, "Yellow");
        colorNameMap.put(Color.kBlack, "Black");

        name2ColorMap = new HashMap<String, Color>();
        name2ColorMap.put("Blue", detectorCyanColor);
        name2ColorMap.put("Red", detectorRedColor);
        name2ColorMap.put("Green", detectorGreenColor);
        name2ColorMap.put("Yellow", detectorYellowColor);
        name2ColorMap.put("Black", Color.kBlack);
    }

    public String colorName(Color color){
        return colorNameMap.get(color);
    }

    public Color nameToColor(String name){
        return name2ColorMap.get(name);
    }

    /** 
     * because work is done in the periodic function, when a command which uses this 
     * subsystem is interrupted, we need to call this function to stop periodic() from
     * executing any actions.
     */
    public void interrupt(){
        // stop the motor
        commandStatus = CommandStates.NOT_RUNNING;
        stop();

    }

    public void setSpeed(double newSpeed){
        speed = newSpeed;

    }
    public void spinCw(){
        motor.set(speed);
        motorState = MotorStates.RUNNING;
    }

    public void spinCCw(){
        motor.set(-speed);
        motorState = MotorStates.RUNNING;
    }

    public void stop(){
        motor.set(0);
        motorState = MotorStates.STOPPED;
    }

    public boolean isSpinning(){
        return (motorState == MotorStates.RUNNING);
    }

    /**
     * these function control the pneumatics to extend
     * the arm
     */
    public void extend(){
        solenoid.set(DoubleSolenoid.Value.kForward);
        solenoidState = SolenoidStates.EXTENDED;
    }

    public boolean isExtended(){
        return (solenoidState == SolenoidStates.EXTENDED);
    }

    public void retract(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
        solenoidState = SolenoidStates.RETRACTED;
    }

    public boolean isRetracted(){
        return (solenoidState == SolenoidStates.RETRACTED);
    }
    
    /**
     * utility functions
     */

    public Color getNextColor(Color currColor){
        Color next = Color.kBlack;
        String name = colorName(currColor);
        switch (name){
            case "Blue":
                next = detectorYellowColor;
                break;
            case "Yellow":
                next = detectorRedColor;
                break;
            case "Red":
                next = detectorGreenColor;
                break;
            case "Green":
                next = detectorCyanColor;
                break;
        }
        return next;
    } 

    Color detectedColor = Color.kBlack;
    public Color getLastDetectedColor(){
        return detectedColor;
    }

    public Color getMatchedSensorColor(){
        detectedColor = colorSensor.getColor();
        Color matchedColor = Color.kBlack;
        double confidence = 0;

        /**
         * match the color read from the sensor to one of the
         * pre defined ones.  if the color is black, then there is
         * no match
         */
        ColorMatchResult match = colorMatcher.matchColor(detectedColor);
        if (match != null){
            matchedColor = match.color;
            confidence = match.confidence;
        }

        // display it on the dashboard
        updateDashboardWithSensorColor(detectedColor);
        updateDashboardWithMatchedColor(matchedColor, confidence);

        return matchedColor;
    }

    private boolean setTargetColor(){
        boolean success = true;
        Color color = new Color(0,0,0);

        /**
         * read the color to match from the network table
         */
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        
        /**
         * it may not be set yet, so return false if it has noe
         */
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    color = detectorCyanColor;
                    break;
                case 'G' :
                    color = detectorGreenColor;
                    break;
                case 'R' :
                    color = detectorRedColor;
                    break;
                case 'Y' :
                    color = detectorYellowColor;
                    break;
                default :
                    success = false;
                break;
            }
        }
        else {
            success = false;
        }

        if (success){
            // color is the color under the control panel sensor
            // we need to get the color under the robot sensor
            targetColor = colorDetectorToRobotSensorMap.get(color);
        }
        else{
            targetColor = Color.kBlack;
        }
        return success;
    }

    public boolean spinToTarget(){
        boolean success = false;

        // if the motor is spinning stop it
        if (commandStatus == CommandStates.RUNNING){
            stop();
            commandStatus = CommandStates.NOT_RUNNING;
        }

        success = setTargetColor();
        if (success){

            /**
             * reset variables holding calculations from a
             * previous run
             */
            transitionsNeededToTargetColor = RobotMap.Spinner_TargetColorTransitions;
            transitionCount = 0;
            samplesBetweenTransitions = 0;
            sampleCount = 0;
    
            /**
             * clear out the samples from previous runs
             */
            samples.clear();

            // indicate the command is not running
            commandStatus = CommandStates.RUNNING;

            // start the motor
            spinCw();

            // start taking samples
            collectSamples = true;

            /**
             * all of the work is done in the state machine run by 
             * the periodic function.  The periodic function should
             * (more or less) be called every 20 milliseconds.  so
             * the processing to spin the wheel may not happen for 20ms.
             * 
             * this is actually ok, because it will give time for the motor
             * to get up to speed (20ms is probably too short, but it's
             * better then nothing)
             */
            success = true;
        }

        return success;
    }

    public void updateDashboardWithSensorColor(Color color){
        if (displayColor){
            SmartDashboard.putNumber("sensor-Red", color.red);
            SmartDashboard.putNumber("sensor-Green", color.green);
            SmartDashboard.putNumber("sensor-Blue", color.blue);
        }
    }

    public void updateDashboardWithMatchedColor(Color color, double confidence){
        if (displayColor){
            SmartDashboard.putNumber("Matched-Red", color.red);
            SmartDashboard.putNumber("matched-Green", color.green);
            SmartDashboard.putNumber("Matched-Blue", color.blue);
            SmartDashboard.putNumber("Matched-Confidence", confidence);
        }
    }

    public void enableDisplayColor(){
        displayColor = true;
    }

    public void disableDisplayColor(){
        displayColor = false;
    }

    void executeRunningState(){

        do {
            // check if we are supposed to collect samples
            if (!collectSamples){
                break;
            }
            
            // collect a sample
            Color detectedColor = getMatchedSensorColor();
            sampleCount++;

            /**
             * black is returned when there is no closest
             * match to the color sensor. This can occur when
             * the sensor is on a transition between two color wedges
             * and there is no close match.
             */
            if (detectedColor == Color.kBlack){
                break;
            }

            samples.add(detectedColor);

            // we need to have collected a min of 2 samples to process
            if(samples.size() < 2){
                break;
            }

            /**
             * we only need two samples, so if we have more get rid
             * of the first one
             */
            if (samples.size() > 2){
                samples.remove(0);
            }        

            /**
             * we have enough samples to process.
             * 
             * a transition occurs when the first sample
             * is different from the last sample, and the last N samples
             * are the same
             */
            Color firstSampleColor = samples.get(0);
            Color lastSampleColor = samples.get(samples.size()-1);

            // if the first color and the last color are the same
            // we can't possible have hit a transition
            if (firstSampleColor == lastSampleColor){
                break;
            }
             
            /**
             * at this point we know we encounted a transition because the
             * previous sample and the current sample are different 
             */            
            transitionCount++;
    
            /**
             * we want to keep track of the number of samples taken between transitions.
             * this is effectively the number of times periodic has been called between
             * transitions.
             * 
             * we use this to get some idea of the distance/time it takes to traverse a wedge.
             * 
             * we need to ignore the first transition because we don't know where in the wedge
             * the robot sensor started.  It could be right next to a transition which would
             * throw off our calculations.
             */
            if(transitionCount > 1){
                samplesBetweenTransitions += sampleCount;
            }

            sampleCount = 0;

            /** 
             * at this point we know we encountered a transition from one wedget to another.
             * 
             * there are two wedges on the wheel for each color.  The same colored wedges
             * are 180 degrees apart. So to spin the wheel once we need to transition to a
             * color twice.
             * 
             * to spin the wheel 3 times we need to transition to a color 6 times.
             * 
             * we know the color that we want to end up under our sensor so we need to 
             * keep spinning until the robot sensor is over the target color.
             * 
             * To make this code simpler, are going to transition to the target color
             * 7 times.  This may mean we spin the wheel an extra half spin, but that is ok
             * because we can spin the wheel up to 5 times.
             */

            /**
             * the color under the sensor is not the one we are lookng for
             * (thinks about some comment about droids) so quit
             */
             if (detectedColor != targetColor){
                break;
            }
            
            /** 
             * we did transition to our target color, we need to check if we are done.
             */
            transitionsNeededToTargetColor--;
            
            /**
             * we are done when we have transitioned x number of times
             * if we haven't quit
             */
            if (transitionsNeededToTargetColor > 0){
                break;
            }

            /**
             * At this point, we have spun the wheel upf to 3.5 times and want to
             * center our sensor in the color wedge.
             * 
             * to do this we transition the state from RUNNING to MOVING _TO_CENTER_OF_WEDGE
             * and quit.
             * 
             * How do we know how far to move to the center of the wedge?
             * 
             * All along we have been keeping track of the number of calls to periodic through
             * our samples array.  We keep accumulating the results in samplesBetweenTransitions.
             * Now we divide that by the number of transitions - 1. (-1 because we need to ignore
             * the first transition which may not have traversed the entire wedge).  Because we only
             * want to move to the center of the wedge, we divide that number by 2
             */
            periodicCallsBeforeStop = (samplesBetweenTransitions/(transitionCount-1))/2;
            commandStatus = CommandStates.MOVING_TO_CENTER_OF_WEDGE;
            collectSamples = false;

        } while(false);
    }

    private void executeMoveToOffsetState(){
        /** 
         * stop the motor have a certain number of calls.  this should stop 
         * wheel where the robot sensor in approximately in the middle of the 
         * target color wedge
         */
        periodicCallsBeforeStop--;
        if (periodicCallsBeforeStop <= 0){
            stop();
            commandStatus = CommandStates.COMPLETE;
        }
    }

    public boolean spinToTargetComplete(){
        return (commandStatus == CommandStates.COMPLETE);
    }

    @Override
    public void periodic(){
        getMatchedSensorColor();
        switch(commandStatus){
            case NOT_RUNNING:
                // nothing to do
                break;
            case RUNNING:
                executeRunningState();
                break;
            case COMPLETE:
                // nothing to do
                break;
            case MOVING_TO_CENTER_OF_WEDGE:
                executeMoveToOffsetState();
                break;
        }        
    }

}
