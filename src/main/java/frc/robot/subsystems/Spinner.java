package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Solenoid;
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
    private enum SolenoidStates { NOT_EXTENDED, EXTENDED};

     /**
     * used to keep track of current state
     */
    private enum MotorStates {STOPPED, RUNNING};

    /**
     * declare variables tracking current state of intake
     * and initialize them
     */
    private MotorStates motorState = MotorStates.STOPPED;
    private SolenoidStates solenoidState = SolenoidStates.NOT_EXTENDED;

    /**
     * these are the controllers for the motors 
     */
    private Spark motor;
    private Solenoid solenoid;

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
     * This is our best guess of the rgb values of each color
     * They are used as a starting point, and filled in by manually
     * checking the sensor
     */
    private final Color detectorCyanColor = ColorMatch.makeColor(0, 255, 255);
    private final Color detectorGreenColor = ColorMatch.makeColor(0, 255, 0);
    private final Color detectorRedColor = ColorMatch.makeColor(255, 0, 0);
    private final Color detectorYellowColor = ColorMatch.makeColor(255, 255, 0);

    /**
     * set to true to display the color under the robot sensor
     */
    private boolean displayColor = false;

    /**
     * keeps track of how many color transitions occurred
     */
    private Color targetColor;

    /** 
     * this tracks the last number of samples from the color detector
     * if we have 1 sample of n that is different, then we transitioned
     */
    private final int MaxSamples = RobotMap.Spinner_SampleCount;
    private final int SameColorSampleCount = RobotMap.Spinner_ContinuousColorsForTransition;
    private ArrayList<Color> samples;
    private enum CommandStates { NOT_RUNNING, RUNNING, MOVING_TO_CENTER_OF_WEDGE, COMPLETE };
    private CommandStates commandStatus;
    private boolean collectSamples;
    private int transitionCount;
    private int samplesBetweenTransitions;
    private int periodicCallsBeforeStop;
    private int transitionsNeededToTargetColor;

    public Spinner(){

        transitionsNeededToTargetColor = RobotMap.Spinner_TargetColorTransitions;
        transitionCount = 0;
        samplesBetweenTransitions = 0;
        collectSamples = false;
        commandStatus = CommandStates.NOT_RUNNING;
        samples = new ArrayList<Color>();
        targetColor = new Color(0,0,0);

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

        // initialize the motor
		motor = new Spark(RobotMap.Spinner_SparkMotor_ID);
        motor.enableDeadbandElimination(true);

        // initialize the solenoid
        solenoid = new Solenoid(RobotMap.Pneumatic_Module_ID, RobotMap.Spinner_Pneumatic_Forward_Solenoid_ID);
    }


    public void spinCw(){
        motor.set(RobotMap.Spinner_MotorSpeed);
        motorState = MotorStates.RUNNING;
    }

    public void spinCCw(){
        motor.set(-RobotMap.Spinner_MotorSpeed);
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

    /**
     * these function control the pneumatics to extend
     * the arm
     */
    public void extend(){
        solenoid.set(true);
        solenoidState = SolenoidStates.EXTENDED;
    }

    public boolean isExtended(){
        return (solenoidState == SolenoidStates.EXTENDED);
    }

    /**
     * utility functions
     */
    public Color getSensorColor(){
        Color detectedColor = colorSensor.getColor();
    
        /**
         * match the color read from the sensor to one of the
         * pre defined ones
         */
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        // display it on the dashboard
        updateDashboardWithSensorColor(detectedColor);
        updateDashboardWithMatchedColor(match.color);

        return match.color;
    }

    public boolean setTargetColor(){
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
            targetColor = new Color(0.0, 0.0, 0.0);
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

    public void updateDashboardWithMatchedColor(Color color){
        if (displayColor){
            SmartDashboard.putNumber("Matched-Red", color.red);
            SmartDashboard.putNumber("matched-Green", color.green);
            SmartDashboard.putNumber("Matched-Blue", color.blue);
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
            Color detectedColor = getSensorColor();
            samples.add(detectedColor);

            // check if we have enough samples to process
            if(samples.size() <= MaxSamples){
                break;
            }

            // throw away the first sample as we have a new one to
            // replace it we added to the end of the list
            samples.remove(0);        

            /**
             * we have enough samples to process.
             * 
             * a transition occurs when the first sample
             * is different from the last sample, and the last N samples
             * are the same
             */

            // if MaxSamples is 1, this will throw an exception
            // but a MaxSamples of 1 makes no sense
            if (MaxSamples <= 1){
                int y = 2/0;
            }

            Color firstSampleColor = samples.get(0);
            Color lastSampleColor = samples.get(samples.size()-1);

            // if the first color and the last color are the same
            // we can't possible have hit a transition
            if (firstSampleColor == lastSampleColor){
                break;
            }

            if (SameColorSampleCount >= MaxSamples){
                // panic as this should never happen
                int y = 2 / 0;
                break;
            }

            int matchingSampleCount = 0;

            // walk the array backwards looking for a color change
            for (int idx = samples.size(); idx > 0; idx--){
                // correct index because 0 is the first array index
                int index = idx - 1;

                // check if the sample is the same color as the last sample
                if (samples.get(index) == lastSampleColor){
                    matchingSampleCount++;
                    break;
                }   
            }

            /**
             * now matching same count holds the number of contigious samples
             * starting from the end of the samples that match the last
             * sample collected. 
             */

            /**
             * we need to get SameColorSampleCount samples
             * to meet the definition of making a transition.
             * If there are less than that, we cannot be at a transition
             * so stop processing
             */
            if (matchingSampleCount < SameColorSampleCount){
                break;
            }
             
            /**
             * at this point we know we encounted a transition. Samples would be 
             * something like   Red,Red, purple, BLUE, BLUE, BLUE
             * we moved from a red wedge to a blue wedge and the third value
             * was sampled when both the red and blue wedge were under the sensor.
             * 
             * because we have 3 blues in a row we know we are in a blue wedge and 
             * transitioned from a red wedge.
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
                samplesBetweenTransitions += matchingSampleCount;
            }

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

    @Override
    public void periodic(){
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
