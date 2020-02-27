/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SpinnerMove1TransitionCmd extends CommandBase {
  class Histogram {
    int count;
    int blackCount;

    public Histogram(){
      count = 0;
      blackCount = 0;
    }

    public void dump(){
      System.out.println("count:" + count + " black:" + blackCount);
    }
  }

  class TransitionDataClass {
      int sampleCount;
      int unmatchedCount;
      double minTimeInterval;
      double maxTimeInterval;
      double avgTimeInterval;
      double totalTime;
      String matchedColor;
      int longestMatchedColorRun;
      boolean valid;

    public TransitionDataClass() {
      sampleCount = 0;
      unmatchedCount = 0;
      minTimeInterval = 999999; // some large number
      maxTimeInterval = 0;
      avgTimeInterval = 0;
      totalTime = 0;
      matchedColor = "";
      longestMatchedColorRun = 0;
      valid = false;
    }

    public void printHeader(){
      System.out.println("samplecount,unmatchedcount,mintime,maxtime,avgtime,totaltime,matchcolor,valid,longestMatchedColorRun");
    }
    public void print(){
      System.out.println(sampleCount + ","  + unmatchedCount + "," + minTimeInterval + "," + maxTimeInterval + "," + avgTimeInterval + "," + totalTime + "," + matchedColor + "," + valid + "," + longestMatchedColorRun);
    }

    public void dump(){
      System.out.print("samples:" + sampleCount + " ");
      System.out.print("blackcnt:" + unmatchedCount + " ");
      System.out.print("min:" + minTimeInterval + " ");
      System.out.print("max:" + maxTimeInterval + " ");
      System.out.print("avg:" + avgTimeInterval + " ");
      System.out.print("total:" + totalTime + " ");
      System.out.print("color:" + matchedColor + " ");
      System.out.print("longest color run:" + longestMatchedColorRun + " ");
      System.out.println("complete:" + valid);
    }

    public void setNewInterval(double interval){
      sampleCount ++;
      totalTime += interval;

      if (interval < minTimeInterval){
        minTimeInterval = interval;
      }

      if (interval > maxTimeInterval){
        maxTimeInterval = interval;
      }

      avgTimeInterval = totalTime / sampleCount;
    }
    
    public void setColors(Color matched){
      matchedColor = Robot.spinnerSubsystem.colorName(matched);
      if (matched == Color.kBlack) {
        unmatchedCount++;
      }
    }
  }

  int transitionCount = 0;
  Color initColor = Color.kBlack;
  int maxTransitions;
  double initSpeed = RobotMap.Spinner_MotorSpeed;
  double currSpeed = initSpeed;
  double finalSpeed = RobotMap.Spinner_MotorSpeed;
  int boostSpeedPeriods = 1;
  int periodCount = 0;
  TransitionDataClass[] transitionData;
  Histogram[] histogram;
  double prevSampleTime;
  double startTime;
  double timeInterval;
  int sampleCount;
  int blackCount;
  int currColorRun;

  /**
   * Creates a new SpinnerMove1TransitionCmd.
   */
  public SpinnerMove1TransitionCmd(int numTransitions) {
    addRequirements(Robot.spinnerSubsystem);
    maxTransitions = numTransitions;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    transitionData = new TransitionDataClass[maxTransitions + 1];
    for(int i=0; i<maxTransitions; i++){
      transitionData[i] = new TransitionDataClass();
    }
    
    histogram = new Histogram[30];
    for(int i=0; i< 30; i++){
      histogram[i] = new Histogram();
    }

    transitionCount = 0;
    initColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    startTime = Timer.getFPGATimestamp();
    prevSampleTime = startTime;

    periodCount = 0;
    sampleCount = 0;
    blackCount = 0;
    currColorRun = 0;

    currSpeed = initSpeed;
    Robot.spinnerSubsystem.setSpeed(currSpeed);
    Robot.spinnerSubsystem.spinCw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    periodCount++;
    sampleCount++;
  
    double currTime = Timer.getFPGATimestamp();
    double deltaTime = currTime - prevSampleTime;
    transitionData[transitionCount].setNewInterval(deltaTime);
    prevSampleTime = currTime;

    // boost speed for a certain number of periods to get the motor
    // moving with enough power to spin the wheel
    double oldSpeed = currSpeed;
    if (periodCount == boostSpeedPeriods){
      currSpeed = finalSpeed;
      Robot.spinnerSubsystem.setSpeed(currSpeed);
      System.out.println("changing speed from " + oldSpeed + " to " + currSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.spinnerSubsystem.stop();

    System.out.println("vvvvvvvvvvvvvvvvvvvvvvv");
    // dump the data in a format that can be exported to a spreadsheet
    transitionData[0].printHeader();
    for(int i = 0; i < maxTransitions; i++){
      transitionData[i].print();
    }
    System.out.println("------------------------");

    // dump the data in a human digestible format
    for(int i = 0; i < maxTransitions; i++){
      System.out.print("transition: " + i + " ");
      transitionData[i].dump();
    }

    // dump the histogram
    System.out.println("------------------------");
    for(int i = 0; i < histogram.length; i++) {
      if (histogram[i].count !=0 || histogram[i].blackCount != 0){
        System.out.print(i + ": ");
        histogram[i].dump();
        }
    }
    System.out.println("^^^^^^^^^^^^^^^^^^^^^^^^");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Color currColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    Color detColor = Robot.spinnerSubsystem.getLastDetectedColor();
    transitionData[transitionCount].setColors(currColor);

    if (currColor == Color.kBlack){
      blackCount++;
    }
    double seconds = Timer.getFPGATimestamp();
    System.out.println("sample:" + sampleCount + " color:" + Robot.spinnerSubsystem.colorName(currColor) + " seconds:" + seconds + " detected r:" + detColor.red + " g:" + detColor.green + " b:" + detColor.blue);

    // we can't have a transition to or from black
    if ((currColor != Color.kBlack) && (initColor != Color.kBlack)){

      if (currColor != initColor){

        // we know what the next color should be.  If it isn't that color there is a bad match
        // so treat it as black
        Color nextColor = Robot.spinnerSubsystem.getNextColor(initColor);
        if (nextColor != currColor){
          currColor = Color.kBlack;
          System.out.println("last color match was bad");
        }
        else {
          // transition is complete, so mark the entry as valid
          transitionData[transitionCount].valid = true;

          System.out.println(transitionCount + " transitions detected:" + seconds + " color:" + Robot.spinnerSubsystem.colorName(currColor));

          // update the histogram
          if (sampleCount >= histogram.length - 1){
            sampleCount = histogram.length - 1;
          }

          histogram[sampleCount].count++;
          histogram[sampleCount].blackCount += blackCount;

          transitionCount++;
          blackCount = 0;
          sampleCount = 0;
          initColor = currColor;
        }
      }
      else{
        // initColor and currColor are matching and nonblack, so increase color run
        currColorRun++;
      }
    }
    else if (currColor != Color.kBlack) {
      // if we are moving to black, that terminates the current color run
      // so update the longest if current is greater
      if (currColorRun >= transitionData[transitionCount].longestMatchedColorRun){
        transitionData[transitionCount].longestMatchedColorRun = currColorRun;
      }
    }

    // We are done if we hit the transitions we wanted
    if (transitionCount >= maxTransitions){
      System.out.println("isFinished: done");
      return true;
    }
    
    return false;
  }
}
