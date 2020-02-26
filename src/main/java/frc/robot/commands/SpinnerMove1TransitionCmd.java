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

public class SpinnerMove1TransitionCmd extends CommandBase {
  class TransitionData
      int sampleCount;
      double minTimeInterval;
      double maxTimeInterval;
      double avgTimeInterval;
      double totalTime;
      Color detectedColor;
      Color matchedColor;
      boolean valid;

    TansitionData() {
      sampleCount = 0;
      minTimeInterval = 9999999;
      maxTimeInterval = 0;
      avgTimeInterval = 0;
      totalTime = 0;
      detectedColor = color.kWhite;
      matchedColor = Color.kWhite;
      valid = false;;
    }
  
  }

  /**
   * Creates a new SpinnerMove1TransitionCmd.
   */
  int transitionCount = 0;
  Color initColor = Color.kBlack;
  double maxTransitions;
  double initSpeed = .2;
  double currSpeed = initSpeed;
  double finalSpeed = .15;
  int reduceSpeedPeriods = 1;
  int periodCount = 0;
  TransitionData[] transitionData;
  double prevSampleTime;
  double startTime;
  double endTime;

  public SpinnerMove1TransitionCmd(double numTransitions) {//set to seconds
    addRequirements(Robot.spinnerSubsystem);
    maxTransitions = numTransitions;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transitionData = new TransitionData[maxSamples];
    transitionCount = 0;
    initColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    startTime = Timer.getFPGATimestamp();
    prevSampleTime = startTime;;

    periodCount = 0;
    currSpeed = initSpeed;
    Robot.spinnerSubsystem.setSpeed(currSpeed);
    Robot.spinnerSubsystem.spinCw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    periodCount++;
    sampleCount++;

    double oldSpeed = currSpeed;
    if (periodCount == reduceSpeedPeriods){
      currSpeed = finalSpeed;
      Robot.spinnerSubsystem.setSpeed(currSpeed);
      Robot.spinnerSubsystem.spinCw();
      System.out.println("changing speed from " + oldSpeed + " to " + currSpeed);
    }

    if (initColor == Color.kBlack){
      initColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    }

    Color color = Robot.spinnerSubsystem.getMatchedSensorColor();
    if (color==Color.kBlack){
      //System.out.println("Color is " + Robot.spinnerSubsystem.colorName(color));
    }

    if (startTime < endTime){
      //System.out.println("Start Time: "+startTime +";;;;; End Time: "+endTime);
      Robot.spinnerSubsystem.setSpeed(currSpeed);
      Robot.spinnerSubsystem.spinCw();
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.spinnerSubsystem.stop();
    double seconds = Timer.getFPGATimestamp();
    int averageSampleTime = (int) totalSampleTime / totalSampleCount;
    System.out.println(totalSampleCount+":tsc, "+totalSampleTime+": tst, "+ averageSampleTime+": avgst, "+minSampleTime+": mst, "+maxSampleTime+": maxst." );
    System.out.println("xStart Time: "+startTime +";;;;; End Time: "+endTime+ " delta: "+(seconds-endTime));
    for (int i= 0; i< maxSamples; i++ ){


      if(histogram[i].count > 0){

      //System.out.println( i + ": "+histogram[i].count + " " + (histogram[i].noMatches)); /// histogram[i]));
      System.out.println(i + "," + histogram[i].count + "," + histogram[i].noMatches + "," + histogram[i].minSampleInterval + "," + histogram[i].maxSampleInterval + "," + histogram[i].avgsampleInterval);
      }
    }

    System.out.println("");


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Color currColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    if (currColor == Color.kBlack){
      blacksPerTransition++;
    }

    if ((currColor != Color.kBlack) && (currColor != initColor)){
      if(sampleCount > (maxSamples-1)){
        sampleCount = maxSamples - 1;
      }
      if (sampleCount < 0){
        sampleCount = 0;
      }

      histogram[sampleCount].count++;
      histogram[sampleCount].noMatches = blacksPerTransition;
      sampleCount = 0;
      transitionCount++;

      double seconds = Timer.getFPGATimestamp();

      histogram[sampleCount].count++;
      histogram[sampleCount].noMatches = blacksPerTransition;
      histogram[sampleCount].minSampleInterval = localMinSampleInterval;
      histogram[sampleCount].maxSampleInterval = localMaxSampleInterval;
      histogram[sampleCount].avgsampleInterval = localTotalSampleTime/localSampleCount;
      histogram[sampleCount].sampleCount = localSampleCount;

      localSampleCount = 0;
      localMinSampleInterval = 999999999;
      localMaxSampleInterval = 0;
      localAvgSampleInterval = 0;
      localTotalSampleTime = 0;
      localPrevTimeInterval = Timer.getFPGATimestamp();
  
      System.out.println(transitionCount + " transitions detected " + seconds + "color " + Robot.spinnerSubsystem.colorName(currColor));
      if (transitionCount > maxTransitions){
          Robot.spinnerSubsystem.setSpeed(currSpeed);
          Robot.spinnerSubsystem.spinCCw();
          return true;
      }
      else {
        initColor = currColor;
      }
    }

    double seconds = Timer.getFPGATimestamp();
    if (seconds > endTime ){
      //System.out.println("Start Time: "+startTime +";;;;; End Time: "+endTime+ " delta: "+(seconds-endTime));
      //return true;
    }
    return false;
  }
}
