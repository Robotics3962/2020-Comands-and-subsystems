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
  double startTime;
  double endTime;

  public SpinnerMove1TransitionCmd(double numTransitions) {//set to seconds
    addRequirements(Robot.spinnerSubsystem);
    maxTransitions = numTransitions;
    periodCount = 0;
    currSpeed = initSpeed;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transitionCount = 0;
    initColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    periodCount = 0;
    currSpeed = initSpeed;
    Robot.spinnerSubsystem.setSpeed(currSpeed);
    Robot.spinnerSubsystem.spinCw();
    startTime = Timer.getFPGATimestamp();
    endTime = startTime + maxTransitions * 0.25;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
   // System.out.println("startTime:"+startTime+" |||||| Endtime: "+endTime);
    periodCount++;
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
    System.out.println("xStart Time: "+startTime +";;;;; End Time: "+endTime+ " delta: "+(seconds-endTime));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Color currColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    if ((currColor != Color.kBlack) && (currColor != initColor)){
      //transitionCount++;
      double seconds = Timer.getFPGATimestamp();
     // System.out.println(transitionCount + " transitions detected " + seconds + "color " + Robot.spinnerSubsystem.colorName(currColor));
      //if (transitionCount > maxTransitions){
          //Robot.spinnerSubsystem.setSpeed(currSpeed);
        //  Robot.spinnerSubsystem.spinCCw();
          //return true;
      //}
      //else {
        //initColor = currColor;
      //}
    }

    double seconds = Timer.getFPGATimestamp();
    if (seconds > endTime ){
      System.out.println("Start Time: "+startTime +";;;;; End Time: "+endTime+ " delta: "+(seconds-endTime));
      return true;
    }
    return false;
  }
}
