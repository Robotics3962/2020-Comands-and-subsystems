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

  int transitionCount = 0;
  Color initColor = Color.kBlack;
  int maxTransitions;
  double initSpeed = RobotMap.Spinner_MotorSpeed;
  double currSpeed = initSpeed;
  double finalSpeed = RobotMap.Spinner_MotorSpeed;
  int boostSpeedPeriods = 1;
  int periodCount = 0;
  double startTime;
  int sampleCount;

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
    transitionCount = 0;
    initColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    startTime = Timer.getFPGATimestamp();
    periodCount = 0;
    sampleCount = 0;

    currSpeed = initSpeed;
    Robot.spinnerSubsystem.setSpeed(currSpeed);
    Robot.spinnerSubsystem.spinCw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    periodCount++;
    sampleCount++;
  
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Color currColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    Color detColor = Robot.spinnerSubsystem.getLastDetectedColor();
    double seconds = Timer.getFPGATimestamp();
    System.out.println("sample:" + sampleCount + " color:" + Robot.spinnerSubsystem.colorName(currColor) + " seconds:" + seconds + " detected r:" + detColor.red + " g:" + detColor.green + " b:" + detColor.blue);

    // we can't have a transition to or from black
    if ((currColor != Color.kBlack) && (initColor != Color.kBlack)){

      if (currColor != initColor){

        // we know what the next color should be.  If it isn't that color there is a bad match
        // so treat it as black
        Color nextColor = Robot.spinnerSubsystem.getNextColor(initColor);
        if (nextColor != currColor){
          System.out.println("last color match of " + Robot.spinnerSubsystem.colorName(currColor) + " was bad");
          currColor = Color.kBlack;
        }
        else {
          System.out.println(transitionCount + " transitions detected:" + seconds + " color:" + Robot.spinnerSubsystem.colorName(currColor));

          transitionCount++;
          initColor = currColor;
        }
      }
    }

    // We are done if we hit the transitions we wanted
    if (transitionCount >= maxTransitions){
      return true;
    }
    
    return false;
  }
}
