/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SpinnerStopCmd extends CommandBase {
  double secondsToStop;
  double startTime;
  double endTime;
  /**
   * Creates a new SpinnerStopCmd.
   */
  public SpinnerStopCmd(double pauseTime) {
    addRequirements(Robot.spinnerSubsystem);
    secondsToStop = pauseTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    endTime = startTime + secondsToStop;
    Robot.spinnerSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.spinnerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTime = Timer.getFPGATimestamp();
    if (currTime > endTime){
      return true;
    }
    return false;
  }
}
