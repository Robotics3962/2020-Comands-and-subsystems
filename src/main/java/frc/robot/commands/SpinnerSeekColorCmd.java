/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SpinnerSeekColorCmd extends CommandBase {
  Color colorToSeek;
  /**
   * Creates a new SpinnerSeekColorCmd.
   */
  public SpinnerSeekColorCmd(String color) {
    addRequirements(Robot.spinnerSubsystem);
    colorToSeek = Robot.spinnerSubsystem.nameToColor(color);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.spinnerSubsystem.setSpeed(RobotMap.Spinner_SlowMotorSpeed);    
    Robot.spinnerSubsystem.spinCw();
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
    Color currColor = Robot.spinnerSubsystem.getMatchedSensorColor();
    if (currColor == colorToSeek){
      return true;
    }
    return false;
  }
}
