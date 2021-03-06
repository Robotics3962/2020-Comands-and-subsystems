/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

/**
 * this command should be the first command run in telop or autonomouse mode.
 * It lowers the collector and extends the spinner arm
 */
public class DeployCmd extends InstantCommand {
  public DeployCmd() {
    addRequirements(Robot.intakeSubsystem);
    addRequirements(Robot.climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // lower the intake so it can collect balls
    Robot.intakeSubsystem.lower();

    // extend the spinner so it can spin the wheels
    Robot.spinnerSubsystem.extend();
  }
}
