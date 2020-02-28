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
import frc.robot.RobotMap;

public class LiftIndexCmd extends CommandBase {

  /**
   * Command to shif the balls one position up in the lift
   */

  private final int runMotorTime = RobotMap.Lift_IndexTimeMilliSeconds;
  private double endTime;

  /**
   * Creates a new LiftIndexCmd.
   */
  public LiftIndexCmd() {
    addRequirements(Robot.liftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTime = Timer.getFPGATimestamp() + runMotorTime;
    Robot.liftSubsystem.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.liftSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currTime = Timer.getFPGATimestamp();
    if (currTime < endTime){
      return false;
    }
    return true;
  }
}
