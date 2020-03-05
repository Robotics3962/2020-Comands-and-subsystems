/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ShooterShootCmd extends CommandBase {
  /**
   * Creates a new ShooterShootCmd.
   */
  public ShooterShootCmd() {
    addRequirements(Robot.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooterSubsystem.spinShooter(RobotMap.Shooter_Speed);
    Robot.shooterSubsystem.displayEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.shooterSubsystem.spinShooter(RobotMap.Shooter_Speed);
    Robot.shooterSubsystem.updaterEncoderRate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
