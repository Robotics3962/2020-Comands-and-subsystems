/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DriveMoveDistanceCmd extends CommandBase {
  /**
   * Creates a new DriveMoveDistanceCmd.
   */
  
  double initialPosition;
  final double countsPerInch = 218;
  double finalPosition;

  public DriveMoveDistanceCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.robotDrive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RobotDriveMoveDist: init");
    initialPosition = Robot.robotDrive.readLEncoder();
    finalPosition = initialPosition - (24* countsPerInch);
    Robot.robotDrive.setSpeedAndRotation(0, 0.5);
    System.out.println("RobotDriveMoveDist: moving from " + initialPosition + " to " + finalPosition);
    SmartDashboard.putNumber("init position:", initialPosition);
    SmartDashboard.putNumber("final position:", finalPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.robotDrive.setSpeedAndRotation(0, 0.5);
    SmartDashboard.putNumber("init position:", initialPosition);
    SmartDashboard.putNumber("final position:", finalPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("RobotDriveMoveDist: end");
    Robot.robotDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double curPosition = Robot.robotDrive.readLEncoder();
    if (finalPosition < curPosition ){
      return false;
    }

    return true;
    
  }
}
