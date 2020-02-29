/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveSeekLimelightTargetCmd extends CommandBase {

  private double initialGyroAngle;
  private double finalGyroAngle;
  private boolean limelightFoundTarget;

  /**
   * Creates a new DriveSeekLimelightTargetCmd.
   */
  public DriveSeekLimelightTargetCmd() {
    addRequirements(Robot.robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialGyroAngle = Robot.robotDrive.readGyro();
    // we do not want to rotate more than 360 degrees.
    finalGyroAngle = initialGyroAngle + 360;
    limelightFoundTarget = false;
    Robot.robotDrive.setSpeedAndRotation(RobotMap.Drive_LimeLight_Search_RotateSpeed, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotMap.Drive_LimeLight_Search_RotateSpeed;
    //Robot.robotDrive.setTankDriveSpeed(-speed, speed);
    Robot.robotDrive.setSpeedAndRotation(RobotMap.Drive_LimeLight_Search_RotateSpeed, 0);
    //System.out.println("Searching for target");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = false;

    // if the limelight found a target we are done
    double targetAcquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (targetAcquired == 1){
      limelightFoundTarget = true;
      done = true;
    }

    // if the robot rotated over 360 degrees, we are done because
    // no target was there to locate
    if (Robot.robotDrive.readGyro() > finalGyroAngle){
      done = true;
    }

    SmartDashboard.putBoolean("LimeLight targeted", limelightFoundTarget);
    //System.out.println("is finished:");
    return done;
  }
}
