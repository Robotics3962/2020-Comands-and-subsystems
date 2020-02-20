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
import frc.robot.RobotMap;

public class DriverMoveDistancePIDCmd extends CommandBase {
  double initialPosition;
  double targetPosition;
  double distance;
  double aggregatedError;
  double prevError;

  /**
   * Creates a new DriverMoveDistance1Cmd.
   */
  public DriverMoveDistancePIDCmd(double distanceToMove) {
    addRequirements(Robot.robotDrive);
    distance = distanceToMove;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aggregatedError = 0;
    prevError = 0;
    double currentPosition = initialPosition = Robot.robotDrive.readLEncoder();
    targetPosition = initialPosition - (distance * RobotMap.Drive_Auto_CountsPerInch);
    SmartDashboard.putNumber("current position:", currentPosition);
    SmartDashboard.putNumber("final position:", targetPosition);
    System.out.println("distance:" + distance);
    System.out.println("MoveToPos: targetPos:" + targetPosition + " currpos:" + currentPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = calculateSpeed();
    double currentPosition = Robot.robotDrive.readLEncoder();
    updateDashboard(currentPosition, targetPosition, speed);
    //System.out.println("MoveToPos: targetPos:" + targetPosition + " currpos:" + currentPosition);

    // for some reason speed and rotation is reversed
    Robot.robotDrive.setSpeedAndRotation(0, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotDrive.stop();
    double currentPosition = Robot.robotDrive.readLEncoder();
    updateDashboard(currentPosition, targetPosition, 0);
    System.out.println("MoveToPos: targetPos:" + targetPosition + " currpos:" + currentPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reachedTargetPosition();
  }

  private double calculateSpeed(){
    double speed = 0;
    double currentPosition = Robot.robotDrive.readLEncoder();
    double error = targetPosition - currentPosition;
    aggregatedError += error;
    double deltaError = prevError - error;
    prevError = error;

    double pidPval = error * RobotMap.Drive_Auto_Distance_Pval;
    double pidIval = aggregatedError * RobotMap.Drive_Auto_Distance_Ival;
    double pidDval = deltaError * RobotMap.Drive_Auto_Distance_Dval;

    speed = pidPval + pidIval + pidDval;
    double adjustedSpeed = limit(speed);

    return adjustedSpeed;
  }

  private boolean reachedTargetPosition(){
    boolean done = false;
    double currentPosition = Robot.robotDrive.readLEncoder();
    if ((currentPosition < (targetPosition + RobotMap.Drive_Auto_Distance_DeadZone))
      && ((currentPosition > (targetPosition - RobotMap.Drive_Auto_Distance_DeadZone)))){
        done = true;
    }    

    return done;
  }

  private double limit(double value){
    if (value <= 0){
      if (value > -RobotMap.Drive_Auto_Distance_MinSpeed){
        value = -RobotMap.Drive_Auto_Distance_MinSpeed;
      } 
      else if (value < -RobotMap.Drive_Auto_Distance_MaxSpeed) {
        value = -RobotMap.Drive_Auto_Distance_MaxSpeed;
      }
    }
    else{
      if (value <= RobotMap.Drive_Auto_Distance_MinSpeed){
        value = RobotMap.Drive_Auto_Distance_MinSpeed;
      }
      else if (value > RobotMap.Drive_Auto_Distance_MaxSpeed) {
        value = RobotMap.Drive_Auto_Distance_MaxSpeed;
      }
    }

    return value;
  }

  private void updateDashboard(double currentPos, double targetPos, double speed){
    SmartDashboard.putNumber("current position:", currentPos);
    SmartDashboard.putNumber("target position:", targetPos);
    SmartDashboard.putNumber("drive speed:", speed);

  }
}
