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

public class DriveRotatePIDCmd extends CommandBase {
  double initialAngle;
  double targetAngle;
  double distance;
  double aggregatedError;
  double prevError;

  /**
   * Creates a new DriveRotatePIDCmd.
   */
  public DriveRotatePIDCmd(double absoluteAngle) {
    addRequirements(Robot.robotDrive);
    targetAngle =  absoluteAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aggregatedError = 0;
    prevError = 0;
    initialAngle = Robot.robotDrive.readGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = calculateSpeed();
    // for some reason speed and rotation is reversed
    Robot.robotDrive.setSpeedAndRotation(0, speed);

    double currentAngle = Robot.robotDrive.readGyro();
    SmartDashboard.putNumber("current angle:", currentAngle);
    SmartDashboard.putNumber("target angle:", targetAngle);
    SmartDashboard.putNumber("angular speed:", speed);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reachedTargetAngle();
  }

  // simple pid calculator.  I don't think we need the I or D
  // parameters set to anything but 0
  private double calculateSpeed(){
    double speed = 0;
    double currentAngle = Robot.robotDrive.readGyro();
    double error = targetAngle - currentAngle;
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

  // if the current position is within the target position +/- deadzone
  // we are done
  private boolean reachedTargetAngle(){
    boolean done = false;
    double currentAngle = Robot.robotDrive.readGyro();
    if ((currentAngle < (targetAngle + RobotMap.Drive_Auto_Angle_DeadZone))
      && ((currentAngle > (targetAngle - RobotMap.Drive_Auto_Angle_DeadZone)))){
        done = true;
    }    

    return done;
  }

  // constrain the speed that we are comfortable the robot rotating
  private double limit(double value){
    if (value <= 0){
      if (value >= -RobotMap.Drive_Auto_Angle_MinSpeed){
        value = -RobotMap.Drive_Auto_Angle_MinSpeed;
      } 
      else if (value < -RobotMap.Drive_Auto_Angle_MaxSpeed) {
        value = -RobotMap.Drive_Auto_Angle_MaxSpeed;
      }
    }
    else{
      if (value <= RobotMap.Drive_Auto_Angle_MinSpeed){
        value = RobotMap.Drive_Auto_Angle_MinSpeed;
      }
      else if (value > RobotMap.Drive_Auto_Angle_MaxSpeed) {
        value = RobotMap.Drive_Auto_Angle_MaxSpeed;
      }
    }
    return value;
  }

}
