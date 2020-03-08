/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveDistLimelightTargetCmd extends CommandBase {
  boolean done = false;
  double timeStabilized;
  int msgCount = 0;
  int hasNotMovedCounter = 0;
  double prevDist;
  double currSpeed;
  private double target = 9.0;
  private double minTargetRange = target - .20;
  private double maxTargetRange = target + .20;
  //private double minTargetRange = target - RobotMap.Drive_Limelight_Search_Tolerance;
  //private double maxTargetRange = target + RobotMap.Drive_Limelight_Search_Tolerance;

  /**
   * Creates a new DriveFaceLimelightCmd.
   */
  public DriveDistLimelightTargetCmd() {
    addRequirements(Robot.robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    msgCount = 0;
    hasNotMovedCounter = 0;
    prevDist = Robot.robotDrive.readGyro();
    timeStabilized = Timer.getFPGATimestamp() + RobotMap.Drive_Auto_StabilizedTime;

    double targetAcquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (targetAcquired != 1){
      done = true;
    }
    else {
      done = false;
      currSpeed = calculateSpeed();
      Robot.robotDrive.setSpeedAndRotation(0, currSpeed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  currSpeed = calculateSpeed();
    if (!done){
      Robot.robotDrive.setSpeedAndRotation(0, currSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(!done){
      double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

      // handle the case where are in the target range
      if ((ty > minTargetRange) && (ty < maxTargetRange)){
       
        hasNotMovedCounter = 0;

        /**
         * We need to know when we converged in the target range.
         * To do this we need to stay in the target range for a 
         * set period of time.  This avoids the case where the robot
         * is just passing through the target range and not stopping
         * in it.
         * 
         * To accomplish this we keep track of the furthest time in the future
         * that we would need to stay in the target area for.
         * 
         * If we are in the target area we check to see if the
         * time stabilized is in the past.  If it is, we are done.
         * 
         * If we are not in the target area, we bump out the time we can 
         * be stabilized to even further in the future.
         */
        double currTime = Timer.getFPGATimestamp();
        if (currTime > timeStabilized){
          done = true;
          System.out.println("Isfinished(DriveDist):::::::::::::: Dones");
        }
      }
      else {
        timeStabilized = Timer.getFPGATimestamp() + RobotMap.Drive_Auto_StabilizedTime;

        double minRange = ty - .25;
        double maxRange = ty + .25;
        if ((prevDist >= minRange) && (prevDist <= maxRange)) {
          hasNotMovedCounter++;

          if (hasNotMovedCounter > RobotMap.Drive_Limelight_NoProgressCnt) {
            //done = true;
          }         
        }
        else {
          prevDist = ty;
        }
      }
    }
    return done;
  }

  public double calculateSpeed(){
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double error = target-ty;
    double speed = error * RobotMap.Drive_Auto_Distance_Pval;
    double adjustedSpeed = limit(speed);

    SmartDashboard.putNumber("Ty", ty);
    SmartDashboard.putNumber("tY Error", error);
    SmartDashboard.putNumber("ty adjusted speed", adjustedSpeed);

    return adjustedSpeed;
  }

  private double limit(double value){
    if (value <= 0){
      if (value >= -RobotMap.Drive_Auto_Distance_MinSpeed){
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

}
