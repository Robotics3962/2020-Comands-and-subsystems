/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveFaceLimelightTargetCmd extends CommandBase {
  boolean done = false;
  double rotateDirection = 1;

  /**
   * Creates a new DriveFaceLimelightCmd.
   */
  public DriveFaceLimelightTargetCmd() {
    addRequirements(Robot.robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double targetAcquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (targetAcquired != 1){
      done = true;
    }
    else {
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      if (tx < 0){
        rotateDirection = 1;
      }
      else {
        rotateDirection = -1;
      }
      done = false;
      double speed = calculateSpeed();
      Robot.robotDrive.setSpeedAndRotation(speed * rotateDirection, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    double speed = calculateSpeed();

    System.out.println(speed); 


    if (!done){
      Robot.robotDrive.setSpeedAndRotation(speed * rotateDirection, 0);
      

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
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      
      if ((tx > -.5) && (tx < .5)){
        done = true;

      }
    }
    return done;
  }

  public double calculateSpeed(){
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double error = 0-tx;
    double speed = error * RobotMap.Drive_Auto_Distance_Pval;
    double adjustedSpeed = limit(speed);

    return adjustedSpeed;
  }

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
