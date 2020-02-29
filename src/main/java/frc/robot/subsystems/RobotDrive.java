/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotDrive extends SubsystemBase {

  private enum DriveModes {MANUAL, AUTO};

  private DriveModes driveMode = DriveModes.MANUAL;

  private WPI_TalonSRX  leftFrontTalonSRX = null;
  private WPI_TalonSRX  leftRearTalonSRX = null;
  private WPI_TalonSRX  rightFrontTalonSRX = null;
  private WPI_TalonSRX  rightRearTalonSRX = null;
  private SpeedControllerGroup leftMotors = null;
  private SpeedControllerGroup rightMotors = null;
  private ADXRS450_Gyro gyro = null;
  private DifferentialDrive differentialDrive = null;

  public RobotDrive(){
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();

    leftFrontTalonSRX = new WPI_TalonSRX(RobotMap.Drive_TalonLeftFront_ID);
    leftRearTalonSRX = new WPI_TalonSRX(RobotMap.Drive_TalonLeftRear_ID);

    rightFrontTalonSRX = new WPI_TalonSRX(RobotMap.Drive_TalonRightFront_ID);
    rightRearTalonSRX = new WPI_TalonSRX(RobotMap.Drive_TalonRightRear_ID);

    leftMotors = new SpeedControllerGroup(leftFrontTalonSRX, leftRearTalonSRX);
    rightMotors = new SpeedControllerGroup(rightFrontTalonSRX, rightRearTalonSRX);

    // tells the left side that is should be inverted so that we drive straight with
    // each side having positive motor values.
    rightFrontTalonSRX.setInverted(false); // 3
    rightRearTalonSRX.setInverted(false); // 4
    leftFrontTalonSRX.setInverted(true); // 2
    leftRearTalonSRX.setInverted(true); // 1

    //Config all talons
    Util.configTalon(rightFrontTalonSRX);
    Util.configTalon(rightRearTalonSRX);
    Util.configTalon(leftFrontTalonSRX);
    Util.configTalon(leftRearTalonSRX);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    gyro.calibrate();

    // configure the encoders
    leftRearTalonSRX.setSensorPhase(true);
  }

  public void setTankDriveSpeed( double speedLeft, double speedRight){
    differentialDrive.tankDrive(speedLeft, speedRight);
  }

  public void setTankDriveSpeedCapped(double speedLeft, double speedRight){
      double cappedSpeedLeft = speedLeft; 
      if (speedLeft < -RobotMap.Drive_SpeedScaleFactor){
        cappedSpeedLeft = -RobotMap.Drive_SpeedScaleFactor;
      }
      
      if (speedLeft > RobotMap.Drive_SpeedScaleFactor){
        cappedSpeedLeft = RobotMap.Drive_SpeedScaleFactor;
      }

      double cappedSpeedRight = speedRight;
      if (speedRight < -RobotMap.Drive_SpeedScaleFactor){
        cappedSpeedRight = -RobotMap.Drive_SpeedScaleFactor;
      }
      
      if (speedRight > RobotMap.Drive_SpeedScaleFactor){
        cappedSpeedRight = RobotMap.Drive_SpeedScaleFactor;
      }
      setTankDriveSpeed(cappedSpeedLeft, cappedSpeedRight);
  }

  /** 
   * 
   * NB: Oddly enought, when called, the params are reversed, so 
   * rotation is passed as the first parameter and speed is the second  
   * 
   * */
  public void setSpeedAndRotation(double speed, double rotation){
    differentialDrive.arcadeDrive(speed, rotation);
  }

  public void stop(){
    leftMotors.stopMotor();
    rightMotors.stopMotor();
  }

  public void setSpeedAndRotationScaled(double rotation, double speed){
    double scaledSpeed;
    double scaledRotation;
      
    scaledSpeed = speed * RobotMap.Drive_SpeedScaleFactor;
    scaledRotation = rotation * RobotMap.Drive_RotationScaleFactor;

    setSpeedAndRotation(scaledRotation, scaledSpeed);//orig Scaledspeed, ScaledRotation

  }

  public void MoveWithJoystick(){
    double speed = Robot.joystickControl.getLeftThrottle() * -1;
    double rotation = Robot.joystickControl.getRightRotation() * -1;

    setSpeedAndRotationScaled(rotation, speed);
  }

  @Override
  public void periodic(){

    dumpEncoderValues();

    // if we are in manual mode, allow the robot to be controlled
    // with the joystick
    if(driveMode == DriveModes.MANUAL){
      MoveWithJoystick();
    }
    else{

    }
  }

  public void dumpEncoderValues(){
    SmartDashboard.putNumber("right encoder val:", rightRearTalonSRX.getSelectedSensorPosition());
    SmartDashboard.putNumber("left encoder val:", leftRearTalonSRX.getSelectedSensorPosition());
    SmartDashboard.putNumber("angle:", readGyro());
  }

  public double readGyro(){
    return gyro.getAngle();
  }

  public double readLEncoder(){
    return leftRearTalonSRX.getSelectedSensorPosition();

  }

  public void resetGyro(){
    gyro.reset();
  }

  /**
   * 
   * @param ts
   * @return currently needs updated
   */
  public double normalizeSkew(double ts){

    return ts;
  }
}
