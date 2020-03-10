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

public class AutoShootBallsCmd extends CommandBase {
  private enum States {
    INIT,
    SPIN_UP_FLYWHEEL,
    FLYWHEEL_SPUN_UP,
    SPIN_FEEDEER,
    FEEDER_SPUN_UP,
    SPIN_LIFT,
    ALL_BALLS_SHOT,
    SHOOTING_COMPLETE
  };

  private States currState;
  private double flyWheelIsSpunUpTime;
  private double feederIsSpunUpTime;
  private double allBallsShotTime;

  /**
   * Creates a new ShooterShootCmd.
   */
  public AutoShootBallsCmd() {
    addRequirements(Robot.shooterSubsystem);
    addRequirements(Robot.liftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currState = States.INIT;
    Robot.shooterSubsystem.spinShooter(RobotMap.Shooter_Speed);
    Robot.shooterSubsystem.displayEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Timer.getFPGATimestamp();
    switch (currState){
      case INIT:
        // start state, nothing to do here so fall through
      case SPIN_UP_FLYWHEEL:
        Robot.shooterSubsystem.spinShooter(RobotMap.Shooter_Speed);
        flyWheelIsSpunUpTime = currTime + RobotMap.Shooter_TimeForFlyWheelToSpinUp;
        currState = States.FLYWHEEL_SPUN_UP;
        break;
      case FLYWHEEL_SPUN_UP:
        // wait for the shooter wheel to be spun up
        if (currTime > flyWheelIsSpunUpTime){
          currState = States.SPIN_FEEDEER;
        }
        break;
      case SPIN_FEEDEER:
        // spin feeder wheel to feed balls into the shooter
        // first ball will shoot here
        Robot.shooterSubsystem.spinFeedMotorCCW();
        feederIsSpunUpTime = currTime + RobotMap.Shooter_TimeForFeederToSpinUp;
        currState = States.FEEDER_SPUN_UP;
        break;
      case FEEDER_SPUN_UP:
        // wait short time for ball to be shot
        if (currTime > feederIsSpunUpTime) {
          currState = States.SPIN_LIFT;
        }
        break;
      case SPIN_LIFT:
        // start the ball lift to push balls from the
        // ball lift into the feeder motor
        Robot.liftSubsystem.start(); // could be reverse instead of start
                                     // if the direction is wrong
        allBallsShotTime = currTime + RobotMap.Shooter_TimeToShootAllBalls;
        currState = States.ALL_BALLS_SHOT;
        break;
      case ALL_BALLS_SHOT:
        // wait a period of time for all balls to shoot
        if (currTime > allBallsShotTime){
          Robot.liftSubsystem.stop();
          Robot.shooterSubsystem.stopFeedMotor();
          Robot.shooterSubsystem.stopShooter();
          currState = States.SHOOTING_COMPLETE;
        }
        break;
      case SHOOTING_COMPLETE:
        // final state, do nothing
    }
    Robot.shooterSubsystem.updaterEncoderRate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooterSubsystem.stopShooter();
    Robot.shooterSubsystem.stopFeedMotor();
    Robot.liftSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // the command is finished when the state machine is in the final
    // (terminal) state
    if (currState == States.SHOOTING_COMPLETE){
      return true;
    }
    else {
      return false;
    }
  }
}
