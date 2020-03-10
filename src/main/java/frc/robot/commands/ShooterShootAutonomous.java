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

public class ShooterShootAutonomous extends CommandBase {
  private enum States {
    INIT,
    SPIN_UP_FLYWHEEL,
    FLYWHEEL_SPUN_UP,
    SPIN_FEEDER
  };
  
  private States currState;
  private double flyWheelIsSpunUpTime;
  /**
   * Creates a new ShooterShootAutonomous.
   */
  public ShooterShootAutonomous() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooterSubsystem);

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
        if (currTime > flyWheelIsSpunUpTime){
          Robot.shooterSubsystem.spinFeedMotorCCW();
          currState = States.SPIN_FEEDER;
        }
        break;
      case SPIN_FEEDER:
        // final state, keep both wheels spinning
        break;
    }
    Robot.shooterSubsystem.updaterEncoderRate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooterSubsystem.stopShooter();
    Robot.shooterSubsystem.stopFeedMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
