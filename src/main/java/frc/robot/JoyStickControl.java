package frc.robot;

/**
 * This file contains the mappings between the joystick buttons 
 * and the command they will run
 */

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.SpinnerGetSensorColorCmd;
import frc.robot.commands.SpinnerExtendCmd;
import frc.robot.commands.SpinnerRetractCmd;
import frc.robot.commands.SpinnerSpinCmd;
import frc.robot.commands.SpinnerSpinReverseCmd;
import frc.robot.commands.IntakeSpinCmd;
import frc.robot.commands.IntakeDeployCmd;
import frc.robot.commands.LiftIndexCmd;
import frc.robot.commands.LiftRunCmd;
import frc.robot.commands.ShooterShootCmd;
import frc.robot.commands.DriveMoveDistanceCmd;
import frc.robot.commands.DriveMoveDistancePIDCmd;
//import frc.robot.commands.SpinnerMove1TransitionCmd;
import frc.robot.commands.DriveResetGyroCmd;
import frc.robot.commands.DriveRotatePIDCmd;
import frc.robot.commands.DriveSeekLimelightTargetCmd;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DemoAutonomousCmdGroup;
import frc.robot.commands.DriveFaceLimelightTargetCmd;

public class JoyStickControl {
  // get both drive and operational joysticks
  public static Joystick driveJoystick = new Joystick(RobotMap.Joystick0Id);
  static Joystick operationJoyStick = new Joystick(RobotMap.Joystick1Id); 
  
  public JoyStickControl(){
    
    // get the buttons on the drive joystick
    JoystickButton driveButtonA = new JoystickButton(driveJoystick, Button.kA.value);
    JoystickButton driveButtonB = new JoystickButton(driveJoystick, Button.kB.value);
    JoystickButton driveButtonX = new JoystickButton(driveJoystick, Button.kX.value);
    JoystickButton driveButtonY = new JoystickButton(driveJoystick, Button.kY.value);
    JoystickButton driveButtonLS = new JoystickButton(driveJoystick, Button.kBumperLeft.value);
    JoystickButton driveButtonRS = new JoystickButton(driveJoystick, Button.kBumperRight.value);
    JoystickButton driveButtonBack = new JoystickButton(driveJoystick, Button.kBack.value);
    JoystickButton driveButtonStart = new JoystickButton(driveJoystick, Button.kStart.value);

    // assign drive joystick buttons to commands
    //driveButtonA.whileHeld(new SpinnerEnableDisplayColorInfoCmd());
    //driveButtonBack.whileHeld(new GrabBallCmd());


    // second joystick I'm calling it operational - no command mapping yet
    JoystickButton opButtonA = new JoystickButton(operationJoyStick, Button.kA.value);
    JoystickButton opButtonB = new JoystickButton(operationJoyStick, Button.kB.value);
    JoystickButton opButtonX = new JoystickButton(operationJoyStick, Button.kX.value);
    JoystickButton opButtonY = new JoystickButton(operationJoyStick, Button.kY.value);
    JoystickButton opButtonLS = new JoystickButton(operationJoyStick, Button.kBumperLeft.value);
    JoystickButton opButtonRS = new JoystickButton(operationJoyStick, Button.kBumperRight.value);
    JoystickButton opButtonBack = new JoystickButton(operationJoyStick, Button.kBack.value);
    JoystickButton opButtonStart = new JoystickButton(operationJoyStick, Button.kStart.value);

    //opButtonA.whileHeld(new SpinnerSpinReverseCmd());
    //opButtonB.whileHeld(new SpinnerSpinCmd());
    opButtonA.whenPressed(new DriveMoveDistancePIDCmd(60));
    opButtonB.whenPressed(new DriveMoveDistancePIDCmd(-60));
    opButtonX.whenPressed(new DemoAutonomousCmdGroup());
    opButtonY.whenPressed(new DriveResetGyroCmd());
    opButtonStart.whenPressed(new DriveRotatePIDCmd(180));
    opButtonBack.whenPressed(new DriveRotatePIDCmd(0));
    //opButtonBack.whenPressed(new SpinnerMove1TransitionCmd(25));

    driveButtonA.whenPressed(new DriveSeekLimelightTargetCmd());
    driveButtonB.whenPressed(new DriveFaceLimelightTargetCmd());
  }
    
  public double getLeftThrottle() {
		return driveJoystick.getY(); 
	}	

	public double getRightRotation() {
        return driveJoystick.getRawAxis(4);
    }

  public static boolean deadManSwitch() {
    boolean active = false;
    active = operationJoyStick.getRawButton(Button.kBumperLeft.value);
    SmartDashboard.putBoolean("deadman switch:", active);
    active = true;
    return active;
  }
}
