package frc.robot;

/**
 * This file contains the mappings between the joystick buttons 
 * and the command they will run
 */

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;
import frc.robot.commands.SpinnerEnableDisplayColorInfoCmd;
import frc.robot.commands.SpinnerDisableDisplayColorCmd;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button;

public class JoyStickControl {
  // get both drive and operational joysticks
  Joystick driveJoystick = null;
  Joystick operationJoyStick = null; 
  
  public JoyStickControl(){
    Joystick driveJoystick = new Joystick(RobotMap.Joystick0Id);
    Joystick operationJoyStick = new Joystick(RobotMap.Joystick1Id);
    
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
    driveButtonA.whileHeld(new SpinnerEnableDisplayColorInfoCmd());
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

    //opButtonA.whenPressed(new LockWristCmd());
    //opButtonLS.whileHeld(new ElevatorDownCmd());
  }
    
  public double getLeftThrottle() {
		return driveJoystick.getY(); 
	}	

	public double getRightRotation() {
        return driveJoystick.getRawAxis(4);
    }
}
